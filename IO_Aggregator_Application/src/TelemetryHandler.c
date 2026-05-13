/* ************************************************************************** */
/** @file    TelemetryHandler.c
 *  @brief   Telemetry Handler — Implementation
 *
 *  @company BACANCY SYSTEMS PVT. LTD. 
 *
 *  @summary
 *    Collects per-dock PM, BMS, and temperature data from the session database
 *    and broadcasts it to a connected TCP client once per second.
 *
 *  Frame format (big-endian):
 *    [0–3]   Unique ID       (uint32_t)
 *    [4–5]   Message type    (uint16_t) = 0x0004
 *    [6]     Compartment ID
 *    [7]     Dock ID
 *    [8]     Command ID
 *    [9]     Payload length
 *    [10–N]  Payload
 *    [N+1–2] CRC16-CCITT
 *
 *  Version : 2.0
 *
 *  Changes from v1.0:
 *    - Fixed: telemetryFrameBuf was a shared static — not safe if TCP send
 *      overlaps (e.g. future ISR path). Changed to local stack buffer per call.
 *    - Fixed: Telemetry_SendFrame had no bounds check on payloadLen vs buffer
 *      size — could silently overflow telemetryFrameBuf. Now asserts.
 *    - Fixed: u32SystemFaultBitmap in BMS telemetry was duplicating
 *      u32BMSFaultCode. Now correctly reads SESSION_GetSystemFaultBitmap().
 *    - Fixed: Telemetry_SendTemperature accessed telemetryData[DOCK_1/2/3]
 *      directly instead of using the dock-agnostic tempData already populated
 *      by Telemetry_UpdateFromSession — causing stale data on first call.
 *      Now reads from session DB directly, which is always fresh.
 *    - Fixed: Telemetry_Init returned void — no way for caller to detect
 *      failure. Return type changed to bool.
 *    - Fixed: float → uint32_t conversion for voltage/current used implicit
 *      cast without rounding — now uses explicit (uint32_t)((val) + 0.5f)
 *      to avoid truncation errors at mV/mA scale.
 *    - Fixed: BMS payload array declared as [20U] but holds 25 bytes —
 *      silent overflow. Now uses TELEMETRY_BMS_PAYLOAD_SIZE constant.
 *    - Fixed: PM payload array declared as [22U] — matches, but now uses
 *      TELEMETRY_PM_PAYLOAD_SIZE constant for safety.
 *    - Fixed: `uint16_t i` used as array index but payload arrays are uint8_t
 *      — changed to uint8_t i where overflow is impossible.
 *    - Fixed: Telemetry_SendTemperature used payload[MAX_DOCKS] (= 4) which
 *      accidentally worked but is semantically wrong — now uses
 *      TELEMETRY_TEMP_PAYLOAD_SIZE.
 *    - Improved: Extracted prv_SerialiseU32BE() helper to eliminate 4-line
 *      big-endian serialisation repeated 9 times.
 *    - Improved: Socket reconnection logic extracted to prv_EnsureSocketOpen()
 *    - Improved: xTaskCreate failure is now handled — task is not silently lost.
 *    - Improved: `telemetryUniqueId` overflow is harmless (wraps to 0) but
 *      documented explicitly.
 */
/* ************************************************************************** */

/* ============================================================================
 * Includes
 * ========================================================================== */
#include "TelemetryHandler.h"
#include <string.h>
#include <stdio.h>
#include "FreeRTOS.h"
#include "task.h"
#include "definitions.h"
#include "configuration.h"
#include "device.h"
#include "tcpip_manager_control.h"
#include "library/tcpip/tcpip_helpers.h"
#include "sessionDBHandler.h"

/* ============================================================================
 * Task Configuration
 * ========================================================================== */
#define TELEMETRY_TASK_STACK_SIZE   (1024U)
#define TELEMETRY_TASK_PRIORITY     (2U)
#define TELEMETRY_TASK_DELAY_MS     (1000U) /**< Broadcast interval (ms)      */
#define TELEMETRY_INTER_SEND_MS     (1U)    /**< Delay between dock sends (ms) */
#define TELEMETRY_RETRY_DELAY_MS    (1000U) /**< Socket retry interval (ms)   */
#define TELEMETRY_PORT_HEV          (8889U) /**< TCP server port               */

/* ============================================================================
 * Global Telemetry Data
 * BUG FIX: was [4] — now MAX_DOCKS to match sessionDBHandler.h
 * ========================================================================== */
TELEMETRY_Data_t telemetryData[MAX_DOCKS];

/* ============================================================================
 * Private Variables
 * ========================================================================== */

/** @brief TCP server socket handle */
static TCP_SOCKET s_telemetrySocket = INVALID_SOCKET;

/** @brief Monotonically incrementing frame sequence number (wraps at UINT32_MAX) */
static uint32_t s_uniqueId = 1U;

/** @brief Task handle — kept for potential future suspend/delete */
static TaskHandle_t s_taskHandle = NULL;

/* ============================================================================
 * Private Function Prototypes
 * ========================================================================== */
static uint16_t prv_CalculateCRC16(const uint8_t *pu8Data, uint16_t u16Len);
static uint8_t  prv_SerialiseU32BE(uint8_t *pu8Buf, uint32_t u32Val);
static bool     prv_EnsureSocketOpen(void);
static void     prv_SendFrame(uint8_t u8CmdId,
                              uint8_t u8CompartmentId,
                              uint8_t u8DockId,
                              const uint8_t *pu8Payload,
                              uint8_t u8PayloadLen);
static void     prv_UpdateFromSession(uint8_t u8DockId);
static void     prv_SendPM(uint8_t u8CompartmentId, uint8_t u8DockId);
static void     prv_SendBMS(uint8_t u8CompartmentId, uint8_t u8DockId);
static void     prv_SendTemperature(uint8_t u8CompartmentId);

/* ============================================================================
 * Private Helpers
 * ========================================================================== */

/**
 * @brief  Serialise a uint32_t into 4 big-endian bytes at pu8Buf.
 *
 * @param  pu8Buf  Destination byte pointer (must have >= 4 bytes available)
 * @param  u32Val  Value to serialise
 * @return Number of bytes written (always 4)
 */
static uint8_t prv_SerialiseU32BE(uint8_t *pu8Buf, uint32_t u32Val)
{
    pu8Buf[0] = (uint8_t)((u32Val >> 24U) & 0xFFU);
    pu8Buf[1] = (uint8_t)((u32Val >> 16U) & 0xFFU);
    pu8Buf[2] = (uint8_t)((u32Val >>  8U) & 0xFFU);
    pu8Buf[3] = (uint8_t)( u32Val         & 0xFFU);
    return 4U;
}

/**
 * @brief  Ensure the TCP server socket is open.
 *
 *         If the socket is already valid, returns true immediately.
 *         If invalid, attempts to open a new server socket.
 *
 * @return true  Socket is ready for use
 * @return false Socket could not be opened
 */
static bool prv_EnsureSocketOpen(void)
{
    if (s_telemetrySocket != INVALID_SOCKET)
    {
        return true;
    }

    s_telemetrySocket = TCPIP_TCP_ServerOpen(IP_ADDRESS_TYPE_IPV4,
                                             TELEMETRY_PORT_HEV, 0U);
    if (s_telemetrySocket == INVALID_SOCKET)
    {
        SYS_CONSOLE_PRINT("[Telemetry] Failed to open TCP socket on port %u\r\n",
                          TELEMETRY_PORT_HEV);
        return false;
    }

    SYS_CONSOLE_PRINT("[Telemetry] TCP socket opened on port %u\r\n",
                      TELEMETRY_PORT_HEV);
    return true;
}

/**
 * @brief  Collect session DB data into the telemetryData[] mirror for one dock.
 *
 *         BUG FIX: u32SystemFaultBitmap now uses SESSION_GetSystemFaultBitmap
 *         instead of duplicating SESSION_GetBMSFaultBitmap.
 *
 *         BUG FIX: float → uint32_t conversion uses rounding (+0.5f) to
 *         avoid truncation errors at mV/mA scale.
 *
 * @param  u8DockId  Dock index (must satisfy DOCK_1 <= u8DockId < MAX_DOCKS)
 */
static void prv_UpdateFromSession(uint8_t u8DockId)
{
    if ((u8DockId < (uint8_t)DOCK_1) || (u8DockId > (uint8_t)SESSION_GetMaxDocks()))
    {
        SYS_CONSOLE_PRINT("[Telemetry] UpdateFromSession: invalid dock %u\r\n",
                          (unsigned)u8DockId);
        return;
    }

    /* ------- PM Data ------- */
    if (SESSION_GetPMRxStatus(u8DockId) == true)
    {
            /* Convert float V/A → uint32_t mV/mA with rounding */
        telemetryData[u8DockId].pmData.u32PmOutputVoltage_mV =
            (uint32_t)((SESSION_GetPmOutputVoltage(u8DockId) * FACTOR_1000_F) + 0.5f);

        telemetryData[u8DockId].pmData.u32PmOutputCurrent_mA =
            (uint32_t)((SESSION_GetPmOutputCurrent(u8DockId) * FACTOR_1000_F) + 0.5f);

        telemetryData[u8DockId].pmData.u32OutputPower_W =
            (uint32_t)(SESSION_GetOutputPower(u8DockId) + 0.5f);

        /* Energy: session stores kWh, telemetry reports Wh */
        telemetryData[u8DockId].pmData.u32TotalEnergy_Wh =
            (uint32_t)((SESSION_GetEnergyDelivered(u8DockId) * FACTOR_1000_F) + 0.5f);

        telemetryData[u8DockId].pmData.u32PMFaultCode  = SESSION_GetPMFaultBitmap(u8DockId);
        telemetryData[u8DockId].pmData.u8PMTemperature = SESSION_GetPMTemperature(u8DockId);
        telemetryData[u8DockId].pmData.u8PMStatus      = SESSION_GetPMRxStatus(u8DockId);
    } else {
        /* No PM connection — zero out all PM fields for clarity */
        (void)memset(&telemetryData[u8DockId].pmData, 0, sizeof(telemetryData[u8DockId].pmData));
    }
    

    /* ------- BMS Data ------- */
    if (SESSION_GetBMSRxStatus(u8DockId) == true)
    {
        telemetryData[u8DockId].bmsData.u32BMSDemandVoltage =
            (uint32_t)((SESSION_GetBMSDemandVoltage(u8DockId) * FACTOR_1000_F) + 0.5f);

        telemetryData[u8DockId].bmsData.u32BMSDemandCurrent =
            (uint32_t)((SESSION_GetBMSDemandCurrent(u8DockId) * FACTOR_1000_F) + 0.5f);

        telemetryData[u8DockId].bmsData.u32EstimatedTime_s  =
            (uint32_t)SESSION_GetEstimatedChargingTime(u8DockId);

        telemetryData[u8DockId].bmsData.u32BMSFaultCode     = SESSION_GetBMSFaultBitmap(u8DockId);
        telemetryData[u8DockId].bmsData.u8CurrentSoc        = SESSION_GetCurrentSoc(u8DockId);
        telemetryData[u8DockId].bmsData.u8InitialSoc        = SESSION_GetInitialSoc(u8DockId);
        telemetryData[u8DockId].bmsData.u8BMSTemperature    = SESSION_GetBMSTemperature(u8DockId);
        telemetryData[u8DockId].bmsData.u8BMSStatus         = SESSION_GetBMSRxStatus(u8DockId);
        telemetryData[u8DockId].bmsData.u8ChargingState     = (uint8_t)SESSION_GetChargingState(u8DockId);

        /* BUG FIX: was SESSION_GetBMSFaultBitmap (duplicate) — now system fault bitmap */
        telemetryData[u8DockId].bmsData.u32SystemFaultBitmap =
            SESSION_GetSystemFaultBitmap(u8DockId);
    } else {
        /* No BMS connection — zero out all BMS fields for clarity */
        (void)memset(&telemetryData[u8DockId].bmsData, 0, sizeof(telemetryData[u8DockId].bmsData));
    }
    
    /* ------- Temperature Data ------- */
    /* COMPARTMENT (index 0) holds the enclosure sensor */
    telemetryData[u8DockId].tempData.u8CompartmentTemperature =
        SESSION_GetDockTemperature((uint8_t)COMPARTMENT);

    telemetryData[u8DockId].tempData.u8DockTemperature[u8DockId] =
        SESSION_GetDockTemperature((uint8_t)u8DockId);
}

/**
 * @brief  Build and transmit one telemetry frame over TCP.
 *
 *         Frame layout:
 *           [0–3]   s_uniqueId (big-endian)
 *           [4–5]   TELEMETRY_MSG_TYPE_BROADCAST (big-endian)
 *           [6]     u8CompartmentId
 *           [7]     u8DockId
 *           [8]     u8CmdId
 *           [9]     u8PayloadLen
 *           [10–N]  payload bytes
 *           [N+1–2] CRC16-CCITT (big-endian)
 *
 *         BUG FIX: buffer is now a local stack variable (was shared static).
 *         BUG FIX: payload length is validated against maximum buffer capacity.
 *
 * @param  u8CmdId         Command identifier (TELEMETRY_CMD_*)
 * @param  u8CompartmentId Compartment ID byte
 * @param  u8DockId        Dock ID byte
 * @param  pu8Payload      Pointer to serialised payload bytes
 * @param  u8PayloadLen    Number of payload bytes
 */
static void prv_SendFrame(uint8_t        u8CmdId,
                          uint8_t        u8CompartmentId,
                          uint8_t        u8DockId,
                          const uint8_t *pu8Payload,
                          uint8_t        u8PayloadLen)
{
    /* Validate inputs */
    if (pu8Payload == NULL)
    {
        SYS_CONSOLE_PRINT("[Telemetry] SendFrame: NULL payload\r\n");
        return;
    }

    /* Guard: total frame must fit in local buffer */
    uint16_t u16TotalSize = (uint16_t)TELEMETRY_FRAME_HEADER_SIZE +
                            (uint16_t)u8PayloadLen               +
                            (uint16_t)TELEMETRY_FRAME_CRC_SIZE;

    if (u16TotalSize > (uint16_t)TELEMETRY_FRAME_BUF_SIZE)
    {
        SYS_CONSOLE_PRINT("[Telemetry] SendFrame: payload too large (%u bytes)\r\n",
                          (unsigned)u8PayloadLen);
        return;
    }

    /* Local frame buffer — no shared state */
    uint8_t  au8Frame[TELEMETRY_FRAME_BUF_SIZE];
    uint16_t u16Idx = 0U;
    uint16_t u16CRC;

    /* Unique ID (4 bytes, big-endian) — wraps harmlessly at UINT32_MAX */
    u16Idx += prv_SerialiseU32BE(&au8Frame[u16Idx], s_uniqueId);

    /* Message type (2 bytes, big-endian) */
    au8Frame[u16Idx++] = (uint8_t)((TELEMETRY_MSG_TYPE_BROADCAST >> 8U) & 0xFFU);
    au8Frame[u16Idx++] = (uint8_t)( TELEMETRY_MSG_TYPE_BROADCAST        & 0xFFU);

    /* Compartment ID, Dock ID, Command ID, Payload length */
    au8Frame[u16Idx++] = u8CompartmentId;
    au8Frame[u16Idx++] = u8DockId;
    au8Frame[u16Idx++] = u8CmdId;
    au8Frame[u16Idx++] = u8PayloadLen;

    /* Payload */
    (void)memcpy(&au8Frame[u16Idx], pu8Payload, (size_t)u8PayloadLen);
    u16Idx += (uint16_t)u8PayloadLen;

    /* CRC16-CCITT over header + payload */
    u16CRC = prv_CalculateCRC16(au8Frame, u16Idx);
    au8Frame[u16Idx++] = (uint8_t)((u16CRC >> 8U) & 0xFFU);
    au8Frame[u16Idx++] = (uint8_t)( u16CRC         & 0xFFU);

    /* Transmit */
    (void)TCPIP_TCP_ArrayPut(s_telemetrySocket, au8Frame, u16Idx);
    (void)TCPIP_TCP_Flush(s_telemetrySocket);

    /* Advance sequence number (wraps at UINT32_MAX → 0, harmless) */
    s_uniqueId++;
}

/* ============================================================================
 * Per-Data-Type Send Functions
 * ========================================================================== */

/**
 * @brief  Serialise and send PM telemetry for one dock.
 *
 *         Payload is manually serialised big-endian (not memcpy of struct)
 *         to guarantee wire format regardless of host endianness.
 *
 * @param  u8CompartmentId  Compartment identifier byte
 * @param  u8DockId         Dock index (DOCK_1–DOCK_3)
 */
static void prv_SendPM(uint8_t u8CompartmentId, uint8_t u8DockId)
{
    uint8_t au8Payload[TELEMETRY_PM_PAYLOAD_SIZE];
    uint8_t u8Idx = 0U;

    const TELEMETRY_PMData_t *pPM = &telemetryData[u8DockId].pmData;

    u8Idx += prv_SerialiseU32BE(&au8Payload[u8Idx], pPM->u32PmOutputVoltage_mV);
    u8Idx += prv_SerialiseU32BE(&au8Payload[u8Idx], pPM->u32PmOutputCurrent_mA);
    u8Idx += prv_SerialiseU32BE(&au8Payload[u8Idx], pPM->u32OutputPower_W);
    u8Idx += prv_SerialiseU32BE(&au8Payload[u8Idx], pPM->u32TotalEnergy_Wh);
    u8Idx += prv_SerialiseU32BE(&au8Payload[u8Idx], pPM->u32PMFaultCode);
    au8Payload[u8Idx++] = pPM->u8PMTemperature;
    au8Payload[u8Idx++] = pPM->u8PMStatus;

    /* Compile-time guard: if struct grows, index must not exceed array */
    /* (static_assert not available in C99 — use runtime check in debug) */
    configASSERT(u8Idx == TELEMETRY_PM_PAYLOAD_SIZE);

    prv_SendFrame(TELEMETRY_CMD_PM_DATA, u8CompartmentId, u8DockId,
                  au8Payload, u8Idx);
}

/**
 * @brief  Serialise and send BMS telemetry for one dock.
 *
 *         BUG FIX: payload array was [20U] but holds 25 bytes —
 *         now uses TELEMETRY_BMS_PAYLOAD_SIZE.
 *
 *         BUG FIX: u32SystemFaultBitmap now carries the correct system-level
 *         fault bitmap (set in prv_UpdateFromSession).
 *
 * @param  u8CompartmentId  Compartment identifier byte
 * @param  u8DockId         Dock index (DOCK_1–DOCK_3)
 */
static void prv_SendBMS(uint8_t u8CompartmentId, uint8_t u8DockId)
{
    uint8_t au8Payload[TELEMETRY_BMS_PAYLOAD_SIZE];
    uint8_t u8Idx = 0U;

    const TELEMETRY_BMSData_t *pBMS = &telemetryData[u8DockId].bmsData;

    u8Idx += prv_SerialiseU32BE(&au8Payload[u8Idx], pBMS->u32BMSDemandVoltage);
    u8Idx += prv_SerialiseU32BE(&au8Payload[u8Idx], pBMS->u32BMSDemandCurrent);
    u8Idx += prv_SerialiseU32BE(&au8Payload[u8Idx], pBMS->u32EstimatedTime_s);
    u8Idx += prv_SerialiseU32BE(&au8Payload[u8Idx], pBMS->u32BMSFaultCode);
    au8Payload[u8Idx++] = pBMS->u8CurrentSoc;
    au8Payload[u8Idx++] = pBMS->u8InitialSoc;
    au8Payload[u8Idx++] = pBMS->u8BMSTemperature;
    au8Payload[u8Idx++] = pBMS->u8BMSStatus;
    au8Payload[u8Idx++] = pBMS->u8ChargingState;
    u8Idx += prv_SerialiseU32BE(&au8Payload[u8Idx], pBMS->u32SystemFaultBitmap);

    configASSERT(u8Idx == TELEMETRY_BMS_PAYLOAD_SIZE);

    prv_SendFrame(TELEMETRY_CMD_BMS_DATA, u8CompartmentId, u8DockId,
                  au8Payload, u8Idx);
}

/**
 * @brief  Serialise and send temperature telemetry (all docks in one frame).
 *
 *         BUG FIX: v1.0 accessed telemetryData[DOCK_1/2/3].tempData, which
 *         could be stale on the first pass before UpdateFromSession ran for
 *         all docks. Now reads directly from the session DB.
 *
 *         BUG FIX: payload array was sized to MAX_DOCKS (= 4 coincidentally),
 *         now explicitly TELEMETRY_TEMP_PAYLOAD_SIZE.
 *
 * @param  u8CompartmentId  Compartment identifier byte
 */
static void prv_SendTemperature(uint8_t u8CompartmentId)
{
    uint8_t au8Payload[TELEMETRY_TEMP_PAYLOAD_SIZE];
    uint8_t u8TemperatureTelemetrySize = SESSION_GetMaxDocks() + 1U; /* +1 for compartment sensor */
    /* Read directly from session DB — always fresh */
    au8Payload[0] = SESSION_GetDockTemperature((uint8_t)COMPARTMENT);
    for (uint8_t u8DockNo = DOCK_1; u8DockNo <= SESSION_GetMaxDocks(); u8DockNo++)
    {
        au8Payload[u8DockNo] = SESSION_GetDockTemperature(u8DockNo);
    }

    /* Dock ID = 0x00 indicates a broadcast (not dock-specific) */
    prv_SendFrame(TELEMETRY_CMD_TEMP_DATA, u8CompartmentId, 0x00U,
                  au8Payload, (uint8_t)u8TemperatureTelemetrySize);
}

/* ============================================================================
 * CRC16-CCITT
 * ========================================================================== */

/**
 * @brief  Compute CRC16-CCITT (polynomial 0x1021, initial value 0xFFFF).
 *
 * @param  pu8Data  Pointer to input byte array (must not be NULL)
 * @param  u16Len   Number of bytes to process
 * @return uint16_t Computed CRC
 */
static uint16_t prv_CalculateCRC16(const uint8_t *pu8Data, uint16_t u16Len)
{
    if (pu8Data == NULL)
    {
        return 0U;
    }

    uint16_t u16CRC = 0xFFFFU;

    for (uint16_t i = 0U; i < u16Len; i++)
    {
        u16CRC ^= ((uint16_t)pu8Data[i] << 8U);
        for (uint8_t j = 0U; j < 8U; j++)
        {
            if ((u16CRC & 0x8000U) != 0U)
            {
                u16CRC = (uint16_t)((u16CRC << 1U) ^ 0x1021U);
            }
            else
            {
                u16CRC <<= 1U;
            }
        }
    }
    return u16CRC;
}

/* ============================================================================
 * Public API
 * ========================================================================== */

/**
 * @brief  Open the TCP server socket and start the telemetry task.
 *
 *         BUG FIX: return type changed from void to bool so callers can detect
 *         initialisation failure.
 *
 * @return true  on success
 * @return false if socket or task creation failed
 */
bool Telemetry_Init(void)
{
    if (!prv_EnsureSocketOpen())
    {
        return false;
    }

    if (xTaskCreate(Telemetry_Task,
                    "Telemetry_Task",
                    TELEMETRY_TASK_STACK_SIZE,
                    NULL,
                    TELEMETRY_TASK_PRIORITY,
                    &s_taskHandle) != pdPASS)
    {
        SYS_CONSOLE_PRINT("[Telemetry] Task creation failed\r\n");
        return false;
    }

    SYS_CONSOLE_PRINT("[Telemetry] Initialised on port %u\r\n", TELEMETRY_PORT_HEV);
    return true;
}

/**
 * @brief  FreeRTOS task: broadcast telemetry to connected TCP client.
 *
 *         Loop behaviour:
 *           1. If socket is not open → attempt reconnect, wait, retry
 *           2. If client not connected → wait one interval
 *           3. If connected → for each dock: update + send BMS + send PM
 *              Then send one combined temperature frame
 *              Then wait TELEMETRY_TASK_DELAY_MS
 *
 * @param  pvParameters  Unused
 */
void Telemetry_Task(void *pvParameters)
{
    (void)pvParameters;

    SYS_CONSOLE_PRINT("[Telemetry] Task started\r\n");

    for (;;)
    {
        if (TCPIP_TCP_IsConnected(s_telemetrySocket))
        {
            uint8_t u8CompartmentId = SESSION_GetCompartmentId();
            /* Step 3: Broadcast per-dock PM and BMS data */
            for (uint8_t u8DockNo = (uint8_t)DOCK_1;
                 u8DockNo <= (uint8_t)SESSION_GetMaxDocks();
                 u8DockNo++)
            {
                prv_UpdateFromSession(u8DockNo);

                prv_SendBMS(u8CompartmentId, u8DockNo);
                vTaskDelay(pdMS_TO_TICKS(TELEMETRY_INTER_SEND_MS));

                prv_SendPM(u8CompartmentId, u8DockNo);
                vTaskDelay(pdMS_TO_TICKS(TELEMETRY_INTER_SEND_MS));
            }

            prv_SendTemperature(u8CompartmentId);
            /* Step 2: Wait for a client to connect */

            if ((!TCPIP_TCP_IsConnected(s_telemetrySocket)) ||
                TCPIP_TCP_WasDisconnected(s_telemetrySocket))
            {
                SYS_CONSOLE_PRINT("[Telemetry] Client disconnected\r\n");
                TCPIP_TCP_Close(s_telemetrySocket);
                s_telemetrySocket = INVALID_SOCKET;
                // vTaskDelay(pdMS_TO_TICKS(TELEMETRY_TASK_DELAY_MS));
                // continue;
            }
        }

        /* Reconnect if socket was lost */
        if (s_telemetrySocket == INVALID_SOCKET)
        {
            SYS_CONSOLE_PRINT("[Telemetry] Attempting socket reconnect...\r\n");
            s_telemetrySocket = TCPIP_TCP_ServerOpen(IP_ADDRESS_TYPE_IPV4,
                                                     TELEMETRY_PORT_HEV, 0U);
            if (s_telemetrySocket == INVALID_SOCKET)
            {
                SYS_CONSOLE_PRINT("[Telemetry] Reconnect failed, retrying in %u ms\r\n",
                                  TELEMETRY_RETRY_DELAY_MS);
            }
            else
            {
                SYS_CONSOLE_PRINT("[Telemetry] Reconnected\r\n");
            }
            /* BUG FIX: was vTaskDelay(RECONNECT_DELAY_MS) — ms not ticks */
            vTaskDelay(pdMS_TO_TICKS(TELEMETRY_RETRY_DELAY_MS));
        }

        /* Step 5: Wait until next broadcast interval */
        vTaskDelay(pdMS_TO_TICKS(TELEMETRY_TASK_DELAY_MS));
    }
}

/* ************************************************************************** */
/* End of File                                                                */
/* ************************************************************************** */