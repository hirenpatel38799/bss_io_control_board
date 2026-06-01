/*******************************************************************************
 * File Name   : AppCanHandler.c
 * Company     : Bacancy
 * Summary     : CAN communication handler — scalable implementation
 *
 * Description :
 *   Implements up to CANBUS_MAX (6) CAN channels with independent FreeRTOS
 *   SW RX/TX queues and dedicated handler tasks.
 *
 *   The number of channels initialised at runtime equals:
 *       u8ActiveChannels = min(SESSION_GetMaxDocks(), CAN_MAX_HW_CHANNELS)
 *
 *   All previously hardcoded 3-channel patterns have been replaced with:
 *     - s_canHwOps[]  : compile-time function-pointer table for all HAL ops.
 *     - s_canRxBuf[]  : statically allocated staging buffer per channel.
 *     - Runtime loops : 0 … u8ActiveChannels-1 everywhere.
 *
 *   To add a 7th CAN channel:
 *     1. Add CANBUS_6 to CANBus_e in sessionDBHandler.h.
 *     2. Add a message RAM buffer: Can6MessageRAM[].
 *     3. Add one CAN_HwOps_t entry to s_canHwOps[].
 *     4. Update CAN_MAX_HW_CHANNELS if the header macro is used instead of
 *        the enum sentinel.
 *     No logic changes needed elsewhere.
 *
 * Version     : 4.0
 *
 * Bug fixes inherited from v3.0 (all preserved):
 *   BUG-1  Server task now drains entire SW RX queue per cycle.
 *   BUG-2  Server task now drains entire SW TX queue per cycle.
 *   BUG-3  CAN_Write() auto-detects standard vs extended by ID value.
 *   BUG-4  All locals zero-initialised before the hardware switch.
 *   BUG-5  vDisplayCanRxMessage uses CAN_GetFrameId() — correct ID display.
 *   BUG-6  CAN_IsRxOK() accepts LEC==7 (no change after reset).
 *
 * New changes in v4.0:
 *   REMOVED  Three hardcoded CAN_MessageRAMConfigSet calls.
 *   REMOVED  Per-channel switch-case chains in RX drain / server / CAN_Write.
 *   REMOVED  Separate CAN0_Write / CAN1_Write / CAN2_Write functions — replaced
 *            by unified CAN_WriteStruct(u8Channel, pBuf).
 *   REMOVED  Dock→channel switch-case — replaced by arithmetic mapping.
 *   ADDED    s_canHwOps[CANBUS_MAX] — HAL function-pointer table.
 *   ADDED    s_canRxBuf[CANBUS_MAX] — per-channel staging buffer array.
 *   ADDED    s_u8ActiveChannels     — runtime active channel count.
 *   ADDED    CAN_GetActiveChannels(), CAN_GetChannelCount(), CAN_GetStats().
 *   ADDED    prv_LaunchRxTask() / prv_LaunchServerTask() — single task entry
 *            point for all channels, channel index passed as task parameter.
 *******************************************************************************/

/* ============================================================================
 * Includes
 * ========================================================================== */
#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "definitions.h"
#include "AppCanHandler.h"

/* ============================================================================
 * Debug Control
 * ========================================================================== */
#define CAN_DEBUG_ENABLE    (0)

#if CAN_DEBUG_ENABLE
    #define CAN_DEBUG(x)    do { x; } while (0)
#else
    #define CAN_DEBUG(x)    do { } while (0)
#endif

/* ============================================================================
 * Timing Configuration (unchanged from v3.0)
 * ========================================================================== */
#define CAN_RX_POLL_DELAY_MS        (2U)
#define CAN_SERVER_TASK_DELAY_MS    (10U)
#define CAN_RX_QUEUE_TIMEOUT_MS     (5U)

/* ============================================================================
 * CAN Message RAM Buffers
 *
 * One buffer per supported channel. Placed in dedicated linker sections so
 * the linker script can locate them in the MCAN peripheral's RAM region.
 *
 * Only the buffers for active channels are passed to the HAL.
 * Unused buffers waste no RAM if the linker section is in a separate region.
 * ========================================================================== */
uint8_t CACHE_ALIGN
    __attribute__((space(data), section(".can0_message_ram")))
    Can0MessageRAM[CAN0_MESSAGE_RAM_CONFIG_SIZE];

uint8_t CACHE_ALIGN
    __attribute__((space(data), section(".can1_message_ram")))
    Can1MessageRAM[CAN1_MESSAGE_RAM_CONFIG_SIZE];

uint8_t CACHE_ALIGN
    __attribute__((space(data), section(".can2_message_ram")))
    Can2MessageRAM[CAN2_MESSAGE_RAM_CONFIG_SIZE];

uint8_t CACHE_ALIGN
    __attribute__((space(data), section(".can3_message_ram")))
    Can3MessageRAM[CAN3_MESSAGE_RAM_CONFIG_SIZE];

uint8_t CACHE_ALIGN
    __attribute__((space(data), section(".can4_message_ram")))
    Can4MessageRAM[CAN4_MESSAGE_RAM_CONFIG_SIZE];

uint8_t CACHE_ALIGN
    __attribute__((space(data), section(".can5_message_ram")))
    Can5MessageRAM[CAN5_MESSAGE_RAM_CONFIG_SIZE];

/* ============================================================================
 * Public Queue Handles and Statistics Arrays
 * Sized to the hardware maximum. Only indices [0 … u8ActiveChannels-1] are
 * valid after vCanHandlerInit() completes.
 * ========================================================================== */
QueueHandle_t      xCANRXQueueHandler[CAN_MAX_HW_CHANNELS];
QueueHandle_t      xCANTXQueueHandler[CAN_MAX_HW_CHANNELS];
CAN_ChannelStats_t xCANStats[CAN_MAX_HW_CHANNELS];

/* ============================================================================
 * Private: Active Channel Count
 * Set once by vCanHandlerInit(). Read via CAN_GetActiveChannels().
 * ========================================================================== */
static uint8_t s_u8ActiveChannels = 0U;

/* ============================================================================
 * Private: Per-Channel RX Staging Buffers
 *
 * One element per supported channel. Each RX drain task owns its own buffer
 * exclusively — no mutex needed.
 * ========================================================================== */
static CAN_RX_BUFFER s_canRxBuf[CAN_MAX_HW_CHANNELS];

/* ============================================================================
 * Hardware Operations Table  (THE single place that names hardware)
 *
 * s_canHwOps[i] holds all HAL function pointers and RAM info for channel i.
 * The generic task bodies call ops->pfXxx() instead of switching on the
 * channel index, so no switch-case exists anywhere in the logic layer.
 *
 * To add a new channel:
 *   1. Add its entry here.
 *   2. Add its message RAM buffer above.
 *   Done.
 * ========================================================================== */
static const CAN_HwOps_t s_canHwOps[CAN_MAX_HW_CHANNELS] =
{
    /* ---- CAN0 ---- */
    {
        .pcName       = "CAN0",
        .pu8MessageRAM = Can0MessageRAM,
        .u32RamSize   = CAN0_MESSAGE_RAM_CONFIG_SIZE,
        .pfRamConfig  = CAN0_MessageRAMConfigSet,
        .pfIntGet     = CAN0_InterruptGet,
        .pfIntClear   = CAN0_InterruptClear,
        .pfErrorGet   = CAN0_ErrorGet,
        .pfFifoFill   = CAN0_RxFifoFillLevelGet,
        .pfRxFifo     = CAN0_MessageReceiveFifo,
        .pfTxFifo     = CAN0_MessageTransmitFifo,
    },
    /* ---- CAN1 ---- */
    {
        .pcName       = "CAN1",
        .pu8MessageRAM = Can1MessageRAM,
        .u32RamSize   = CAN1_MESSAGE_RAM_CONFIG_SIZE,
        .pfRamConfig  = CAN1_MessageRAMConfigSet,
        .pfIntGet     = CAN1_InterruptGet,
        .pfIntClear   = CAN1_InterruptClear,
        .pfErrorGet   = CAN1_ErrorGet,
        .pfFifoFill   = CAN1_RxFifoFillLevelGet,
        .pfRxFifo     = CAN1_MessageReceiveFifo,
        .pfTxFifo     = CAN1_MessageTransmitFifo,
    },
    /* ---- CAN2 ---- */
    {
        .pcName       = "CAN2",
        .pu8MessageRAM = Can2MessageRAM,
        .u32RamSize   = CAN2_MESSAGE_RAM_CONFIG_SIZE,
        .pfRamConfig  = CAN2_MessageRAMConfigSet,
        .pfIntGet     = CAN2_InterruptGet,
        .pfIntClear   = CAN2_InterruptClear,
        .pfErrorGet   = CAN2_ErrorGet,
        .pfFifoFill   = CAN2_RxFifoFillLevelGet,
        .pfRxFifo     = CAN2_MessageReceiveFifo,
        .pfTxFifo     = CAN2_MessageTransmitFifo,
    },
    /* ---- CAN3 ---- */
    {
        .pcName       = "CAN3",
        .pu8MessageRAM = Can3MessageRAM,
        .u32RamSize   = CAN3_MESSAGE_RAM_CONFIG_SIZE,
        .pfRamConfig  = CAN3_MessageRAMConfigSet,
        .pfIntGet     = CAN3_InterruptGet,
        .pfIntClear   = CAN3_InterruptClear,
        .pfErrorGet   = CAN3_ErrorGet,
        .pfFifoFill   = CAN3_RxFifoFillLevelGet,
        .pfRxFifo     = CAN3_MessageReceiveFifo,
        .pfTxFifo     = CAN3_MessageTransmitFifo,
    },
    /* ---- CAN4 ---- */
    {
        .pcName       = "CAN4",
        .pu8MessageRAM = Can4MessageRAM,
        .u32RamSize   = CAN4_MESSAGE_RAM_CONFIG_SIZE,
        .pfRamConfig  = CAN4_MessageRAMConfigSet,
        .pfIntGet     = CAN4_InterruptGet,
        .pfIntClear   = CAN4_InterruptClear,
        .pfErrorGet   = CAN4_ErrorGet,
        .pfFifoFill   = CAN4_RxFifoFillLevelGet,
        .pfRxFifo     = CAN4_MessageReceiveFifo,
        .pfTxFifo     = CAN4_MessageTransmitFifo,
    },
    /* ---- CAN5 ---- */
    {
        .pcName       = "CAN5",
        .pu8MessageRAM = Can5MessageRAM,
        .u32RamSize   = CAN5_MESSAGE_RAM_CONFIG_SIZE,
        .pfRamConfig  = CAN5_MessageRAMConfigSet,
        .pfIntGet     = CAN5_InterruptGet,
        .pfIntClear   = CAN5_InterruptClear,
        .pfErrorGet   = CAN5_ErrorGet,
        .pfFifoFill   = CAN5_RxFifoFillLevelGet,
        .pfRxFifo     = CAN5_MessageReceiveFifo,
        .pfTxFifo     = CAN5_MessageTransmitFifo,
    },
};

/* ============================================================================
 * External Protocol Dispatch Functions
 * ========================================================================== */
extern void vProcessBMSCanMessage(CAN_RX_BUFFER *pRxBuf, uint8_t u8CanBus);
extern void vProcessPMCanMessage(CAN_RX_BUFFER  *pRxBuf, uint8_t u8CanBus);

/* ============================================================================
 * Private Function Declarations
 * ========================================================================== */
static inline bool CAN_IsRxOK(uint32_t u32Status);
static void        vDisplayCanErrorStatus(uint32_t u32Status, const char *pcName);
static void        vDisplayCanRxMessage(const CAN_RX_BUFFER *pRxBuf);
static void        vProcessCanRxMessage(CAN_RX_BUFFER *pRxBuf, uint8_t u8CanBus);
static inline bool prv_ValidateTxBuffer(const CAN_TX_BUFFER *pTxBuffer,
                                        const char *pcChannel);
static void        prv_RxHandlerTask_Generic(void *pvParameters);
static void        prv_ServerTask_Generic(void *pvParameters);

/* ============================================================================
 * Utility: ID Decode
 * ========================================================================== */
/**
 * @brief  Return the decoded CAN ID from a received frame.
 *
 *         Extended (xtd=1): id field holds the 29-bit ID directly.
 *         Standard (xtd=0): id field holds the 11-bit ID left-shifted by 18;
 *                           CAN_READ_ID() corrects this.
 *
 *         This is the ONLY function that accesses pRxBuf->id. All routing and
 *         display code calls this helper to prevent the standard/extended
 *         encoding mismatch (v3.0 BUG-5).
 */
uint32_t CAN_GetFrameId(const CAN_RX_BUFFER *pRxBuf)
{
    if (pRxBuf == NULL)
    {
        return 0U;
    }
    return (pRxBuf->xtd == CAN_FRAME_EXTENDED)
           ? (pRxBuf->id & CAN_EXT_ID_MASK)
           : (CAN_READ_ID(pRxBuf->id) & CAN_STD_ID_MASK);
}

/* ============================================================================
 * Utility: 32-bit Byte-Swap
 * ========================================================================== */
uint32_t swap_endian_32(uint32_t u32Value)
{
    return ((u32Value >> 24U) & 0x000000FFUL) |
           ((u32Value >>  8U) & 0x0000FF00UL) |
           ((u32Value <<  8U) & 0x00FF0000UL) |
           ((u32Value << 24U) & 0xFF000000UL);
}

/* ============================================================================
 * Runtime Query Functions
 * ========================================================================== */

/**
 * @brief  Return the number of CAN channels actually initialised.
 *         Valid after vCanHandlerInit() has been called.
 */
uint8_t CAN_GetActiveChannels(void)
{
    return s_u8ActiveChannels;
}

/**
 * @brief  Return the channel count that vCanHandlerInit() will target.
 *         = min(SESSION_GetMaxDocks(), CAN_MAX_HW_CHANNELS)
 */
uint8_t CAN_GetChannelCount(void)
{
    uint8_t u8Docks = SESSION_GetMaxDocks();
    return (u8Docks < CAN_MAX_HW_CHANNELS) ? u8Docks : CAN_MAX_HW_CHANNELS;
}

/**
 * @brief  Safe accessor for per-channel diagnostic counters.
 */
bool CAN_GetStats(uint8_t u8Channel, CAN_ChannelStats_t *pStats)
{
    if ((pStats == NULL) || (u8Channel >= s_u8ActiveChannels))
    {
        return false;
    }
    *pStats = xCANStats[u8Channel];
    return true;
}

/* ============================================================================
 * CAN Status Helpers
 * ========================================================================== */

/**
 * @brief  Return true if CAN protocol status allows frame reception.
 *
 *         Accepted: LEC==0 (no error) or LEC==7 (no change since last read).
 *         Rejected: Bus-Off or Error-Passive state.
 *
 *         LEC==7 must be accepted to handle the case immediately after a PCAN
 *         reset, where the first valid frame arrives before the LEC has been
 *         re-written by hardware. (v3.0 BUG-6 fix, preserved in v4.0.)
 */
static inline bool CAN_IsRxOK(uint32_t u32Status)
{
    if ((u32Status & CAN_PSR_BO_Msk) != 0U) { return false; }
    if ((u32Status & CAN_PSR_EP_Msk) != 0U) { return false; }

    uint32_t u32Lec = u32Status & CAN_PSR_LEC_Msk;
    return ((u32Lec == CAN_ERROR_NONE) || (u32Lec == CAN_ERROR_LEC_NC));
}

static void vDisplayCanErrorStatus(uint32_t u32Status, const char *pcName)
{
    if (pcName == NULL) { return; }

    static const char * const pcLecTable[] =
    {
        NULL, "Stuff error", "Form error", "ACK error",
        "Bit-1 error", "Bit-0 error", "CRC error", "LEC unchanged"
    };

    uint32_t u32Lec = u32Status & CAN_PSR_LEC_Msk;
    if ((u32Lec > 0U) && (u32Lec < 7U))
    {
        SYS_CONSOLE_PRINT("[%s] LEC: %s\r\n", pcName, pcLecTable[u32Lec]);
    }
    if ((u32Status & CAN_PSR_BO_Msk) != 0U)
    {
        SYS_CONSOLE_PRINT("[%s] Bus-Off!\r\n", pcName);
    }
    if ((u32Status & CAN_PSR_EP_Msk) != 0U)
    {
        SYS_CONSOLE_PRINT("[%s] Error-Passive\r\n", pcName);
    }
    if ((u32Status & CAN_PSR_EW_Msk) != 0U)
    {
        SYS_CONSOLE_PRINT("[%s] Error Warning\r\n", pcName);
    }
}

static void vDisplayCanRxMessage(const CAN_RX_BUFFER *pRxBuf)
{
    if (pRxBuf == NULL) { return; }

    uint8_t  u8Len  = (pRxBuf->dlc <= CAN_MAX_DLC) ? pRxBuf->dlc : CAN_MAX_DLC;
    uint32_t u32Id  = CAN_GetFrameId(pRxBuf);
    char     cType  = (pRxBuf->xtd == CAN_FRAME_EXTENDED) ? 'E' : 'S';

    SYS_CONSOLE_PRINT("[RxMsg] %c ID:0x%08lX DLC:%u Data:",
                      cType, (unsigned long)u32Id, (unsigned)u8Len);
    for (uint8_t i = 0U; i < u8Len; i++)
    {
        SYS_CONSOLE_PRINT(" %02X", pRxBuf->data[i]);
    }
    SYS_CONSOLE_PRINT("\r\n");
}

/* ============================================================================
 * RX Routing
 * ========================================================================== */
/**
 * @brief  Route a received frame to the correct protocol handler.
 *
 *         Extended (29-bit) → Power Module handler.
 *         Standard (11-bit) → BMS handler.
 */
static void vProcessCanRxMessage(CAN_RX_BUFFER *pRxBuf, uint8_t u8CanBus)
{
    if (pRxBuf == NULL)
    {
        SYS_CONSOLE_PRINT("[RxDispatch] NULL buffer on CAN%u\r\n",
                          (unsigned)u8CanBus);
        return;
    }

    uint32_t u32Id = CAN_GetFrameId(pRxBuf);
    (void)u32Id;    /* used only when CAN_DEBUG_ENABLE == 1 */

    if (pRxBuf->xtd == CAN_FRAME_EXTENDED)
    {
        CAN_DEBUG(SYS_CONSOLE_PRINT("[RxDispatch] CAN%u → PM  ID=0x%08lX\r\n",
                                    (unsigned)u8CanBus, (unsigned long)u32Id));
        vProcessPMCanMessage(pRxBuf, u8CanBus);
    }
    else
    {
        CAN_DEBUG(SYS_CONSOLE_PRINT("[RxDispatch] CAN%u → BMS ID=0x%03lX\r\n",
                                    (unsigned)u8CanBus, (unsigned long)u32Id));
        vProcessBMSCanMessage(pRxBuf, u8CanBus);
    }
}

/* ============================================================================
 * TX Buffer Validation Helper
 * ========================================================================== */
static inline bool prv_ValidateTxBuffer(const CAN_TX_BUFFER *pTxBuffer,
                                        const char *pcChannel)
{
    if (pTxBuffer == NULL)
    {
        SYS_CONSOLE_PRINT("[%s Write] NULL TX buffer\r\n", pcChannel);
        return false;
    }
    if (pTxBuffer->dlc > CAN_MAX_DLC)
    {
        SYS_CONSOLE_PRINT("[%s Write] DLC out of range: %u\r\n",
                          pcChannel, (unsigned)pTxBuffer->dlc);
        return false;
    }
    return true;
}

/* ============================================================================
 * CAN Write — Raw Buffer API
 * ========================================================================== */
/**
 * @brief  Transmit a raw byte-array CAN frame on the specified channel.
 *
 *         Frame type auto-detection (v3.0 BUG-3 fix, preserved):
 *           u32RawId <= CAN_STD_ID_MASK → standard frame (xtd=0, id shifted).
 *           u32RawId >  CAN_STD_ID_MASK → extended frame (xtd=1, id as-is).
 *
 *         Uses s_canHwOps[u8CanIndex].pfTxFifo instead of a switch-case.
 */
bool CAN_Write(uint8_t u8CanIndex, const uint8_t *pu8Data, uint8_t u8Len)
{
    if (pu8Data == NULL)
    {
        SYS_CONSOLE_PRINT("[CAN_Write] NULL data pointer\r\n");
        return false;
    }
    if (u8Len < (uint8_t)CAN_RAW_FRAME_SIZE)
    {
        SYS_CONSOLE_PRINT("[CAN_Write] Buffer too short: %u < %u\r\n",
                          (unsigned)u8Len, (unsigned)CAN_RAW_FRAME_SIZE);
        return false;
    }
    if (u8CanIndex >= s_u8ActiveChannels)
    {
        SYS_CONSOLE_PRINT("[CAN_Write] Channel %u not active (active=%u)\r\n",
                          (unsigned)u8CanIndex, (unsigned)s_u8ActiveChannels);
        return false;
    }

    /* Parse big-endian ID */
    uint32_t u32RawId = ((uint32_t)pu8Data[0] << 24U) |
                        ((uint32_t)pu8Data[1] << 16U) |
                        ((uint32_t)pu8Data[2] <<  8U) |
                        ((uint32_t)pu8Data[3]);

    CAN_TX_BUFFER txBuf;
    (void)memset(&txBuf, 0, sizeof(CAN_TX_BUFFER));

    if (u32RawId <= (uint32_t)CAN_STD_ID_MASK)
    {
        txBuf.id  = CAN_WRITE_ID(u32RawId);
        txBuf.xtd = CAN_FRAME_STANDARD;
    }
    else
    {
        txBuf.id  = u32RawId & CAN_EXT_ID_MASK;
        txBuf.xtd = CAN_FRAME_EXTENDED;
    }

    txBuf.dlc = CAN_MAX_DLC;
    (void)memcpy(txBuf.data, &pu8Data[CAN_ID_BYTE_SIZE], CAN_MAX_DLC);

    CAN_DEBUG(
        SYS_CONSOLE_PRINT("[%s Write] ID:0x%08lX xtd:%u  Data:",
                          s_canHwOps[u8CanIndex].pcName,
                          (unsigned long)u32RawId, (unsigned)txBuf.xtd);
        for (uint8_t i = 0U; i < CAN_MAX_DLC; i++)
            SYS_CONSOLE_PRINT(" %02X", txBuf.data[i]);
        SYS_CONSOLE_PRINT("\r\n");
    );

    /* Route to hardware via ops table — no switch-case needed */
    return s_canHwOps[u8CanIndex].pfTxFifo(1U, &txBuf);
}

/* ============================================================================
 * CAN Write — Structured TX Buffer API (unified, replaces CAN0/1/2_Write)
 * ========================================================================== */
/**
 * @brief  Transmit a structured TX buffer on the specified channel directly.
 *         Uses s_canHwOps[u8CanIndex].pfTxFifo — no switch-case.
 */
bool CAN_WriteStruct(uint8_t u8CanIndex, const CAN_TX_BUFFER *const pTxBuffer)
{
    if (u8CanIndex >= s_u8ActiveChannels)
    {
        SYS_CONSOLE_PRINT("[CAN_WriteStruct] Channel %u not active\r\n",
                          (unsigned)u8CanIndex);
        return false;
    }

    const char *pcName = s_canHwOps[u8CanIndex].pcName;

    if (!prv_ValidateTxBuffer(pTxBuffer, pcName))
    {
        return false;
    }

    if (!s_canHwOps[u8CanIndex].pfTxFifo(1U, pTxBuffer))
    {
        SYS_CONSOLE_PRINT("[%s WriteStruct] TX FIFO failed ID=0x%08lX\r\n",
                          pcName, (unsigned long)pTxBuffer->id);
        return false;
    }
    return true;
}

/* ============================================================================
 * Backward-Compatible Per-Channel Write Wrappers
 *
 * Code that calls CAN0_Write / CAN1_Write / CAN2_Write directly still
 * compiles. These forward to CAN_WriteStruct with the fixed channel index.
 * ========================================================================== */
bool CAN0_Write(const CAN_TX_BUFFER *const pTxBuffer)
{
    return CAN_WriteStruct(0U, pTxBuffer);
}
bool CAN1_Write(const CAN_TX_BUFFER *const pTxBuffer)
{
    return CAN_WriteStruct(1U, pTxBuffer);
}
bool CAN2_Write(const CAN_TX_BUFFER *const pTxBuffer)
{
    return CAN_WriteStruct(2U, pTxBuffer);
}
bool CAN3_Write(const CAN_TX_BUFFER *const pTxBuffer)
{
    return CAN_WriteStruct(3U, pTxBuffer);
}
bool CAN4_Write(const CAN_TX_BUFFER *const pTxBuffer)
{
    return CAN_WriteStruct(4U, pTxBuffer);
}
bool CAN5_Write(const CAN_TX_BUFFER *const pTxBuffer)
{
    return CAN_WriteStruct(5U, pTxBuffer);
}

/* ============================================================================
 * TX Queue Enqueue API
 * ========================================================================== */
/**
 * @brief  Enqueue a TX frame for the dock's CAN channel.
 *
 *         Dock → channel mapping:
 *           channel = (u8DockNo - DOCK_1)
 *
 *         This arithmetic replaces the switch-case from v3.0.
 *         DOCK_1 = 1, so DOCK_1 → channel 0, DOCK_6 → channel 5.
 *
 *         Validation:
 *           - u8DockNo must be >= DOCK_1 and < (DOCK_1 + s_u8ActiveChannels)
 *           - Derived channel must be < s_u8ActiveChannels
 *
 *         Non-blocking: drops frame and increments counter if queue is full.
 */
void vSendCanTxMsgToQueue(const CAN_TX_BUFFER *const pCanTxBuffer,
                          uint8_t u8DockNo)
{
    if (pCanTxBuffer == NULL)
    {
        SYS_CONSOLE_PRINT("[TxQueue] NULL buffer for dock %u\r\n",
                          (unsigned)u8DockNo);
        return;
    }

    /* Validate dock range */
    if ((u8DockNo < (uint8_t)DOCK_1) ||
        (u8DockNo >= (uint8_t)(DOCK_1 + s_u8ActiveChannels)))
    {
        SYS_CONSOLE_PRINT("[TxQueue] Dock %u out of range "
                          "(active channels: %u, valid: DOCK_%u … DOCK_%u)\r\n",
                          (unsigned)u8DockNo,
                          (unsigned)s_u8ActiveChannels,
                          (unsigned)DOCK_1,
                          (unsigned)(DOCK_1 + s_u8ActiveChannels - 1U));
        return;
    }
    /* Arithmetic dock → channel mapping */
    const uint8_t u8Channel = (uint8_t)(u8DockNo - (uint8_t)DOCK_1);

    QueueHandle_t xQueue = xCANTXQueueHandler[u8Channel];
    if (xQueue == NULL)
    {
        SYS_CONSOLE_PRINT("[TxQueue] Queue NULL for %s\r\n",
                          s_canHwOps[u8Channel].pcName);
        return;
    }

    if (xQueueSend(xQueue, pCanTxBuffer, 0U) != pdPASS)
    {
        xCANStats[u8Channel].u32TxDrops++;
        SYS_CONSOLE_PRINT("[TxQueue] %s TX queue full — frame dropped "
                          "(total: %lu)\r\n",
                          s_canHwOps[u8Channel].pcName,
                          (unsigned long)xCANStats[u8Channel].u32TxDrops);
    }
}

/* ============================================================================
 * Generic RX Drain Task
 *
 * Single entry point for all channels. Channel index is passed as pvParameters
 * (cast from uint32_t to avoid pointer-size issues on 32-bit MCU).
 *
 * Replaces vCan0RxHandlerTask / vCan1RxHandlerTask / vCan2RxHandlerTask.
 * All hardware accesses go through s_canHwOps[u8Channel] — no switch-case.
 * ========================================================================== */
/**
 * @brief  Generic RX drain task. Polls hardware FIFO and pushes to SW queue.
 *
 *         pvParameters: (void*)(uintptr_t)channel_index
 *
 *         Per cycle:
 *           1. Read interrupt flag via ops->pfIntGet().
 *           2. If set: clear, get error status, check CAN_IsRxOK().
 *           3. Get fill level via ops->pfFifoFill().
 *           4. Read frame via ops->pfRxFifo().
 *           5. Push to SW RX queue.
 *           6. Sleep CAN_RX_POLL_DELAY_MS.
 *
 *         All locals initialised before use (v3.0 BUG-4 fix, preserved).
 */
static void prv_RxHandlerTask_Generic(void *pvParameters)
{
    uint8_t u8Channel = (uint8_t)(uintptr_t)pvParameters;

    /* Defensive guard — should never fail if launched by vCanHandlerInit() */
    if (u8Channel >= s_u8ActiveChannels)
    {
        SYS_CONSOLE_PRINT("[RxTask] Channel %u >= active %u — exiting\r\n",
                          (unsigned)u8Channel, (unsigned)s_u8ActiveChannels);
        vTaskDelete(NULL);
        return;
    }

    const CAN_HwOps_t *pOps     = &s_canHwOps[u8Channel];
    CAN_RX_BUFFER     *pRxBuf   = &s_canRxBuf[u8Channel];
    QueueHandle_t      xRxQueue = xCANRXQueueHandler[u8Channel];

    if ((pOps == NULL) || (pRxBuf == NULL) || (xRxQueue == NULL))
    {
        SYS_CONSOLE_PRINT("[%s RX] NULL resource — exiting\r\n",
                          pOps ? pOps->pcName : "???");
        vTaskDelete(NULL);
        return;
    }

    SYS_CONSOLE_PRINT("[%s RX] Task started (poll %u ms)\r\n",
                      pOps->pcName, (unsigned)CAN_RX_POLL_DELAY_MS);

    while (true)
    {
        /* BUG-4 FIX: zero-initialise all locals */
        bool     bIntFlag    = false;
        uint32_t u32ErrStat  = 0U;
        uint8_t  u8FillLevel = 0U;
        bool     bRxOk       = false;

        /* ---- Poll hardware via ops table ---- */
        bIntFlag = pOps->pfIntGet(CAN_INTERRUPT_RF0N_MASK);

        if (bIntFlag)
        {
            pOps->pfIntClear(CAN_INTERRUPT_RF0N_MASK);
            u32ErrStat  = pOps->pfErrorGet();
            CAN_DEBUG(vDisplayCanErrorStatus(u32ErrStat, pOps->pcName));
            bRxOk       = CAN_IsRxOK(u32ErrStat);

            if (bRxOk)
            {
                u8FillLevel = pOps->pfFifoFill(CAN_RX_FIFO_0);
                if (u8FillLevel > 0U)
                {
                    bRxOk = pOps->pfRxFifo(CAN_RX_FIFO_0, 1U, pRxBuf);
                }
                else
                {
                    bRxOk = false;  /* nothing to read */
                }
            }
        }

        /* ---- Push to SW queue if we got a frame ---- */
        if (bIntFlag && bRxOk)
        {
            CAN_DEBUG(vDisplayCanRxMessage(pRxBuf));

            if (xQueueSend(xRxQueue, pRxBuf,
                           pdMS_TO_TICKS(CAN_RX_QUEUE_TIMEOUT_MS)) != pdPASS)
            {
                xCANStats[u8Channel].u32RxDrops++;
                SYS_CONSOLE_PRINT("[%s RX] SW queue full — dropped "
                                  "ID=0x%08lX (total: %lu)\r\n",
                                  pOps->pcName,
                                  (unsigned long)CAN_GetFrameId(pRxBuf),
                                  (unsigned long)xCANStats[u8Channel].u32RxDrops);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(CAN_RX_POLL_DELAY_MS));
    }
}

/* ============================================================================
 * Generic Server Task
 *
 * Single entry point for all channels. Channel index passed as pvParameters.
 *
 * Replaces vCan0HandlerServerTask / vCan1HandlerServerTask / vCan2HandlerServerTask.
 * TX write uses CAN_WriteStruct() which routes via s_canHwOps — no switch-case.
 * ========================================================================== */
/**
 * @brief  Generic server task. Drains entire SW TX and RX queues each cycle.
 *
 *         pvParameters: (void*)(uintptr_t)channel_index
 *
 *         TX phase: loop xQueueReceive(xTxQueue, 0) until empty → CAN_WriteStruct().
 *         RX phase: loop xQueueReceive(xRxQueue, 0) until empty → vProcessCanRxMessage().
 *
 *         Full-drain per cycle (v3.0 BUG-1 + BUG-2 fix, preserved):
 *         All frames that arrived simultaneously are processed before sleeping.
 */
static void prv_ServerTask_Generic(void *pvParameters)
{
    const uint8_t u8Channel = (uint8_t)(uintptr_t)pvParameters;

    if (u8Channel >= s_u8ActiveChannels)
    {
        SYS_CONSOLE_PRINT("[SrvTask] Channel %u >= active %u — exiting\r\n",
                          (unsigned)u8Channel, (unsigned)s_u8ActiveChannels);
        vTaskDelete(NULL);
        return;
    }

    QueueHandle_t xTxQueue = xCANTXQueueHandler[u8Channel];
    QueueHandle_t xRxQueue = xCANRXQueueHandler[u8Channel];

    if ((xTxQueue == NULL) || (xRxQueue == NULL))
    {
        SYS_CONSOLE_PRINT("[%s SRV] NULL queue — exiting\r\n",
                          s_canHwOps[u8Channel].pcName);
        vTaskDelete(NULL);
        return;
    }

    SYS_CONSOLE_PRINT("[%s SRV] Task started (cycle %u ms)\r\n",
                      s_canHwOps[u8Channel].pcName,
                      (unsigned)CAN_SERVER_TASK_DELAY_MS);

    CAN_TX_BUFFER tx;
    CAN_RX_BUFFER rx;

    while (true)
    {
        /* ---- TX phase: drain entire SW TX queue ---- */
        while (xQueueReceive(xTxQueue, &tx, 10U) == pdPASS)
        {
            // if (u8Channel == (uint8_t)CANBUS_0)
            // {
            //     SYS_CONSOLE_PRINT("[TxQueue] Channel %u -> pCanTxBuffer ID=0x%08lX\r\n",
            //                       (unsigned)u8Channel, (unsigned long)tx.id);
            // }
            if (!CAN_WriteStruct(u8Channel, &tx))
            {
                SYS_CONSOLE_PRINT("[%s SRV] TX failed ID=0x%08lX\r\n",
                                  s_canHwOps[u8Channel].pcName,
                                  (unsigned long)tx.id);
            }
            vTaskDelay(pdMS_TO_TICKS(1U));
        }

        /* ---- RX phase: drain entire SW RX queue ---- */
        while (xQueueReceive(xRxQueue, &rx, 0U) == pdPASS)
        {
            vProcessCanRxMessage(&rx, u8Channel);
        }

        vTaskDelay(pdMS_TO_TICKS(CAN_SERVER_TASK_DELAY_MS));
    }
}

/* ============================================================================
 * Backward-Compatible Per-Channel Task Wrappers
 *
 * These exist so any existing xTaskCreate(vCan0RxHandlerTask, ...) calls
 * continue to compile. They forward to the generic launcher with the
 * appropriate channel index hardcoded.
 *
 * New code should call vCanHandlerInit() instead of creating tasks manually.
 * ========================================================================== */
void vCan0RxHandlerTask(void *pvParameters) { (void)pvParameters; prv_RxHandlerTask_Generic((void *)(uintptr_t)0U); }
void vCan1RxHandlerTask(void *pvParameters) { (void)pvParameters; prv_RxHandlerTask_Generic((void *)(uintptr_t)1U); }
void vCan2RxHandlerTask(void *pvParameters) { (void)pvParameters; prv_RxHandlerTask_Generic((void *)(uintptr_t)2U); }
void vCan3RxHandlerTask(void *pvParameters) { (void)pvParameters; prv_RxHandlerTask_Generic((void *)(uintptr_t)3U); }
void vCan4RxHandlerTask(void *pvParameters) { (void)pvParameters; prv_RxHandlerTask_Generic((void *)(uintptr_t)4U); }
void vCan5RxHandlerTask(void *pvParameters) { (void)pvParameters; prv_RxHandlerTask_Generic((void *)(uintptr_t)5U); }

void vCan0HandlerServerTask(void *pvParameters) { (void)pvParameters; prv_ServerTask_Generic((void *)(uintptr_t)0U); }
void vCan1HandlerServerTask(void *pvParameters) { (void)pvParameters; prv_ServerTask_Generic((void *)(uintptr_t)1U); }
void vCan2HandlerServerTask(void *pvParameters) { (void)pvParameters; prv_ServerTask_Generic((void *)(uintptr_t)2U); }
void vCan3HandlerServerTask(void *pvParameters) { (void)pvParameters; prv_ServerTask_Generic((void *)(uintptr_t)3U); }
void vCan4HandlerServerTask(void *pvParameters) { (void)pvParameters; prv_ServerTask_Generic((void *)(uintptr_t)4U); }
void vCan5HandlerServerTask(void *pvParameters) { (void)pvParameters; prv_ServerTask_Generic((void *)(uintptr_t)5U); }

/* ============================================================================
 * Initialisation
 * ========================================================================== */
/**
 * @brief  Dynamically initialise CAN channels based on SESSION_GetMaxDocks().
 *
 *         Step 1 — Determine active channel count.
 *           u8N = min(SESSION_GetMaxDocks(), CAN_MAX_HW_CHANNELS).
 *           Validates SESSION_GetMaxDocks() > 0.
 *
 *         Step 2 — Configure message RAM for channels 0 … u8N-1.
 *           Calls s_canHwOps[i].pfRamConfig(s_canHwOps[i].pu8MessageRAM).
 *           No hardcoded peripheral names.
 *
 *         Step 3 — Create SW RX + TX queues for channels 0 … u8N-1.
 *           Aborts (returns false, no tasks started) if any queue fails.
 *
 *         Step 4 — Create RX drain task and server task for each channel.
 *           Channel index is passed as (void*)(uintptr_t)i to the generic
 *           launcher. No per-channel task functions needed.
 *
 *         Step 5 — Store active count in s_u8ActiveChannels.
 *           Subsequent calls to CAN_GetActiveChannels(), CAN_Write(),
 *           CAN_WriteStruct(), and vSendCanTxMsgToQueue() all use this.
 */
bool vCanHandlerInit(void)
{
    bool bSuccess = true;

    /* ── Step 1: Determine active channel count ──────────────────────────── */
    uint8_t u8MaxDocks = SESSION_GetMaxDocks();

    if (u8MaxDocks == 0U)
    {
        SYS_CONSOLE_PRINT("[CanInit] SESSION_GetMaxDocks() returned 0 "
                          "— at least 1 dock required\r\n");
        return false;
    }

    uint8_t u8N = (u8MaxDocks < (uint8_t)CANBUS_MAX)
                  ? u8MaxDocks
                  : (uint8_t)CANBUS_MAX;

    SYS_CONSOLE_PRINT("[CanInit] Initialising %u CAN channel(s) "
                      "(MaxDocks=%u, HW_MAX=%u)\r\n",
                      (unsigned)u8N,
                      (unsigned)u8MaxDocks,
                      (unsigned)CANBUS_MAX);

    /* ── Step 2: Hardware message RAM ───────────────────────────────────── */
    for (uint8_t i = 0U; i < u8N; i++)
    {
        const CAN_HwOps_t *pOps = &s_canHwOps[i];

        if ((pOps->pfRamConfig == NULL) || (pOps->pu8MessageRAM == NULL))
        {
            SYS_CONSOLE_PRINT("[CanInit] %s: NULL RAM config pointer\r\n",
                              pOps->pcName);
            bSuccess = false;
            continue;
        }

        if (!pOps->pfRamConfig(pOps->pu8MessageRAM))
        {
            SYS_CONSOLE_PRINT("[CanInit] %s: MessageRAMConfigSet failed\r\n",
                              pOps->pcName);
            bSuccess = false;
        }
        else
        {
            SYS_CONSOLE_PRINT("[CanInit] %s: RAM configured (%lu bytes)\r\n",
                              pOps->pcName, (unsigned long)pOps->u32RamSize);
        }
    }

    if (!bSuccess)
    {
        SYS_CONSOLE_PRINT("[CanInit] RAM configuration error(s) — aborting\r\n");
        return false;
    }

    /* ── Step 3: SW queues ───────────────────────────────────────────────── */
    (void)memset(xCANStats, 0, sizeof(xCANStats));

    for (uint8_t i = 0U; i < u8N; i++)
    {
        xCANRXQueueHandler[i] = xQueueCreate(CAN_QUEUE_SIZE,
                                             sizeof(CAN_RX_BUFFER));
        xCANTXQueueHandler[i] = xQueueCreate(CAN_QUEUE_SIZE,
                                             sizeof(CAN_TX_BUFFER));

        if (xCANRXQueueHandler[i] == NULL)
        {
            SYS_CONSOLE_PRINT("[CanInit] %s: RX queue creation failed\r\n",
                              s_canHwOps[i].pcName);
            bSuccess = false;
        }
        if (xCANTXQueueHandler[i] == NULL)
        {
            SYS_CONSOLE_PRINT("[CanInit] %s: TX queue creation failed\r\n",
                              s_canHwOps[i].pcName);
            bSuccess = false;
        }
    }

    if (!bSuccess)
    {
        SYS_CONSOLE_PRINT("[CanInit] Queue creation failed — tasks NOT started\r\n");
        return false;
    }

    SYS_CONSOLE_PRINT("[CanInit] All %u queue pair(s) created\r\n",
                      (unsigned)u8N);

    /* ── Step 4: FreeRTOS tasks ──────────────────────────────────────────── */
    for (uint8_t i = 0U; i < u8N; i++)
    {
        char acTaskName[16];

        /* RX drain task — channel index passed as pvParameters */
        (void)snprintf(acTaskName, sizeof(acTaskName), "%s_RX",
                       s_canHwOps[i].pcName);

        if (xTaskCreate(prv_RxHandlerTask_Generic,
                        acTaskName,
                        CAN_RX_HANDLER_STACK_DEPTH,
                        (void *)(uintptr_t)i,
                        CAN_RX_HANDLER_TASK_PRIORITY,
                        NULL) != pdPASS)
        {
            SYS_CONSOLE_PRINT("[CanInit] %s: RX task creation failed\r\n",
                              acTaskName);
            bSuccess = false;
        }
        else
        {
            SYS_CONSOLE_PRINT("[CanInit] %s: RX task started\r\n", acTaskName);
        }

        /* Server task */
        (void)snprintf(acTaskName, sizeof(acTaskName), "%s_SRV",
                       s_canHwOps[i].pcName);

        if (xTaskCreate(prv_ServerTask_Generic,
                        acTaskName,
                        CAN_SERVER_HANDLER_STACK_DEPTH,
                        (void *)(uintptr_t)i,
                        CAN_SERVER_HANDLER_TASK_PRIORITY,
                        NULL) != pdPASS)
        {
            SYS_CONSOLE_PRINT("[CanInit] %s: server task creation failed\r\n",
                              acTaskName);
            bSuccess = false;
        }
        else
        {
            SYS_CONSOLE_PRINT("[CanInit] %s: server task started\r\n",
                              acTaskName);
        }
    }

    /* ── Step 5: Commit active channel count ─────────────────────────────── */
    if (bSuccess)
    {
        s_u8ActiveChannels = u8N;
        SYS_CONSOLE_PRINT("[CanInit] Initialisation complete. "
                          "Active channels: %u\r\n", (unsigned)s_u8ActiveChannels);
    }
    else
    {
        SYS_CONSOLE_PRINT("[CanInit] WARNING: one or more resources failed. "
                          "Active channels: 0\r\n");
    }

    return bSuccess;
}

/*******************************************************************************
 * End of File
 *******************************************************************************/