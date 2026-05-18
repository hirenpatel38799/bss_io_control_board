/* ************************************************************************** */
/** @file    IOHandler.c
 *  @brief   IO status management and flash config storage — implementation
 *
 *  @company BACANCY SYSTEMS PVT. LTD.
 *
 *  @details
 *    Implements digital/analog pin management, flash I/O state storage,
 *    TCP command parsing, and port configuration for the IO Aggregator board.
 *
 *  Version : 2.2
 *
 *  Changes from v2.1:
 *    - Fixed: Task loop called prv_HandleCommissioningCommand(buf, len) with
 *      arguments — function takes no parameters.  Fixed to call
 *      prv_ParseAndProcessInbPacket() identically to the normal path, so the
 *      same frame parser populates s_reqFrame before dispatching.
 *    - Fixed: prv_ProcessCommissioningWrite() expected a 38-byte payload with
 *      an extra sub-cmd prefix byte.  Corrected to match the actual 37-byte
 *      wire format where payload[0] IS msgType (0x00=READ / 0x01=WRITE).
 *    - Fixed: prv_HandleCommissioningCommand() now checks payloadLen == 37 in
 *      commissioning mode (not 38) before routing to Read/Write.
 *    - Fixed: flash save in prv_ProcessCommissioningWrite() called
 *      prv_SaveOutputsToFlash_Raw() which only updates digitalOutputs /
 *      relayOutputs / serialNumber — iocCfg changes were lost on power cycle.
 *      Replaced with new prv_SaveIocConfigToFlash() which writes the full
 *      flash_data_t page so all fields (including iocCfg) are persisted.
 *    - Added: prv_SaveIocConfigToFlash() — atomic erase+write of the entire
 *      flash_data_t page with timeout guards, preserving all existing fields.
 *    - Improved: All commissioning log messages now include field values for
 *      easier on-device debugging.
 *
 *  Changes from v2.0:
 *    - Added: CMD_COMMISSIONING_COMMAND (0x07) to command switch.
 *    - Added: system_mode_t enum (MODE_NORMAL / MODE_COMMISSIONING) and
 *      static volatile g_systemMode variable.
 *    - Added: s_commSocket — dedicated TCP socket for the commissioning server.
 *    - Added: prv_HandleCommissioningCommand() — dispatches Enter / Read / Write
 *      sub-commands; only Enter is accepted while in MODE_NORMAL.
 *    - Added: prv_StopCurrentServer() — closes s_ioSocket gracefully before
 *      switching to commissioning mode.
 *    - Added: prv_StartCommissioningServer() — sets IP to COMM_SERVER_IP
 *      (192.168.1.22) and opens a TCP server on COMM_SERVER_PORT (6235).
 *    - Added: prv_ProcessCommissioningRead() — serialises writeData.iocCfg
 *      into a 37-byte IOC_Commisioning_t frame and sends it as a response.
 *    - Added: prv_ProcessCommissioningWrite() — validates payload length, CRC,
 *      MaxDock range and IP/subnet strings; on success updates RAM config,
 *      persists to flash, ACKs the client, and triggers SYS_RESET_SoftwareReset.
 *    - Added: commissioning server poll branch in prv_IOHandlerServerTask —
 *      when g_systemMode == MODE_COMMISSIONING the task services s_commSocket
 *      instead of s_ioSocket.
 *    - Added: macros for all commissioning configuration (server IP/port,
 *      payload offsets, struct size, response codes, sub-command bytes).
 *
 *    - Fixed: `bGPIO_Operation` had no `return` statement at the end of the
 *      function — undefined behaviour; all paths now return explicitly.
 *    - Fixed: `u8DockNo` used as direct array index into Gpio_conf arrays
 *      without bounds check — index could be >= MAX_DOCKS causing buffer
 *      overflow. All per-dock GPIO operations now guard the index.
 *    - Fixed: `uint16_t relayStatus` and `uint32_t doStatus` re-declared as
 *      local variables inside `readOutputsFromFlash`, shadowing the globals
 *      — the save path used the wrong (local) values. Removed local shadows.
 *    - Fixed: `uint16_t serialnum` declared twice (global + local in header).
 *      Removed duplicate; now a single definition here.
 *    - Fixed: `StoreRelay_DigitalOutputsFrame` used |= on persistent globals
 *      (`doStatus`, `relayStatus`) without clearing first — bits that were
 *      once set could never be cleared. Now assigns = not |=.
 *    - Fixed: `Analog_Input_Get(channel)` uses `channel - 1` without guarding
 *      against channel == 0 → underflow to 0xFF → OOB read. Added guard.
 *    - Fixed: `MovingAverage_Update(channel, ...)` uses `channel` as index
 *      into `ma[]` (size 20) without bounds check — OOB if channel >= 20.
 *    - Fixed: `ADC busy-wait` in `ReadAllAnalogInputPins` has no timeout —
 *      could block forever. Added timeout counter.
 *    - Fixed: `SendResponsePayload` had no bounds check on `len` vs
 *      `outbPacketBuf` size (256 bytes) — potential buffer overflow.
 *    - Fixed: `PrepareDispatchOutbPacket` directly indexes
 *      `Reqframe_St.payload[0/1/2]` without checking payloadLen >= 3 — OOB
 *      read if payload is shorter.
 *    - Fixed: `ParseAndProcessInbPacket` declares `uint8_t u8DockNo` inside
 *      a `case` statement without a compound statement — C99 violation.
 *    - Fixed: `vTaskDelay(100)` in solenoid path — raw number, should be
 *      `pdMS_TO_TICKS(100U)`.
 *    - Fixed: `vTaskDelay(RECONNECT_DELAY_MS)` — macro is milliseconds but
 *      vTaskDelay takes ticks. Wrapped in pdMS_TO_TICKS.
 *    - Fixed: `vTaskDelay(10)` in main task loop — raw number, should be
 *      `pdMS_TO_TICKS(10U)`.
 *    - Fixed: `inbPortNo`, `inbPortVal`, `inbCrc` declared as `short`/`int`
 *      — signed types for port numbers invite negative-index bugs.
 *      Changed to appropriate unsigned types.
 *    - Fixed: `CalculateCRC` comment claims polynomial 0x8005 but uses 0x1021
 *      — corrected comment to CRC-16/CCITT.
 *    - Fixed: `GetPortFromSerialNumber` checks 4 non-overlapping ranges that
 *      together cover 1–110 and always return `serialNumber` unmodified —
 *      function is pure dead code / identity function. Replaced with inline
 *      range check.
 *    - Fixed: `ResetInbOutbData` uses two separate for-loops of 256 to zero
 *      packet buffers — replaced with memset.
 *    - Fixed: `HandleGPIOCommand` validates portVal != 0 && != 1 but only
 *      for write operations — read operations still set errorCode on portVal
 *      that defaults to 0 (valid). Moved portVal check inside write-only path.
 *    - Fixed: `saveOutputsToFlash` blocks on `while(FCW_IsBusy())` without
 *      timeout — could hang forever if flash controller locks up.
 *    - Fixed: NTC sensor `TempCalcNTC` has division by (VREF - in_volt) with
 *      no guard against in_volt >= VREF — divide-by-zero (or negative
 *      resistance). Added guard.
 *    - Improved: `TCA9539_CONFIG_PORT0/1` macro values corrected in .c
 *      (were 0x06/0x07 — those are the configuration registers, consistent
 *      with the TCA9539 datasheet).
 *    - Improved: `Board_gpio_st Gpio_conf` array indices now start at
 *      DOCK_1 (1) not 0 — index 0 is COMPARTMENT which has no dock GPIO.
 *      Added comment warning.
 *    - Improved: Magic numbers in `SaveResponsePayload` / `PrepareDispatch`
 *      for response type (0x00, 0x02) replaced with MSG_TYPE_RESPONSE.
 */
/* ************************************************************************** */

/* ============================================================================
 * Includes
 * ========================================================================== */
#include "IOHandler.h"
#include "definitions.h"
#include "configuration.h"
#include "device.h"
#include "tcpip_manager_control.h"
#include "library/tcpip/tcpip_helpers.h"
#include <math.h>
#include <string.h>
#include "sessionDBHandler.h"
#include "led_manager.h"
/* ============================================================================
 * Internal Configuration
 * ========================================================================== */
#define NUM_DI_PINS             (60U)   /**< Total digital inputs                */
#define NUM_DO_PINS             (24U)   /**< Total digital outputs               */
#define NUM_RELAY_PINS          (6U)    /**< Total relay / eFuse outputs         */

#define TCA9539_REG_OUT_PORT0   (0x02U) /**< TCA9539 output register port 0     */
#define TCA9539_REG_OUT_PORT1   (0x03U) /**< TCA9539 output register port 1     */

#define ADC_TIMEOUT_CYCLES      (100000U) /**< Max spin cycles waiting for ADC  */

/** TCP frame layout constants */
#define FRAME_HEADER_SIZE       (10U)   /**< UniqueID(4)+MsgType(2)+Cmp(1)+Dock(1)+Cmd(1)+PayLen(1) */
#define FRAME_CRC_SIZE          (2U)
#define RESPONSE_MSG_TYPE_HI    (0x00U)
#define RESPONSE_MSG_TYPE_LO    (0x02U)

/** Packet buffer sizes */
#define PKT_BUF_SIZE            (256U)

/** Solenoid actuation pulse width */
#define SOLENOID_PULSE_MS       (100U)

/** Task loop delay */
#define IO_TASK_LOOP_DELAY_MS   (10U)

/** Moving average sample count and channel count */
#define MA_SAMPLE_LEN           (50U)
#define MA_CHANNEL_COUNT        (20U)   /**< Must match ALL_ANALOG_PINS        */

#define MSG_TYPE_REQUEST        (0x01)
#define MSG_TYPE_RESPONSE       (0x02)

/** MAC address base bytes */
#define APP_MAC_BASE_BYTE0    (0x00U)
#define APP_MAC_BASE_BYTE1    (0x04U)
#define APP_MAC_BASE_BYTE2    (0x25U)
#define APP_MAC_BASE_BYTE3    (0x1CU)
#define APP_MAC_BASE_BYTE4    (0xA0U)
/* ============================================================================
 * Commissioning Mode Configuration
 * ========================================================================== */
/** TCP command ID that triggers / handles commissioning operations */
#define CMD_COMMISSIONING_COMMAND       (0x07U)

/** Sub-command byte within a CMD_COMMISSIONING_COMMAND frame */
#define COMM_SUBCMD_ENTER               (0x01U)  /**< Enter commissioning mode     */
#define COMM_SUBCMD_READ                (0x00U)  /**< Read current configuration   */
#define COMM_SUBCMD_WRITE               (0x01U)  /**< Write new configuration      */

/** Commissioning server network parameters */
#define COMM_SERVER_IP                  "192.168.1.22"
#define COMM_SERVER_PORT                (6235U)

/** IOC_Commisioning_t payload field offsets inside the TCP frame payload */
#define COMM_PAYLOAD_MSGTYPE_OFF        (0U)
#define COMM_PAYLOAD_COMP_ID_OFF        (1U)
#define COMM_PAYLOAD_MAX_DOCK_OFF       (2U)
#define COMM_PAYLOAD_IP_OFF             (3U)
#define COMM_PAYLOAD_SUBNET_OFF         (19U)
#define COMM_PAYLOAD_CRC_OFF            (35U)

/** Total size of a serialised IOC_Commisioning_t structure (bytes) */
#define IOC_COMMISSIONING_STRUCT_SIZE   (37U)

/** Maximum valid value for u8MaxDocks */
#define COMM_MAX_DOCK_LIMIT             (6U)

/** ACK / NACK response codes */
#define COMM_RESP_ACK                   (0x00U)
#define COMM_RESP_NACK_PAYLOAD_LEN      (0x01U)
#define COMM_RESP_NACK_CRC              (0x02U)
#define COMM_RESP_NACK_INVALID_FIELD    (0x03U)
#define COMM_RESP_NACK_FLASH            (0x04U)
/* ============================================================================
 * Sensor Calibration Constants (file-scope only — not exported)
 * ========================================================================== */
#if PT100_Sensor
static const float k_VrefL          = 4.9500F;
static const float k_AdcMaxVal      = 4095.0F;
static const float k_Vref           = 3.290F;
static const float k_Gain           = 10.9F;
static const float k_Rref           = 3300.0F;
static const float k_TempCoeffPT100 = 0.00385F;
#endif

#if NTC_Sensor
static const float    k_CoeffA      = 0.0011279F;
static const float    k_CoeffB      = 0.00023429F;
static const float    k_CoeffC      = 0.000000087298F;
static const uint32_t k_RefResistor = 4700U;
static const float    k_VrefNTC     = 3.3F;
#endif
/* =========================
 * GLOBAL MAC STORAGE
 * ========================= */

// static mac_flash_data_t gMacData;
/* =========================
 * INTERNAL STORAGE
 * ========================= */
uint8_t gDeviceMac[6];

/* ============================================================================
 * Global Variables
 * ========================================================================== */

/** Digital / relay output state bitmaps */
uint32_t doStatus    = 0U;
uint16_t relayStatus = 0U;
uint16_t serialnum   = 0U;

/** ADC / temperature buffers */
volatile uint16_t currentTempAIQueueBuffer[NUM_TEMPERATURE_ANALOG_PINS];
volatile uint16_t currentAIQueueBuffer[NUM_ANALOG_PINS];
volatile uint8_t  currentDIQueueBuffer[NUM_DI_PINS];

/** Analog message buffer */
char     messageAnalog[MESSAGE_BUFFER_SIZE];
uint32_t total_length = 0U;

/** I2C shared buffers */
uint8_t i2cTxBuf[2]      = {0U};
uint8_t i2cRxBuf[2]      = {0U};
bool    i2cTransferDone   = false;

/** Bootloader trigger RAM pointer */
uint32_t *ramStart = (uint32_t *)BTL_TRIGGER_RAM_START;

/** ADC result array (used by some external callers) */
volatile uint32_t adc_data[NUM_ANALOG_PINS];

/* ============================================================================
 * Private Variables
 * ========================================================================== */

/** TCP server socket */
static TCP_SOCKET s_ioSocket = INVALID_SOCKET;

/** Parsed request frame (valid between parse and reset) */
static TCP_RequestFrame_t s_reqFrame;

/** Response packet buffer */
static uint8_t s_outbuf[PKT_BUF_SIZE];

/** Legacy inbound state (kept for PrepareDispatchOutbPacket compatibility) */
static char     s_inbMsgSubType  = 0x00;
static int32_t  s_inbPortVal     = -1;
static char     s_outbErrorCode;
static int32_t  s_outbPortVal    = -1;

/** Board selection flag (set once after first DI read) */
static volatile bool s_boardSelectionDone = false;

/** IO operation busy flag */
volatile bool bIOOperation = false;

/* ── Commissioning state ──────────────────────────────────────────────────── */

/**
 * @brief  System operation mode.
 */
typedef enum
{
    MODE_NORMAL = 0,    /**< Normal TCP server operation */
    MODE_COMMISSIONING  /**< Commissioning server active  */
} system_mode_t;

/** Current system operation mode */
static volatile system_mode_t g_systemMode = MODE_NORMAL;

/** Dedicated commissioning TCP socket */
static TCP_SOCKET s_commSocket = INVALID_SOCKET;

/* ============================================================================
 * GPIO Pin Configuration Table
 *
 * BUG FIX: Gpio_conf arrays are indexed with u8DockNo directly.
 * u8DockNo comes from sessionDB Dock_e where DOCK_1=1, DOCK_2=2, DOCK_3=3.
 * Index 0 (COMPARTMENT) is unused for per-dock GPIO — arrays sized to
 * MAX_DOCKS with index 0 left as 0.
 * ========================================================================== */
static const Board_gpio_st k_GpioConf = {
    .AC_Relay_Pin       = {0,  61, 62, 63, 73, 74, 75},  /* [0]=unused, [1]=DOCK_1, ... */
    .DC_Relay_Pin       = {0,  64, 65, 66, 76, 77, 78},
    .Solenoid_PinHi     = {0,  67, 68, 69, 79, 80, 81},
    .Solenoid_PinLo     = {0,  70, 71, 72, 82, 83, 84},
    .R_LED_Pin          = {0,  91, 92, 93, 100, 101, 102},
    .G_LED_Pin          = {0,  94, 95, 96, 103, 104, 105},
    .B_LED_Pin          = {0,  97, 98, 99, 106, 107, 108},
    .Dock_Fan_Pin       = {0,  85, 86, 87, 88, 89, 90},
    .Compartment_Fan_Pin = 0, /* No compartment fan GPIO */
    .DoorLock_Pin       = {0,  1,  2,  3, 17, 18, 19},
    .SolenoidLock_Pin   = {0,  4,  5,  6, 20, 21, 22},
    .IgnitionSence_Pin  = {0,  8,  9,  10, 23, 24, 25},
    .EStop_Pin          = 7
};

/* ============================================================================
 * Error Code Constants
 * ========================================================================== */
static const uint8_t k_ErrNone          = 0x00U;
static const uint8_t k_ErrInvalidMsg    = 0x01U;
static const uint8_t k_ErrInvalidCrc    = 0x02U;
static const uint8_t k_ErrInvalidPort   = 0x08U;
static const uint8_t k_ErrInvalidValue  = 0x10U;

/* ============================================================================
 * Message Sub-type Constants
 * ========================================================================== */
static const uint8_t k_MsgDigitalRead   = 0x00U;
static const uint8_t k_MsgDigitalWrite  = 0x01U;

/* ============================================================================
 * Private Function Prototypes
 * ========================================================================== */
static void     prv_SaveOutputsToFlash_Raw(uint32_t u32Digital,
                                           uint16_t u16Relay,
                                           uint16_t u16Serial);
static void     prv_ReadOutputsFromFlash(void);
static void     prv_StoreRelay_DigitalOutputsFrame(void);
static void     prv_ReadDigitalInputs(void);
static void     prv_SetStaticIPAddress(const char *pcIpStr, const char *pcSubnetStr);
static uint16_t prv_CalculateCRC16(const uint8_t *pu8Data, uint16_t u16Len);
static void     prv_ParseAndProcessInbPacket(uint8_t *pu8Buf, uint16_t u16Len);
static void     prv_HandleGPIOCommand(void);
static void     prv_HandleAnalogRead(void);
static void     prv_HandleChargingCommand(uint8_t u8DockNo);
static void     prv_HandleBootloaderCommand(void);
static void     prv_SendResponsePayload(const uint8_t *pu8Payload, uint8_t u8Len);
static void     prv_PrepareDispatchOutbPacket(void);
static void     prv_ResetFrameState(void);
static uint8_t  prv_GPIO_Read(uint16_t u16PortNo, uint8_t *pu8ErrorCode);
static void     prv_GPIO_Write(uint16_t u16PortNo, uint8_t u8Val, uint8_t *pu8ErrorCode);
static uint8_t  prv_ChargingControl(uint8_t u8DockNo, uint8_t u8Action);
static uint32_t prv_AnalogInputGet(uint8_t u8Channel);
static void     prv_vConfigureIOexpanders(void);
static void     prv_SetStoredStaticIP(void);

/* Commissioning mode */
static void     prv_HandleCommissioningCommand(void);
static void     prv_ParseCommissioningPacket(uint8_t *pu8Buf, uint16_t u16Len);
static void     prv_StopCurrentServer(void);
static void     prv_StartCommissioningServer(void);
static void     prv_ProcessCommissioningRead(void);
static void     prv_ProcessCommissioningWrite(const uint8_t *pu8Struct);
static void     prv_SaveIocConfigToFlash(void);

#if PT100_Sensor
static inline float prv_TempCalcPT100(float fVoltage);
#endif
#if NTC_Sensor
static inline float prv_TempCalcNTC(float fVoltage);
#endif

/* ============================================================================
 * Moving Average Filter
 * ========================================================================== */

typedef struct
{
    uint16_t au16Buffer[MA_SAMPLE_LEN];
    uint32_t u32Sum;
    uint8_t  u8Index;
} MovingAverage_t;

static MovingAverage_t s_ma[MA_CHANNEL_COUNT];

/**
 * @brief  Initialise all moving average filter instances to zero.
 */
static void prv_MovingAverage_Init_All(void)
{
    (void)memset(s_ma, 0, sizeof(s_ma));
}

/**
 * @brief  Update moving average for one channel and return the new average.
 *
 *         BUG FIX: original function had no bounds check on channel — if
 *         channel >= MA_CHANNEL_COUNT (20), it wrote past the end of s_ma[].
 *
 * @param  u8Channel  ADC channel index (0 to MA_CHANNEL_COUNT-1)
 * @param  u16New     New raw ADC sample
 * @return uint16_t   Running average
 */
uint16_t MovingAverage_Update(uint8_t u8Channel, uint16_t u16New)
{
    if (u8Channel >= (uint8_t)MA_CHANNEL_COUNT)
    {
        SYS_CONSOLE_PRINT("[MA] Invalid channel %u\r\n", (unsigned)u8Channel);
        return 0U;
    }

    MovingAverage_t *pMA = &s_ma[u8Channel];
    pMA->u32Sum -= (uint32_t)pMA->au16Buffer[pMA->u8Index];
    pMA->au16Buffer[pMA->u8Index] = u16New;
    pMA->u32Sum += (uint32_t)u16New;
    pMA->u8Index = (uint8_t)((pMA->u8Index + 1U) % MA_SAMPLE_LEN);
    return (uint16_t)(pMA->u32Sum / MA_SAMPLE_LEN);
}

/* ============================================================================
 * Temperature Calculation Helpers
 * ========================================================================== */

#if PT100_Sensor
/**
 * @brief  Convert ADC voltage to PT100 temperature (°C).
 * @param  fVoltage  Voltage at ADC input (V)
 * @return float     Temperature in °C
 */
static inline float prv_TempCalcPT100(float fVoltage)
{
    float fDenom = (k_Gain * k_VrefL) - fVoltage;
    if (fDenom <= 0.0F)
    {
        return -273.15F; /* Return absolute-zero sentinel on invalid input */
    }
    float fResistance   = (k_Rref * fVoltage) / fDenom;
    float fTemperature  = ((fResistance / 100.0F) - 1.0F) / k_TempCoeffPT100;
    return fTemperature;
}
#endif

#if NTC_Sensor
/**
 * @brief  Convert ADC voltage to NTC thermistor temperature (°C).
 *
 *         BUG FIX: original had no guard on (VREF - in_volt) <= 0, which
 *         causes divide-by-zero or negative resistance when in_volt >= VREF.
 *
 * @param  fVoltage  Voltage at ADC input (V)
 * @return float     Temperature in °C (returns -273.15 on invalid input)
 */
static inline float prv_TempCalcNTC(float fVoltage)
{
    float fDenom = k_VrefNTC - fVoltage;
    if (fDenom <= 0.0F)
    {
        return -273.15F; /* Guard against divide-by-zero */
    }
    float fResistance = (float)k_RefResistor * (fVoltage / fDenom);
    if (fResistance <= 0.0F)
    {
        return -273.15F;
    }
    float fLnRes  = logf(fResistance);
    float fInvT   = k_CoeffA + (k_CoeffB * fLnRes) +
                    (k_CoeffC * (fLnRes * fLnRes * fLnRes));
    if (fInvT <= 0.0F)
    {
        return -273.15F;
    }
    return (1.0F / fInvT) - 273.15F;
}
#endif

/* ============================================================================
 * Flash Operations
 * ========================================================================== */

/**
 * @brief  Print the current flash configuration to console.
 */
static void prv_PrintSavedFlashData(void)
{
    flash_data_t savedData;
    FCW_Read((uint32_t *)&savedData, sizeof(savedData), FLASH_ADDRESS);

    if (savedData.magic != FLASH_MAGIC)
    {
        SYS_CONSOLE_PRINT("[Flash] Invalid or empty (magic=0x%08lX)\r\n",
                          (unsigned long)savedData.magic);
        return;
    }

    SYS_CONSOLE_PRINT("---- SAVED FLASH DATA ----\r\n");

    SYS_CONSOLE_PRINT("Digital: 0x%08lX  Relay: 0x%04X\r\n",
                      (unsigned long)savedData.digitalOutputs,
                      (unsigned)savedData.relayOutputs);

    /* --- IOC CONFIG --- */
    SYS_CONSOLE_PRINT("---- IOC CONFIG ----\r\n");
    SYS_CONSOLE_PRINT("Compartment ID: %u\r\n",
                      (unsigned)savedData.iocCfg.u8CompartmentId);

    SYS_CONSOLE_PRINT("Max Dock: %u\r\n",
                      (unsigned)savedData.iocCfg.u8MaxDocks);

    SYS_CONSOLE_PRINT("IP Address: %s\r\n",
                      savedData.iocCfg.cIpAddress);

    SYS_CONSOLE_PRINT("Subnet Mask: %s\r\n",
                      savedData.iocCfg.cSubnetMask);

    for (uint8_t i = 0U; i < 6U; i++)
    {
        SYS_CONSOLE_PRINT("CAN%u: Port=%u Baud=%lu\r\n",
                          (unsigned)(i + 1U),
                          (unsigned)savedData.canPorts[i],
                          (unsigned long)savedData.canBaudRates[i]);
    }

    for (uint8_t i = 0U; i < 2U; i++)
    {
        SYS_CONSOLE_PRINT("RS485%u: Port=%u Baud=%lu Data=%u Parity=%c Stop=%u\r\n",
                          (unsigned)(i + 1U),
                          (unsigned)savedData.rs485Ports[i],
                          (unsigned long)savedData.rs485Config[i].baudRate,
                          (unsigned)savedData.rs485Config[i].dataBits,
                          savedData.rs485Config[i].parity,
                          (unsigned)savedData.rs485Config[i].stopBits);
    }

    SYS_CONSOLE_PRINT("--------------------------\r\n");
}

/**
 * @brief  Write flash with busy-wait and timeout.
 *
 *         BUG FIX: original `while (FCW_IsBusy())` had no timeout and could
 *         block indefinitely. Added a maximum iteration count.
 *
 * @param  u32Digital   Digital output bitmap to save
 * @param  u16Relay     Relay output bitmap to save
 * @param  u16Serial    Serial number to save
 */
static void prv_SaveOutputsToFlash_Raw(uint32_t u32Digital,
                                       uint16_t u16Relay,
                                       uint16_t u16Serial)
{
    uint32_t u32Timeout;
    uint8_t  au8WriteBuffer[sizeof(flash_data_t)];

    writeData.magic          = FLASH_MAGIC;
    writeData.digitalOutputs = u32Digital;
    writeData.relayOutputs   = u16Relay;
    writeData.serialNumber   = u16Serial;

    (void)memcpy(au8WriteBuffer, &writeData, sizeof(flash_data_t));
    DCACHE_CLEAN_BY_ADDR((uint32_t *)au8WriteBuffer, sizeof(au8WriteBuffer));

    FCW_PageErase(FLASH_ADDRESS);
    u32Timeout = ADC_TIMEOUT_CYCLES;
    while (FCW_IsBusy() && (u32Timeout > 0U)) { u32Timeout--; }
    if (u32Timeout == 0U)
    {
        SYS_CONSOLE_PRINT("[Flash] Erase timeout!\r\n");
        return;
    }

    FCW_RowWrite((uint32_t *)au8WriteBuffer, FLASH_ADDRESS);
    u32Timeout = ADC_TIMEOUT_CYCLES;
    while (FCW_IsBusy() && (u32Timeout > 0U)) { u32Timeout--; }
    if (u32Timeout == 0U)
    {
        SYS_CONSOLE_PRINT("[Flash] Write timeout!\r\n");
        return;
    }

    SYS_CONSOLE_PRINT("[Flash] Configuration saved.\r\n");
}

/**
 * @brief  Public wrapper — save output states to flash.
 */
void saveOutputsToFlash(uint32_t u32Digital, uint16_t u16Relay, uint16_t u16SerialNum)
{
    prv_SaveOutputsToFlash_Raw(u32Digital, u16Relay, u16SerialNum);
}

/**
 * @brief  Restore output states from flash, or write defaults if flash is invalid.
 *
 *         BUG FIX: local `uint16_t relayStatus` and `uint32_t doStatus` inside
 *         this function shadowed the globals. The Save path at the end would
 *         save the local (potentially wrong) values. Removed local shadows.
 */
static void prv_ReadOutputsFromFlash(void)
{
    uint8_t     au8ReadBuffer[sizeof(flash_data_t)];
    uint32_t    u32Timeout;
    bool        bFlashInvalid = false;

    FCW_Read((uint32_t *)au8ReadBuffer, sizeof(flash_data_t), FLASH_ADDRESS);
    flash_data_t *pData = (flash_data_t *)au8ReadBuffer;

    /* Check for blank flash (all 0x00 or 0xFF) */
    bool bBlank = true;
    for (size_t i = 0U; i < sizeof(flash_data_t); i++)
    {
        if ((au8ReadBuffer[i] != 0x00U) && (au8ReadBuffer[i] != 0xFFU))
        {
            bBlank = false;
            break;
        }
    }

    if ((pData->magic != FLASH_MAGIC) || bBlank)
    {
        bFlashInvalid = true;
    }
    else
    {
        for (uint8_t i = 0U; i < 6U; i++)
        {
            if ((pData->canPorts[i] == 0U) || (pData->canBaudRates[i] == 0UL))
            {
                bFlashInvalid = true;
                break;
            }
        }
        for (uint8_t i = 0U; (i < 2U) && (!bFlashInvalid); i++)
        {
            if ((pData->rs485Ports[i] == 0U)           ||
                (pData->rs485Config[i].baudRate == 0UL) ||
                (pData->rs485Config[i].dataBits == 0U)  ||
                (pData->rs485Config[i].stopBits == 0U))
            {
                bFlashInvalid = true;
            }
        }
    }

    if (!bFlashInvalid)
    {
        (void)memcpy(&writeData, pData, sizeof(flash_data_t));
        serialnum = writeData.serialNumber;

        SYS_CONSOLE_PRINT("[Flash] Restored Digital=0x%08lX Relay=0x%04X\r\n",
                          (unsigned long)writeData.digitalOutputs,
                          (unsigned)writeData.relayOutputs);
        /* Restore RS-485 UART configuration */
        for (uint8_t u8UartIdx = 0U; u8UartIdx < 2U; u8UartIdx++)
        {
            uint32_t u32Baud      = writeData.rs485Config[u8UartIdx].baudRate;
            uint8_t  u8DataBits   = writeData.rs485Config[u8UartIdx].dataBits;
            uint8_t  u8StopBits   = writeData.rs485Config[u8UartIdx].stopBits;
            int32_t  i32Parity    = (int32_t)writeData.rs485Config[u8UartIdx].parity;

            USART_DATA eDataWidth;
            switch (u8DataBits)
            {
                case 5U: eDataWidth = USART_DATA_5_BIT; break;
                case 6U: eDataWidth = USART_DATA_6_BIT; break;
                case 7U: eDataWidth = USART_DATA_7_BIT; break;
                case 8U: eDataWidth = USART_DATA_8_BIT; break;
                default:
                    SYS_CONSOLE_PRINT("[RS485%u] Invalid data bits (%u)\r\n",
                                      (unsigned)(u8UartIdx + 1U), (unsigned)u8DataBits);
                    continue;
            }

            USART_PARITY eParity;
            switch (i32Parity)
            {
                case 78: eParity = USART_PARITY_NONE; break;
                case 79: eParity = USART_PARITY_ODD;  break;
                case 69: eParity = USART_PARITY_EVEN; break;
                default:
                    SYS_CONSOLE_PRINT("[RS485%u] Invalid parity (%ld)\r\n",
                                      (unsigned)(u8UartIdx + 1U), (long)i32Parity);
                    continue;
            }

            USART_STOP eStop = (u8StopBits == 1U) ? USART_STOP_0_BIT : USART_STOP_1_BIT;

            USART_SERIAL_SETUP sSetup = {
                .baudRate  = u32Baud,
                .dataWidth = eDataWidth,
                .parity    = eParity,
                .stopBits  = eStop
            };

            if (u8UartIdx == 0U)
            {
                SERCOM8_USART_ReadAbort();
                if (!SERCOM8_USART_SerialSetup(&sSetup, 0U))
                {
                    SYS_CONSOLE_PRINT("[RS485_1] Setup failed\r\n");
                }
                else
                {
                    uint8_t u8Dummy;
                    (void)SERCOM8_USART_Read(&u8Dummy, 1U);
                    SYS_CONSOLE_PRINT("[RS485_1] Setup OK Baud=%lu\r\n", (unsigned long)u32Baud);
                }
            }
            else
            {
                SERCOM9_USART_ReadAbort();
                if (!SERCOM9_USART_SerialSetup(&sSetup, 0U))
                {
                    SYS_CONSOLE_PRINT("[RS485_2] Setup failed\r\n");
                }
                else
                {
                    uint8_t u8Dummy;
                    (void)SERCOM9_USART_Read(&u8Dummy, 1U);
                    SYS_CONSOLE_PRINT("[RS485_2] Setup OK Baud=%lu\r\n", (unsigned long)u32Baud);
                }
            }
        }
        SYS_CONSOLE_PRINT("[Flash] Outputs restored.\r\n");
    }
    else
    {
        SYS_CONSOLE_PRINT("[Flash] Invalid/empty loading defaults.\r\n");
        (void)memset(&writeData, 0, sizeof(flash_data_t));
        writeData.magic = FLASH_MAGIC;
        
        /* --- DEFAULT IOC CONFIG --- */
        writeData.iocCfg.u8CompartmentId = 1U;
        writeData.iocCfg.u8MaxDocks       = 3U;

        strncpy(writeData.iocCfg.cIpAddress, "192.168.1.231", 16);
        writeData.iocCfg.cIpAddress[15] = '\0';

        strncpy(writeData.iocCfg.cSubnetMask, "255.255.255.0", 16);
        writeData.iocCfg.cSubnetMask[15] = '\0';

        static const uint16_t k_DefCanPorts[6]   = {8120U, 8121U, 8122U, 8123U, 8124U, 8125U};
        static const uint16_t k_DefRs485Ports[2] = {9114U, 9115U};
        static const uint32_t k_DefCanBaud[6]    = {500000UL, 500000UL, 500000UL,
                                                     500000UL, 500000UL, 500000UL};

        (void)memcpy(writeData.canPorts,    k_DefCanPorts,   sizeof(k_DefCanPorts));
        (void)memcpy(writeData.rs485Ports,  k_DefRs485Ports, sizeof(k_DefRs485Ports));
        (void)memcpy(writeData.canBaudRates, k_DefCanBaud,   sizeof(k_DefCanBaud));

        for (uint8_t i = 0U; i < 2U; i++)
        {
            writeData.rs485Config[i].baudRate = 9600UL;
            writeData.rs485Config[i].dataBits = 8U;
            writeData.rs485Config[i].parity   = 'N';
            writeData.rs485Config[i].stopBits = 1U;
        }
        uint8_t au8WriteBuf[sizeof(flash_data_t)];
        (void)memcpy(au8WriteBuf, &writeData, sizeof(flash_data_t));
        DCACHE_CLEAN_BY_ADDR((uint32_t *)au8WriteBuf, sizeof(au8WriteBuf));

        FCW_PageErase(FLASH_ADDRESS);
        u32Timeout = ADC_TIMEOUT_CYCLES;
        while (FCW_IsBusy() && (u32Timeout > 0U)) { u32Timeout--; }

        FCW_RowWrite((uint32_t *)au8WriteBuf, FLASH_ADDRESS);
        u32Timeout = ADC_TIMEOUT_CYCLES;
        while (FCW_IsBusy() && (u32Timeout > 0U)) { u32Timeout--; }
        SYS_CONSOLE_PRINT("[Flash] Defaults saved.\r\n");
    }
    SESSION_SetCompartmentId(writeData.iocCfg.u8CompartmentId);
    SESSION_SetMaxDocks(writeData.iocCfg.u8MaxDocks);
    SESSION_SetIpAddress(writeData.iocCfg.cIpAddress);
    SESSION_SetSubnetMask(writeData.iocCfg.cSubnetMask);
    SYS_CONSOLE_PRINT("[Flash] CompartmentId=%u MaxDocks=%u IP=%s Subnet=%s\r\n",
                      (unsigned)writeData.iocCfg.u8CompartmentId,
                      (unsigned)writeData.iocCfg.u8MaxDocks,
                      writeData.iocCfg.cIpAddress,
                      writeData.iocCfg.cSubnetMask);
}

/**
 * @brief  Snapshot current hardware output states into global bitmaps and save to flash.
 *
 *         BUG FIX: original used |= which meant once-set bits could never be
 *         cleared. Now assigns fresh bitmaps from hardware reads.
 */
static void prv_StoreRelay_DigitalOutputsFrame(void)
{
    /* Build fresh relay bitmap */
    relayStatus = 0U;
    relayStatus |= (uint16_t)((efuse1_in_Get()      != 0U) ? (1U << 0U) : 0U);
    relayStatus |= (uint16_t)((efuse2_in_Get()      != 0U) ? (1U << 1U) : 0U);
    relayStatus |= (uint16_t)((efuse3_in_Get()      != 0U) ? (1U << 2U) : 0U);
    relayStatus |= (uint16_t)((efuse4_in_Get()      != 0U) ? (1U << 3U) : 0U);
    relayStatus |= (uint16_t)((Relay_Output_1_Get() != 0U) ? (1U << 4U) : 0U);
    relayStatus |= (uint16_t)((Relay_Output_2_Get() != 0U) ? (1U << 5U) : 0U);

    /* Build fresh digital output bitmap */
    doStatus = 0U;
    doStatus |= ((Digital_Output_1_Get()  != 0U) ? (1UL <<  0U) : 0UL);
    doStatus |= ((Digital_Output_2_Get()  != 0U) ? (1UL <<  1U) : 0UL);
    doStatus |= ((Digital_Output_3_Get()  != 0U) ? (1UL <<  2U) : 0UL);
    doStatus |= ((Digital_Output_4_Get()  != 0U) ? (1UL <<  3U) : 0UL);
    doStatus |= ((Digital_Output_5_Get()  != 0U) ? (1UL <<  4U) : 0UL);
    doStatus |= ((Digital_Output_6_Get()  != 0U) ? (1UL <<  5U) : 0UL);
    doStatus |= ((Digital_Output_7_Get()  != 0U) ? (1UL <<  6U) : 0UL);
    doStatus |= ((Digital_Output_8_Get()  != 0U) ? (1UL <<  7U) : 0UL);
    doStatus |= ((Digital_Output_9_Get()  != 0U) ? (1UL <<  8U) : 0UL);
    doStatus |= ((Digital_Output_10_Get() != 0U) ? (1UL <<  9U) : 0UL);
    doStatus |= ((Digital_Output_11_Get() != 0U) ? (1UL << 10U) : 0UL);
    doStatus |= ((Digital_Output_12_Get() != 0U) ? (1UL << 11U) : 0UL);
    doStatus |= ((Digital_Output_13_Get() != 0U) ? (1UL << 12U) : 0UL);
    doStatus |= ((Digital_Output_14_Get() != 0U) ? (1UL << 13U) : 0UL);
    doStatus |= ((Digital_Output_15_Get() != 0U) ? (1UL << 14U) : 0UL);
    doStatus |= ((Digital_Output_16_Get() != 0U) ? (1UL << 15U) : 0UL);
    doStatus |= ((Digital_Output_17_Get() != 0U) ? (1UL << 16U) : 0UL);
    doStatus |= ((Digital_Output_18_Get() != 0U) ? (1UL << 17U) : 0UL);
    doStatus |= ((Digital_Output_19_Get() != 0U) ? (1UL << 18U) : 0UL);
    doStatus |= ((Digital_Output_20_Get() != 0U) ? (1UL << 19U) : 0UL);
    doStatus |= ((Digital_Output_21_Get() != 0U) ? (1UL << 20U) : 0UL);
    doStatus |= ((Digital_Output_22_Get() != 0U) ? (1UL << 21U) : 0UL);
    doStatus |= ((Digital_Output_23_Get() != 0U) ? (1UL << 22U) : 0UL);
    doStatus |= ((Digital_Output_24_Get() != 0U) ? (1UL << 23U) : 0UL);

    prv_SaveOutputsToFlash_Raw(doStatus, relayStatus, serialnum);
    SYS_CONSOLE_PRINT("[Flash] Outputs saved DO=0x%08lX RELAY=0x%04X\r\n",
                      (unsigned long)doStatus, (unsigned)relayStatus);
}

/* ============================================================================
 * ADC / Analog Input Reading
 * ========================================================================== */

/**
 * @brief  Read all ADC channels, apply moving average, convert to temperature
 *         or voltage, store results, and update session DB temperatures.
 *
 *         BUG FIX: ADC busy-wait now has a timeout (ADC_TIMEOUT_CYCLES).
 *         BUG FIX: temperature fixed-point value clamp checks were unnecessary
 *         because uint16_t already can't exceed 65535 — simplified.
 */
void ReadAllAnalogInputPins(void)
{
    /* Logging every 100 cycles */
    static uint8_t logCounter = 0;
    logCounter++;

    if (logCounter >= 100)
    {
        static const ADC_CORE_NUM k_AdcCores[ALL_ANALOG_PINS] = {
            ADC_CORE_NUM2, ADC_CORE_NUM2, ADC_CORE_NUM0, ADC_CORE_NUM0,
            ADC_CORE_NUM0, ADC_CORE_NUM1, ADC_CORE_NUM0, ADC_CORE_NUM1,
            ADC_CORE_NUM1, ADC_CORE_NUM0, ADC_CORE_NUM1, ADC_CORE_NUM0,
            ADC_CORE_NUM3, ADC_CORE_NUM3, ADC_CORE_NUM2, ADC_CORE_NUM2,
            ADC_CORE_NUM3, ADC_CORE_NUM3, ADC_CORE_NUM3, ADC_CORE_NUM3};
        static const ADC_CHANNEL_NUM k_AdcChannels[ALL_ANALOG_PINS] = {
            ADC_CH5, ADC_CH4, ADC_CH5, ADC_CH2,
            ADC_CH0, ADC_CH4, ADC_CH7, ADC_CH0,
            ADC_CH5, ADC_CH4, ADC_CH2, ADC_CH1,
            ADC_CH0, ADC_CH1, ADC_CH3, ADC_CH2,
            ADC_CH4, ADC_CH5, ADC_CH2, ADC_CH3};

        volatile uint32_t au32AdcData[ALL_ANALOG_PINS];
        const float k_InvAdcMax = 1.0F / 4095.0F;

        RTC_Timer32Start();
        ADC_GlobalEdgeConversionStart();

        /* BUG FIX: timeout guard on ADC busy-wait */
        uint32_t u32Timeout = ADC_TIMEOUT_CYCLES;
        while ((!ADC_CORE_INT_EOSRDY) && (u32Timeout > 0U))
        {
            u32Timeout--;
        }
        if (u32Timeout == 0U)
        {
            SYS_CONSOLE_PRINT("[ADC] Conversion timeout\r\n");
            return;
        }

        for (uint8_t i = 0U; i < (uint8_t)ALL_ANALOG_PINS; i++)
        {
            au32AdcData[i] = ADC_ResultGet(k_AdcCores[i], k_AdcChannels[i]);
            uint16_t u16Avg = MovingAverage_Update(i, (uint16_t)au32AdcData[i]);

#if PT100_Sensor
            float fVoltage = (float)u16Avg * k_Vref * k_InvAdcMax;
#else
            float fVoltage = (float)u16Avg * ADC_VREF * k_InvAdcMax;
#endif

            if (i < (uint8_t)NUM_TEMPERATURE_ANALOG_PINS)
            {
                float fTemp;
#if defined(PT100_Sensor) && (PT100_Sensor == 1)
                fTemp = prv_TempCalcPT100(fVoltage);
#elif defined(NTC_Sensor) && (NTC_Sensor == 1)
                fTemp = prv_TempCalcNTC(fVoltage);
#else
                fTemp = (float)u16Avg;
#endif
                /* Store as fixed-point (0.01 °C units) */
                float fFixed = fTemp * 100.0F;
                if (fFixed < 0.0F)
                {
                    fFixed = 0.0F;
                }
                currentTempAIQueueBuffer[i] = (uint16_t)fFixed;
            }
            else
            {
                float fScaled = fVoltage * 100.0F;
                if (fScaled < 0.0F)
                {
                    fScaled = 0.0F;
                }
                currentAIQueueBuffer[i - (uint8_t)NUM_TEMPERATURE_ANALOG_PINS] = (uint16_t)fScaled;
            }
        }

        /* Update session DB temperatures — convert back to °C */
        uint8_t u8Temp = (uint8_t)(currentTempAIQueueBuffer[6U] / 100U); /* Convert back to °C */
        SESSION_SetDockTemperature((uint8_t)COMPARTMENT, u8Temp);
        SYS_CONSOLE_PRINT("Temps -> Comp:%d", u8Temp);
        for (uint8_t u8DockNo = DOCK_1; u8DockNo <= SESSION_GetMaxDocks(); u8DockNo++)
        {
            uint8_t u8DockId = (uint8_t)(u8DockNo);
            uint8_t u8Temp = (uint8_t)(currentTempAIQueueBuffer[6U + u8DockNo] / 100U); /* Convert back to °C */
            SESSION_SetDockTemperature(u8DockId, u8Temp);
            SYS_CONSOLE_PRINT(" D%d:%d", u8DockId, u8Temp);
        }
        SYS_CONSOLE_PRINT("\r\n");
        logCounter = 0;
    }
}

/* ============================================================================
 * Digital Input Reading
 * ========================================================================== */

/**
 * @brief  Read all 49 direct digital inputs and 11 IO-expander inputs.
 *         Sets static IP address once on first call.
 */
static void prv_ReadDigitalInputs(void)
{
    uint8_t u8Exp0 = TCA9539_ReadRegister(INPUT_PORT_REG);
    uint8_t u8Exp1 = TCA9539_ReadRegister(INPUT_PORT_REG_2);

    /* Direct digital inputs 1–49 → buffer indices 0–48 */
    currentDIQueueBuffer[0]  = (Digital_Input_1_Get()  != 0U) ? 1U : 0U;
    currentDIQueueBuffer[1]  = (Digital_Input_2_Get()  != 0U) ? 1U : 0U;
    currentDIQueueBuffer[2]  = (Digital_Input_3_Get()  != 0U) ? 1U : 0U;
    currentDIQueueBuffer[3]  = (Digital_Input_4_Get()  != 0U) ? 1U : 0U;
    currentDIQueueBuffer[4]  = (Digital_Input_5_Get()  != 0U) ? 1U : 0U;
    currentDIQueueBuffer[5]  = (Digital_Input_6_Get()  != 0U) ? 1U : 0U;
    currentDIQueueBuffer[6]  = (Digital_Input_7_Get()  != 0U) ? 1U : 0U;
    currentDIQueueBuffer[7]  = (Digital_Input_8_Get()  != 0U) ? 1U : 0U;
    currentDIQueueBuffer[8]  = (Digital_Input_9_Get()  != 0U) ? 1U : 0U;
    currentDIQueueBuffer[9]  = (Digital_Input_10_Get() != 0U) ? 1U : 0U;
    currentDIQueueBuffer[10] = (Digital_Input_11_Get() != 0U) ? 1U : 0U;
    currentDIQueueBuffer[11] = (Digital_Input_12_Get() != 0U) ? 1U : 0U;
    currentDIQueueBuffer[12] = (Digital_Input_13_Get() != 0U) ? 1U : 0U;
    currentDIQueueBuffer[13] = (Digital_Input_14_Get() != 0U) ? 1U : 0U;
    currentDIQueueBuffer[14] = (Digital_Input_15_Get() != 0U) ? 1U : 0U;
    currentDIQueueBuffer[15] = (Digital_Input_16_Get() != 0U) ? 1U : 0U;
    currentDIQueueBuffer[16] = (Digital_Input_17_Get() != 0U) ? 1U : 0U;
    currentDIQueueBuffer[17] = (Digital_Input_18_Get() != 0U) ? 1U : 0U;
    currentDIQueueBuffer[18] = (Digital_Input_19_Get() != 0U) ? 1U : 0U;
    currentDIQueueBuffer[19] = (Digital_Input_20_Get() != 0U) ? 1U : 0U;
    currentDIQueueBuffer[20] = (Digital_Input_21_Get() != 0U) ? 1U : 0U;
    currentDIQueueBuffer[21] = (Digital_Input_22_Get() != 0U) ? 1U : 0U;
    currentDIQueueBuffer[22] = (Digital_Input_23_Get() != 0U) ? 1U : 0U;
    currentDIQueueBuffer[23] = (Digital_Input_24_Get() != 0U) ? 1U : 0U;
    currentDIQueueBuffer[24] = (Digital_Input_25_Get() != 0U) ? 1U : 0U;
    currentDIQueueBuffer[25] = (Digital_Input_26_Get() != 0U) ? 1U : 0U;
    currentDIQueueBuffer[26] = (Digital_Input_27_Get() != 0U) ? 1U : 0U;
    currentDIQueueBuffer[27] = (Digital_Input_28_Get() != 0U) ? 1U : 0U;
    currentDIQueueBuffer[28] = (Digital_Input_29_Get() != 0U) ? 1U : 0U;
    currentDIQueueBuffer[29] = (Digital_Input_30_Get() != 0U) ? 1U : 0U;
    currentDIQueueBuffer[30] = (Digital_Input_31_Get() != 0U) ? 1U : 0U;
    currentDIQueueBuffer[31] = (Digital_Input_32_Get() != 0U) ? 1U : 0U;
    currentDIQueueBuffer[32] = (Digital_Input_33_Get() != 0U) ? 1U : 0U;
    currentDIQueueBuffer[33] = (Digital_Input_34_Get() != 0U) ? 1U : 0U;
    currentDIQueueBuffer[34] = (Digital_Input_35_Get() != 0U) ? 1U : 0U;
    currentDIQueueBuffer[35] = (Digital_Input_36_Get() != 0U) ? 1U : 0U;
    currentDIQueueBuffer[36] = (Digital_Input_37_Get() != 0U) ? 1U : 0U;
    currentDIQueueBuffer[37] = (Digital_Input_38_Get() != 0U) ? 1U : 0U;
    currentDIQueueBuffer[38] = (Digital_Input_39_Get() != 0U) ? 1U : 0U;
    currentDIQueueBuffer[39] = (Digital_Input_40_Get() != 0U) ? 1U : 0U;
    currentDIQueueBuffer[40] = (Digital_Input_41_Get() != 0U) ? 1U : 0U;
    currentDIQueueBuffer[41] = (Digital_Input_42_Get() != 0U) ? 1U : 0U;
    currentDIQueueBuffer[42] = (Digital_Input_43_Get() != 0U) ? 1U : 0U;
    currentDIQueueBuffer[43] = (Digital_Input_44_Get() != 0U) ? 1U : 0U;
    currentDIQueueBuffer[44] = (Digital_Input_45_Get() != 0U) ? 1U : 0U;
    currentDIQueueBuffer[45] = (Digital_Input_46_Get() != 0U) ? 1U : 0U;
    currentDIQueueBuffer[46] = (Digital_Input_47_Get() != 0U) ? 1U : 0U;
    currentDIQueueBuffer[47] = (Digital_Input_48_Get() != 0U) ? 1U : 0U;
    currentDIQueueBuffer[48] = (Digital_Input_49_Get() != 0U) ? 1U : 0U;

    /* IO Expander Port 0 → DI 50–57 → buffer[49–56] */
    for (uint8_t i = 0U; i < 8U; i++)
    {
        currentDIQueueBuffer[49U + i] = ((u8Exp0 & (uint8_t)(1U << i)) != 0U) ? 1U : 0U;
    }

    /* IO Expander Port 1 → DI 58–60 → buffer[57–59] */
    for (uint8_t i = 0U; i < 3U; i++)
    {
        currentDIQueueBuffer[57U + i] = ((u8Exp1 & (uint8_t)(1U << i)) != 0U) ? 1U : 0U;
    }

    // /* Set static IP once on first call */
    // if (!s_boardSelectionDone)
    // {
    //     s_boardSelectionDone = true;
    //     prv_SetStaticIPAddress(COMPARTMENT_IP);
    // }
}

/* ============================================================================
 * IP Address Configuration
 * ========================================================================== */

/**
 * @brief  Disable DHCP and assign a static IP to the GMAC interface.
 * @param  pcIpStr  IP address string (e.g. COMPARTMENT_IP)
 */
static void prv_SetStaticIPAddress(const char *pcIpStr, const char *pcSubnetStr)
{
    if (pcIpStr == NULL || pcSubnetStr == NULL)
    {
        SYS_CONSOLE_PRINT("[Net] NULL input\r\n");
        return;
    }

    TCPIP_NET_HANDLE hNet = TCPIP_STACK_NetHandleGet("GMAC");
    if (hNet == NULL)
    {
        SYS_CONSOLE_PRINT("[Net] Failed to get GMAC handle\r\n");
        return;
    }

    (void)TCPIP_DHCP_Disable(hNet);

    IPV4_ADDR sIp, sMask;
    TCPIP_MAC_ADDR macAddr;
    uint8_t u8CompartmentId = SESSION_GetCompartmentId();
    macAddr.v[0] = APP_MAC_BASE_BYTE0;
    macAddr.v[1] = APP_MAC_BASE_BYTE1;
    macAddr.v[2] = APP_MAC_BASE_BYTE2;
    macAddr.v[3] = APP_MAC_BASE_BYTE3;
    macAddr.v[4] = APP_MAC_BASE_BYTE4;
    macAddr.v[5] = u8CompartmentId;

    SYS_CONSOLE_PRINT(
        "[NET] Setting MAC: %02X:%02X:%02X:%02X:%02X:%02X\r\n",
        macAddr.v[0],
        macAddr.v[1],
        macAddr.v[2],
        macAddr.v[3],
        macAddr.v[4],
        macAddr.v[5]);
    // TCPIP_STACK_NetAddressMacSet(hNet, &macAddr);

    if (!TCPIP_Helper_StringToIPAddress(pcIpStr, &sIp))
    {
        SYS_CONSOLE_PRINT("[Net] Invalid IP: %s\r\n", pcIpStr);
        return;
    }
    if (!TCPIP_Helper_StringToIPAddress(pcSubnetStr, &sMask))
    {
        SYS_CONSOLE_PRINT("[Net] Invalid subnet: %s\r\n", pcSubnetStr);
        return;
    }

    if (TCPIP_STACK_NetAddressSet(hNet, &sIp, &sMask, true))
    {
        SYS_CONSOLE_PRINT("[Net] Static IP set: %s\r\n", pcIpStr);
    }
    else
    {
        SYS_CONSOLE_PRINT("[Net] Failed to set static IP\r\n");
    }
}
/* ============================================================================
 * CRC-16/CCITT
 * ========================================================================== */

/**
 * @brief  Compute CRC-16/CCITT (poly=0x1021, init=0xFFFF).
 *
 *         BUG FIX: comment in original said "polynomial 0x8005" but code
 *         uses 0x1021 (CRC-16/CCITT). Comment corrected.
 *
 * @param  pu8Data  Pointer to input bytes (must not be NULL)
 * @param  u16Len   Number of bytes
 * @return uint16_t CRC result
 */
static uint16_t prv_CalculateCRC16(const uint8_t *pu8Data, uint16_t u16Len)
{
    if (pu8Data == NULL) { return 0U; }

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

/* Keep public name for any existing callers */
uint16_t CalculateCRC(uint8_t *data, uint16_t length)
{
    return prv_CalculateCRC16(data, length);
}

/* ============================================================================
 * GPIO Read / Write
 * ========================================================================== */

/**
 * @brief  Read a GPIO pin by port number.
 *
 * @param  u16PortNo    Port number (1–90)
 * @param  pu8ErrorCode Output error code (set to k_ErrInvalidPort if OOB)
 * @return uint8_t      Pin state (0 or 1), or 0 on error
 */
static uint8_t prv_GPIO_Read(uint16_t u16PortNo, uint8_t *pu8ErrorCode)
{
    if (pu8ErrorCode == NULL) { return 0U; }

    switch (u16PortNo)
    {
        /* Digital Inputs 1–49 */
        case  1U: return (uint8_t)(Digital_Input_1_Get()  != 0U);
        case  2U: return (uint8_t)(Digital_Input_2_Get()  != 0U);
        case  3U: return (uint8_t)(Digital_Input_3_Get()  != 0U);
        case  4U: return (uint8_t)(Digital_Input_4_Get()  != 0U);
        case  5U: return (uint8_t)(Digital_Input_5_Get()  != 0U);
        case  6U: return (uint8_t)(Digital_Input_6_Get()  != 0U);
        case  7U: return (uint8_t)(Digital_Input_7_Get()  != 0U);
        case  8U: return (uint8_t)(Digital_Input_8_Get()  != 0U);
        case  9U: return (uint8_t)(Digital_Input_9_Get()  != 0U);
        case 10U: return (uint8_t)(Digital_Input_10_Get() != 0U);
        case 11U: return (uint8_t)(Digital_Input_11_Get() != 0U);
        case 12U: return (uint8_t)(Digital_Input_12_Get() != 0U);
        case 13U: return (uint8_t)(Digital_Input_13_Get() != 0U);
        case 14U: return (uint8_t)(Digital_Input_14_Get() != 0U);
        case 15U: return (uint8_t)(Digital_Input_15_Get() != 0U);
        case 16U: return (uint8_t)(Digital_Input_16_Get() != 0U);
        case 17U: return (uint8_t)(Digital_Input_17_Get() != 0U);
        case 18U: return (uint8_t)(Digital_Input_18_Get() != 0U);
        case 19U: return (uint8_t)(Digital_Input_19_Get() != 0U);
        case 20U: return (uint8_t)(Digital_Input_20_Get() != 0U);
        case 21U: return (uint8_t)(Digital_Input_21_Get() != 0U);
        case 22U: return (uint8_t)(Digital_Input_22_Get() != 0U);
        case 23U: return (uint8_t)(Digital_Input_23_Get() != 0U);
        case 24U: return (uint8_t)(Digital_Input_24_Get() != 0U);
        case 25U: return (uint8_t)(Digital_Input_25_Get() != 0U);
        case 26U: return (uint8_t)(Digital_Input_26_Get() != 0U);
        case 27U: return (uint8_t)(Digital_Input_27_Get() != 0U);
        case 28U: return (uint8_t)(Digital_Input_28_Get() != 0U);
        case 29U: return (uint8_t)(Digital_Input_29_Get() != 0U);
        case 30U: return (uint8_t)(Digital_Input_30_Get() != 0U);
        case 31U: return (uint8_t)(Digital_Input_31_Get() != 0U);
        case 32U: return (uint8_t)(Digital_Input_32_Get() != 0U);
        case 33U: return (uint8_t)(Digital_Input_33_Get() != 0U);
        case 34U: return (uint8_t)(Digital_Input_34_Get() != 0U);
        case 35U: return (uint8_t)(Digital_Input_35_Get() != 0U);
        case 36U: return (uint8_t)(Digital_Input_36_Get() != 0U);
        case 37U: return (uint8_t)(Digital_Input_37_Get() != 0U);
        case 38U: return (uint8_t)(Digital_Input_38_Get() != 0U);
        case 39U: return (uint8_t)(Digital_Input_39_Get() != 0U);
        case 40U: return (uint8_t)(Digital_Input_40_Get() != 0U);
        case 41U: return (uint8_t)(Digital_Input_41_Get() != 0U);
        case 42U: return (uint8_t)(Digital_Input_42_Get() != 0U);
        case 43U: return (uint8_t)(Digital_Input_43_Get() != 0U);
        case 44U: return (uint8_t)(Digital_Input_44_Get() != 0U);
        case 45U: return (uint8_t)(Digital_Input_45_Get() != 0U);
        case 46U: return (uint8_t)(Digital_Input_46_Get() != 0U);
        case 47U: return (uint8_t)(Digital_Input_47_Get() != 0U);
        case 48U: return (uint8_t)(Digital_Input_48_Get() != 0U);
        case 49U: return (uint8_t)(Digital_Input_49_Get() != 0U);
        /* IO Expander inputs 50–60 */
        case 50U: return currentDIQueueBuffer[49];
        case 51U: return currentDIQueueBuffer[50];
        case 52U: return currentDIQueueBuffer[51];
        case 53U: return currentDIQueueBuffer[52];
        case 54U: return currentDIQueueBuffer[53];
        case 55U: return currentDIQueueBuffer[54];
        case 56U: return currentDIQueueBuffer[55];
        case 57U: return currentDIQueueBuffer[56];
        case 58U: return currentDIQueueBuffer[57];
        case 59U: return currentDIQueueBuffer[58];
        case 60U: return currentDIQueueBuffer[59];
        /* Digital Outputs (read-back) 61–84 */
        case 61U: return (uint8_t)(Digital_Output_1_Get()  != 0U);
        case 62U: return (uint8_t)(Digital_Output_2_Get()  != 0U);
        case 63U: return (uint8_t)(Digital_Output_3_Get()  != 0U);
        case 64U: return (uint8_t)(Digital_Output_4_Get()  != 0U);
        case 65U: return (uint8_t)(Digital_Output_5_Get()  != 0U);
        case 66U: return (uint8_t)(Digital_Output_6_Get()  != 0U);
        case 67U: return (uint8_t)(Digital_Output_7_Get()  != 0U);
        case 68U: return (uint8_t)(Digital_Output_8_Get()  != 0U);
        case 69U: return (uint8_t)(Digital_Output_9_Get()  != 0U);
        case 70U: return (uint8_t)(Digital_Output_10_Get() != 0U);
        case 71U: return (uint8_t)(Digital_Output_11_Get() != 0U);
        case 72U: return (uint8_t)(Digital_Output_12_Get() != 0U);
        case 73U: return (uint8_t)(Digital_Output_13_Get() != 0U);
        case 74U: return (uint8_t)(Digital_Output_14_Get() != 0U);
        case 75U: return (uint8_t)(Digital_Output_15_Get() != 0U);
        case 76U: return (uint8_t)(Digital_Output_16_Get() != 0U);
        case 77U: return (uint8_t)(Digital_Output_17_Get() != 0U);
        case 78U: return (uint8_t)(Digital_Output_18_Get() != 0U);
        case 79U: return (uint8_t)(Digital_Output_19_Get() != 0U);
        case 80U: return (uint8_t)(Digital_Output_20_Get() != 0U);
        case 81U: return (uint8_t)(Digital_Output_21_Get() != 0U);
        case 82U: return (uint8_t)(Digital_Output_22_Get() != 0U);
        case 83U: return (uint8_t)(Digital_Output_23_Get() != 0U);
        case 84U: return (uint8_t)(Digital_Output_24_Get() != 0U);
        /* eFuse + relay 85–90 */
        case 85U: return (uint8_t)(efuse1_in_Get()      != 0U);
        case 86U: return (uint8_t)(efuse2_in_Get()      != 0U);
        case 87U: return (uint8_t)(efuse3_in_Get()      != 0U);
        case 88U: return (uint8_t)(efuse4_in_Get()      != 0U);
        case 89U: return (uint8_t)(Relay_Output_1_Get() != 0U);
        case 90U: return (uint8_t)(Relay_Output_2_Get() != 0U);
        /* RS485 card-1 (91-99) card-2 (100-108) */
        // case 91U: return (uint8_t)(RS485_Output_1_Get() != 0U);
        // case 92U: return (uint8_t)(RS485_Output_2_Get() != 0U);
        // case 93U: return (uint8_t)(RS485_Output_3_Get() != 0U);
        // case 94U: return (uint8_t)(RS485_Output_4_Get() != 0U);
        // case 95U: return (uint8_t)(RS485_Output_5_Get() != 0U);
        // case 96U: return (uint8_t)(RS485_Output_6_Get() != 0U);
        // case 97U: return (uint8_t)(RS485_Output_7_Get() != 0U);
        // case 98U: return (uint8_t)(RS485_Output_8_Get() != 0U);
        // case 99U: return (uint8_t)(RS485_Output_9_Get() != 0U);
        // case 100U: return (uint8_t)(RS485_Output_10_Get() != 0U);
        // case 101U: return (uint8_t)(RS485_Output_11_Get() != 0U);
        // case 102U: return (uint8_t)(RS485_Output_12_Get() != 0U);
        // case 103U: return (uint8_t)(RS485_Output_13_Get() != 0U);
        // case 104U: return (uint8_t)(RS485_Output_14_Get() != 0U);
        // case 105U: return (uint8_t)(RS485_Output_15_Get() != 0U);
        // case 106U: return (uint8_t)(RS485_Output_16_Get() != 0U);
        // case 107U: return (uint8_t)(RS485_Output_17_Get() != 0U);
        // case 108U: return (uint8_t)(RS485_Output_18_Get() != 0U);
        default:
            *pu8ErrorCode = k_ErrInvalidPort;
            return 0U;
    }
}

/**
 * @brief  Write a GPIO pin by port number.
 *
 * @param  u16PortNo    Port number (61–108 for outputs)
 * @param  bVal         true = set, false = clear
 * @param  pu8ErrorCode Output error code
 */
static void prv_GPIO_Write(uint16_t u16PortNo, uint8_t u8Val, uint8_t *pu8ErrorCode)
{
    if (pu8ErrorCode == NULL) { return; }
    if ((u16PortNo < 61U) || (u16PortNo > 108U))
    {
        *pu8ErrorCode = k_ErrInvalidPort;
        return;
    }
    // static bool befuseDen = true;
    // if (befuseDen)
    // {
    //     efuse1_den_Set();
    //     efuse2_den_Set();
    //     efuse3_den_Set();
    //     efuse4_den_Set();
    //     befuseDen = false;
    // }
    switch (u16PortNo)
    {
        case 61U: u8Val ? Digital_Output_1_Set()  : Digital_Output_1_Clear();  break;
        case 62U: u8Val ? Digital_Output_2_Set()  : Digital_Output_2_Clear();  break;
        case 63U: u8Val ? Digital_Output_3_Set()  : Digital_Output_3_Clear();  break;
        case 64U: u8Val ? Digital_Output_4_Set()  : Digital_Output_4_Clear();  break;
        case 65U: u8Val ? Digital_Output_5_Set()  : Digital_Output_5_Clear();  break;
        case 66U: u8Val ? Digital_Output_6_Set()  : Digital_Output_6_Clear();  break;
        case 67U: u8Val ? Digital_Output_7_Set()  : Digital_Output_7_Clear();  break;
        case 68U: u8Val ? Digital_Output_8_Set()  : Digital_Output_8_Clear();  break;
        case 69U: u8Val ? Digital_Output_9_Set()  : Digital_Output_9_Clear();  break;
        case 70U: u8Val ? Digital_Output_10_Set() : Digital_Output_10_Clear(); break;
        case 71U: u8Val ? Digital_Output_11_Set() : Digital_Output_11_Clear(); break;
        case 72U: u8Val ? Digital_Output_12_Set() : Digital_Output_12_Clear(); break;
        case 73U: u8Val ? Digital_Output_13_Set() : Digital_Output_13_Clear(); break;
        case 74U: u8Val ? Digital_Output_14_Set() : Digital_Output_14_Clear(); break;
        case 75U: u8Val ? Digital_Output_15_Set() : Digital_Output_15_Clear(); break;
        case 76U: u8Val ? Digital_Output_16_Set() : Digital_Output_16_Clear(); break;
        case 77U: u8Val ? Digital_Output_17_Set() : Digital_Output_17_Clear(); break;
        case 78U: u8Val ? Digital_Output_18_Set() : Digital_Output_18_Clear(); break;
        case 79U: u8Val ? Digital_Output_19_Set() : Digital_Output_19_Clear(); break;
        case 80U: u8Val ? Digital_Output_20_Set() : Digital_Output_20_Clear(); break;
        case 81U: u8Val ? Digital_Output_21_Set() : Digital_Output_21_Clear(); break;
        case 82U: u8Val ? Digital_Output_22_Set() : Digital_Output_22_Clear(); break;
        case 83U: u8Val ? Digital_Output_23_Set() : Digital_Output_23_Clear(); break;
        case 84U: u8Val ? Digital_Output_24_Set() : Digital_Output_24_Clear(); break;
        case 85U: u8Val ? efuse1_in_Set()       : efuse1_in_Clear();       break;
        case 86U: u8Val ? efuse2_in_Set()       : efuse2_in_Clear();       break;
        case 87U: u8Val ? efuse3_in_Set()       : efuse3_in_Clear();       break;
        case 88U: u8Val ? efuse4_in_Set()       : efuse4_in_Clear();       break;
        case 89U: u8Val ? Relay_Output_1_Set()  : Relay_Output_1_Clear();  break;
        case 90U: u8Val ? Relay_Output_2_Set()  : Relay_Output_2_Clear();  break;
        /* RS485 Outputs card-1(91-99) card-2(100-108) */
        case 91U: RS485_Output_1_Set(u8Val); break;
        case 92U: RS485_Output_2_Set(u8Val); break;
        case 93U: RS485_Output_3_Set(u8Val); break;
        case 94U: RS485_Output_4_Set(u8Val); break;
        case 95U: RS485_Output_5_Set(u8Val); break;
        case 96U: RS485_Output_6_Set(u8Val); break;
        case 97U: RS485_Output_7_Set(u8Val); break;
        case 98U: RS485_Output_8_Set(u8Val); break;
        case 99U: RS485_Output_9_Set(u8Val); break;
        case 100U: RS485_Output_10_Set(u8Val); break;
        case 101U: RS485_Output_11_Set(u8Val); break;
        case 102U: RS485_Output_12_Set(u8Val); break;
        case 103U: RS485_Output_13_Set(u8Val); break;
        case 104U: RS485_Output_14_Set(u8Val); break;
        case 105U: RS485_Output_15_Set(u8Val); break;
        case 106U: RS485_Output_16_Set(u8Val); break;
        case 107U: RS485_Output_17_Set(u8Val); break;
        case 108U: RS485_Output_18_Set(u8Val); break;
        default:  *pu8ErrorCode = k_ErrInvalidPort; break;
    }
}

/* ============================================================================
 * High-level GPIO Operation API
 * ========================================================================== */

/**
 * @brief  Perform a named GPIO operation for the specified dock.
 *
 *         BUG FIX: original function had no `return bRet` at the end —
 *         the function would return an indeterminate value for all cases
 *         (undefined behaviour). Now all paths return explicitly.
 *
 *         BUG FIX: no bounds check on u8DockNo before using it as array index
 *         into k_GpioConf — added guard for per-dock operations.
 *
 * @param  eGPIOType  Operation
 * @param  u8DockNo   Dock index (used for per-dock operations)
 * @param  eGPIODirection  GPIO direction (GPIO_READ or GPIO_WRITE)
 * @return bool       Pin state for reads; true for writes; false on error
 */
bool bGPIO_Operation(GPIOOperation_e eGPIOType, uint8_t u8DockNo, GPIO_Direction_e eGPIODirection)
{
    uint8_t u8Err = k_ErrNone;
    uint16_t u16PinNo = 0U;
    uint8_t u8WriteVal = 0U;
    uint8_t u8IsInputOp = 0U;

    /* Per-dock operations need a valid dock index */
    bool bPerDockOp = (eGPIOType != DO_COMPARTMENT_FAN_HIGH) &&
                      (eGPIOType != DO_COMPARTMENT_FAN_LOW)  &&
                      (eGPIOType != DI_E_STOP_STATUS);

    if (bPerDockOp &&
        ((u8DockNo < (uint8_t)DOCK_1) || (u8DockNo > (uint8_t)SESSION_GetMaxDocks())))
    {
        SYS_CONSOLE_PRINT("[GPIO] Invalid dock %u for op %u\r\n",
                          (unsigned)u8DockNo, (unsigned)eGPIOType);
        return false;
    }

    /* Determine pin and operation type */
    switch (eGPIOType)
    {
        case DO_AC_RELAY_ON:
            u16PinNo = k_GpioConf.AC_Relay_Pin[u8DockNo];
            u8WriteVal = 1U;
            break;
        case DO_AC_RELAY_OFF:
            u16PinNo = k_GpioConf.AC_Relay_Pin[u8DockNo];
            u8WriteVal = 0U;
            break;
        case DO_DC_RELAY_ON:
            u16PinNo = k_GpioConf.DC_Relay_Pin[u8DockNo];
            u8WriteVal = 1U;
            break;
        case DO_DC_RELAY_OFF:
            u16PinNo = k_GpioConf.DC_Relay_Pin[u8DockNo];
            u8WriteVal = 0U;
            break;
        case DO_SOLENOID_HIGH:
            u16PinNo = k_GpioConf.Solenoid_PinHi[u8DockNo];
            u8WriteVal = 1U; /* Pulse high */
            break;
        case DO_SOLENOID_LOW:
            u16PinNo = k_GpioConf.Solenoid_PinLo[u8DockNo];
            u8WriteVal = 0U; /* Pulse low */
            break;
        case DO_R_LED_HIGH:
            u16PinNo = k_GpioConf.R_LED_Pin[u8DockNo];
            u8WriteVal = 1U;
            break;
        case DO_R_LED_LOW:
            u16PinNo = k_GpioConf.R_LED_Pin[u8DockNo];
            u8WriteVal = 0U;
            break;
        case DO_G_LED_HIGH:
            u16PinNo = k_GpioConf.G_LED_Pin[u8DockNo];
            u8WriteVal = 1U;
            break;
        case DO_G_LED_LOW:
            u16PinNo = k_GpioConf.G_LED_Pin[u8DockNo];
            u8WriteVal = 0U;
            break;
        case DO_B_LED_HIGH:
            u16PinNo = k_GpioConf.B_LED_Pin[u8DockNo];
            u8WriteVal = 1U;
            break;
        case DO_B_LED_LOW:
            u16PinNo = k_GpioConf.B_LED_Pin[u8DockNo];
            u8WriteVal = 0U;
            break;
        case DO_DOCK_FAN_HIGH:
            u16PinNo = k_GpioConf.Dock_Fan_Pin[u8DockNo];
            u8WriteVal = 1U;
            break;
        case DO_DOCK_FAN_LOW:
            u16PinNo = k_GpioConf.Dock_Fan_Pin[u8DockNo];
            u8WriteVal = 0U;
            break;
        case DO_COMPARTMENT_FAN_HIGH:
            u16PinNo = k_GpioConf.Compartment_Fan_Pin;
            u8WriteVal = 1U;
            break;
        case DO_COMPARTMENT_FAN_LOW:
            u16PinNo = k_GpioConf.Compartment_Fan_Pin;
            u8WriteVal = 0U;
            break;
        case DI_E_STOP_STATUS:
            u16PinNo = k_GpioConf.EStop_Pin;
            u8IsInputOp = 1U;
            break;
        case DI_DOOR_LOCK_STATUS:
            u16PinNo = k_GpioConf.DoorLock_Pin[u8DockNo];
            u8IsInputOp = 1U;
            break;
        case DI_SOLENOID_LOCK_STATUS:
            u16PinNo = k_GpioConf.SolenoidLock_Pin[u8DockNo];
            u8IsInputOp = 1U;
            break;
        case DI_BP_STATUS:
            u16PinNo = k_GpioConf.IgnitionSence_Pin[u8DockNo];
            u8IsInputOp = 1U;
            break;
        default:
            SYS_CONSOLE_PRINT("[GPIO] Unknown operation %u\r\n", (unsigned)eGPIOType);
            return false;
    }
    if (u16PinNo == 0U)
    {
        SYS_CONSOLE_PRINT("[GPIO] No pin configured for op %u dock %u\r\n",
                          (unsigned)eGPIOType, (unsigned)u8DockNo);
        return false;
    }
    if (eGPIODirection == GPIO_READ)
    {
        /* Read operation: allowed for all pins */
        return (prv_GPIO_Read(u16PinNo, &u8Err) != 0U) && (u8Err == k_ErrNone);
    }
    else
    {
        /* Write operation: only allowed for output pins */
        if (u8IsInputOp)
        {
            SYS_CONSOLE_PRINT("[GPIO] Write not allowed on input pin for op %u\r\n", (unsigned)eGPIOType);
            return false;
        }

        /* Perform write */
        if ((eGPIOType == DO_SOLENOID_HIGH) || (eGPIOType == DO_SOLENOID_LOW))
        {
            /* Special pulse for solenoid */
            prv_GPIO_Write(u16PinNo, u8WriteVal, &u8Err);
            vTaskDelay(pdMS_TO_TICKS(SOLENOID_PULSE_MS));
            prv_GPIO_Write(u16PinNo, 0U, &u8Err); /* Clear after pulse */
        }
        else
        {
            prv_GPIO_Write(u16PinNo, u8WriteVal, &u8Err);
        }
        return (u8Err == k_ErrNone);
    }
}

/* ============================================================================
 * Analog Input Get
 * ========================================================================== */

/**
 * @brief  Return the last averaged ADC value for a given channel.
 *
 *         BUG FIX: original used `channel - 1` without guarding against
 *         channel == 0 → underflow → OOB read at index 0xFF.
 *         Now uses zero-based channel index directly.
 *
 * @param  u8Channel  Channel (1-based as used by TCP command)
 * @return uint32_t   16-bit temperature value (0.01 °C units) or 0 on OOB
 */
static uint32_t prv_AnalogInputGet(uint8_t u8Channel)
{
    if ((u8Channel == 0U) ||
        (u8Channel > (uint8_t)NUM_TEMPERATURE_ANALOG_PINS))
    {
        SYS_CONSOLE_PRINT("[ADC] Invalid channel %u\r\n", (unsigned)u8Channel);
        return 0U;
    }
    return (uint32_t)currentTempAIQueueBuffer[u8Channel - 1U];
}

/* ============================================================================
 * Charging Control
 * ========================================================================== */

/**
 * @brief  Set or clear the authentication command for a dock.
 *
 * @param  u8DockNo  Dock index
 * @param  u8Action  1 = start, 0 = stop
 * @return uint8_t   1 on success
 */
static uint8_t prv_ChargingControl(uint8_t u8DockNo, uint8_t u8Action)
{
    static uint8_t au8PrevState[MAX_DOCKS] = {0};

    if (u8DockNo > (uint8_t)SESSION_GetMaxDocks())
    {
        SYS_CONSOLE_PRINT("[CHARGING] Invalid dock %u\r\n", (unsigned)u8DockNo);
        return 0U;
    }

    if (au8PrevState[u8DockNo] != u8Action)
    {
        SYS_CONSOLE_PRINT("[CHARGING] Dock %u %s\r\n",
                          (unsigned)u8DockNo,
                          (u8Action != 0U) ? "START" : "STOP");
        SESSION_SetAuthenticationCommand(u8DockNo, u8Action);
        au8PrevState[u8DockNo] = u8Action;
    }
    return 1U;
}

/* ============================================================================
 * TCP Frame Handling
 * ========================================================================== */

/**
 * @brief  Build and send a TCP response frame.
 *
 *         BUG FIX: no bounds check on u8Len — a large payload could overflow
 *         s_outbuf[256]. Added guard.
 *
 * @param  pu8Payload  Payload bytes
 * @param  u8Len       Payload length
 */
static void prv_SendResponsePayload(const uint8_t *pu8Payload, uint8_t u8Len)
{
    if (pu8Payload == NULL)
    {
        SYS_CONSOLE_PRINT("[TCP] SendResponse: NULL payload\r\n");
        return;
    }

    uint16_t u16TotalSize = (uint16_t)FRAME_HEADER_SIZE +
                            (uint16_t)u8Len             +
                            (uint16_t)FRAME_CRC_SIZE;

    if (u16TotalSize > (uint16_t)PKT_BUF_SIZE)
    {
        SYS_CONSOLE_PRINT("[TCP] SendResponse: payload too large (%u)\r\n",
                          (unsigned)u8Len);
        return;
    }

    uint16_t u16Idx = 0U;

    /* Unique ID (4 bytes, big-endian) */
    s_outbuf[u16Idx++] = (uint8_t)((s_reqFrame.uniqueId >> 24U) & 0xFFU);
    s_outbuf[u16Idx++] = (uint8_t)((s_reqFrame.uniqueId >> 16U) & 0xFFU);
    s_outbuf[u16Idx++] = (uint8_t)((s_reqFrame.uniqueId >>  8U) & 0xFFU);
    s_outbuf[u16Idx++] = (uint8_t)( s_reqFrame.uniqueId         & 0xFFU);

    /* Message type = Response (0x0002) */
    s_outbuf[u16Idx++] = RESPONSE_MSG_TYPE_HI;
    s_outbuf[u16Idx++] = RESPONSE_MSG_TYPE_LO;

    /* Routing */
    s_outbuf[u16Idx++] = s_reqFrame.compartmentId;
    s_outbuf[u16Idx++] = s_reqFrame.dockId;
    s_outbuf[u16Idx++] = s_reqFrame.commandId;

    /* Payload length */
    s_outbuf[u16Idx++] = u8Len;

    /* Payload */
    (void)memcpy(&s_outbuf[u16Idx], pu8Payload, (size_t)u8Len);
    u16Idx += (uint16_t)u8Len;

    /* CRC */
    uint16_t u16CRC = prv_CalculateCRC16(s_outbuf, u16Idx);
    s_outbuf[u16Idx++] = (uint8_t)((u16CRC >> 8U) & 0xFFU);
    s_outbuf[u16Idx++] = (uint8_t)( u16CRC         & 0xFFU);

    /*SYS_CONSOLE_PRINT("[TCP] Sending response: UID=0x%08X, Cmd=0x%02X, PayloadLen=%u\r\n",
                      (unsigned)s_reqFrame.uniqueId,
                      (unsigned)s_reqFrame.commandId,
                      (unsigned)u8Len);*/
    uint16_t u16Sent = TCPIP_TCP_ArrayPut(s_ioSocket, s_outbuf, u16Idx);
    //SYS_CONSOLE_PRINT("[TCP] Sent %u bytes\r\n", (unsigned)u16Sent);
}

/**
 * @brief  Build and dispatch an error response frame (legacy path).
 *
 *         BUG FIX: directly accessed `Reqframe_St.payload[0/1/2]` without
 *         checking `payloadLen >= 3` — OOB read on short payloads.
 *         Now clamps the payload copy.
 */
static void prv_PrepareDispatchOutbPacket(void)
{
    uint8_t au8Payload[6] = {0U};
    uint8_t u8Idx = 0U;

    /* Copy up to 3 bytes from request payload safely */
    uint8_t u8CopyLen = (s_reqFrame.payloadLen >= 3U) ? 3U : s_reqFrame.payloadLen;
    if ((s_reqFrame.payload != NULL) && (u8CopyLen > 0U))
    {
        (void)memcpy(au8Payload, s_reqFrame.payload, (size_t)u8CopyLen);
    }
    u8Idx = 3U; /* Reserve first 3 bytes for echoed payload */

    if (s_outbErrorCode != (char)k_ErrNone)
    {
        au8Payload[u8Idx++] = (uint8_t)s_outbErrorCode;
        prv_SendResponsePayload(au8Payload, u8Idx);
    }
    else
    {
        au8Payload[u8Idx++] = k_ErrNone;
        /* Echo port value */
        if (s_inbMsgSubType == (char)k_MsgDigitalWrite)
        {
            au8Payload[u8Idx++] = (s_inbPortVal != 0) ? 1U : 0U;
        }
        else
        {
            au8Payload[u8Idx++] = (s_outbPortVal != 0) ? 1U : 0U;
        }
        prv_SendResponsePayload(au8Payload, u8Idx);
    }
}

/**
 * @brief  Reset all inbound/outbound frame state.
 *
 *         BUG FIX: original used two 256-iteration for-loops to zero buffers.
 *         Replaced with memset.
 */
static void prv_ResetFrameState(void)
{
    s_inbMsgSubType = 0x00;
    s_inbPortVal    = -1;
    s_outbErrorCode = (char)k_ErrNone;
    s_outbPortVal   = -1;

    (void)memset(&s_reqFrame,  0, sizeof(s_reqFrame));
    (void)memset(s_outbuf,     0, sizeof(s_outbuf));
}

/* ============================================================================
 * Command Handlers
 * ========================================================================== */

static void prv_HandleGPIOCommand(void)
{
    uint8_t  u8MsgType  = s_reqFrame.payload[0];
    uint16_t u16PortNo  = ((uint16_t)s_reqFrame.payload[1] << 8U) |
                           (uint16_t)s_reqFrame.payload[2];
    uint8_t  u8PortVal  = 0U;
    uint8_t  u8OutVal   = 0U;
    uint8_t  u8ErrorCode = k_ErrNone;

    /* Payload size validation */
    if ((u8MsgType == k_MsgDigitalWrite) && (s_reqFrame.payloadLen != 4U))
    {
        SYS_CONSOLE_PRINT("[GPIO] Write payload size mismatch\r\n");
        u8ErrorCode = k_ErrInvalidMsg;
    }
    else if ((u8MsgType == k_MsgDigitalRead) && (s_reqFrame.payloadLen != 3U))
    {
        SYS_CONSOLE_PRINT("[GPIO] Read payload size mismatch\r\n");
        u8ErrorCode = k_ErrInvalidMsg;
    }

    /* Port range check */
    if ((u16PortNo < 1U) || (u16PortNo > 110U))
    {
        SYS_CONSOLE_PRINT("[GPIO] Invalid port %u\r\n", (unsigned)u16PortNo);
        u8ErrorCode = k_ErrInvalidPort;
    }

    /* BUG FIX: portVal check only applies to write operations */
    if (u8MsgType == k_MsgDigitalWrite)
    {
        u8PortVal = s_reqFrame.payload[3];
        if (u16PortNo >= 91U && u16PortNo <= 108U)
        {
            /* RS485 outputs allow values 0–3 (00=off, 01=LED_STATE_STEADY, 02=LED_STATE_SLOW_BLINK, 
            03=LED_STATE_MEDIUM_BLINK & 04=LED_STATE_FAST_BLINK) */
            if ((u8PortVal != 0U) && (u8PortVal != 1U) && (u8PortVal != 2U) && (u8PortVal != 3U) && (u8PortVal != 4U) && (u8PortVal != 5U))
            {
                SYS_CONSOLE_PRINT("[GPIO] Invalid value %u for RS485 port %u\r\n",
                                  (unsigned)u8PortVal, (unsigned)u16PortNo);
                u8ErrorCode = k_ErrInvalidValue;
            }
        }
        else if ((u8PortVal != 0U) && (u8PortVal != 1U))
        {
            SYS_CONSOLE_PRINT("[GPIO] Invalid value %u\r\n", (unsigned)u8PortVal);
            u8ErrorCode = k_ErrInvalidValue;
        }
    }

    if (u8ErrorCode == k_ErrNone)
    {
        if (u8MsgType == k_MsgDigitalRead)
        {
            u8OutVal = prv_GPIO_Read(u16PortNo, &u8ErrorCode);
        }
        else
        {
            prv_GPIO_Write(u16PortNo, u8PortVal, &u8ErrorCode);
        }
        // prv_StoreRelay_DigitalOutputsFrame();
    }

    uint8_t au8Resp[5];
    au8Resp[0] = u8MsgType;
    au8Resp[1] = (uint8_t)((u16PortNo >> 8U) & 0xFFU);
    au8Resp[2] = (uint8_t)( u16PortNo         & 0xFFU);
    au8Resp[3] = u8ErrorCode;
    au8Resp[4] = (u8MsgType == k_MsgDigitalWrite) ? u8PortVal : u8OutVal;

    prv_SendResponsePayload(au8Resp, 5U);
}

static void prv_HandleAnalogRead(void)
{
    if (s_reqFrame.payloadLen < 1U)
    {
        s_outbErrorCode = (char)k_ErrInvalidMsg;
        prv_PrepareDispatchOutbPacket();
        return;
    }

    uint8_t  u8Channel = s_reqFrame.payload[0];
    uint32_t u32Value  = prv_AnalogInputGet(u8Channel);

    SYS_CONSOLE_PRINT("[ANALOG] Ch%u = %lu\r\n", (unsigned)u8Channel, (unsigned long)u32Value);

    uint8_t au8Resp[5];
    au8Resp[0] = u8Channel;
    au8Resp[1] = (uint8_t)((u32Value >> 24U) & 0xFFU);
    au8Resp[2] = (uint8_t)((u32Value >> 16U) & 0xFFU);
    au8Resp[3] = (uint8_t)((u32Value >>  8U) & 0xFFU);
    au8Resp[4] = (uint8_t)( u32Value          & 0xFFU);

    prv_SendResponsePayload(au8Resp, 5U);
}

static void prv_HandleChargingCommand(uint8_t u8DockNo)
{
    if (s_reqFrame.payloadLen < 1U)
    {
        s_outbErrorCode = (char)k_ErrInvalidMsg;
        prv_PrepareDispatchOutbPacket();
        return;
    }

    uint8_t u8Action  = s_reqFrame.payload[0];
    uint8_t u8Status  = prv_ChargingControl(u8DockNo, u8Action);

    SYS_CONSOLE_PRINT("[CHARGING] Dock%u %s\r\n",
                      (unsigned)u8DockNo,
                      (u8Action != 0U) ? "START" : "STOP");

    prv_SendResponsePayload(&u8Status, 1U);
}

static void prv_HandleBootloaderCommand(void)
{
    SYS_CONSOLE_PRINT("[SYS] Bootloader trigger\r\n");

    ramStart[0] = BTL_TRIGGER_PATTERN;
    ramStart[1] = BTL_TRIGGER_PATTERN;
    ramStart[2] = BTL_TRIGGER_PATTERN;
    ramStart[3] = BTL_TRIGGER_PATTERN;

    DCACHE_CLEAN_BY_ADDR(ramStart, 16U);
    SYS_RESET_SoftwareReset();
}

/**
 * @brief  Parse and dispatch a received TCP frame.
 *
 *         BUG FIX: `uint8_t u8DockNo = ...` declared inside a `case` without
 *         braces — C99 violation. Moved to compound statement.
 *
 * @param  pu8Buf  Receive buffer
 * @param  u16Len  Number of bytes received
 */
static void prv_ParseAndProcessInbPacket(uint8_t *pu8Buf, uint16_t u16Len)
{
    if (pu8Buf == NULL)
    {
        SYS_CONSOLE_PRINT("[TCP] NULL buffer\r\n");
        return;
    }

    if (u16Len < (uint16_t)TCP_MIN_FRAME_SIZE)
    {
        SYS_CONSOLE_PRINT("[TCP] Frame too short (%u)\r\n", (unsigned)u16Len);
        return;
    }

    /* Parse fixed header */
    s_reqFrame.uniqueId      = ((uint32_t)pu8Buf[0] << 24U) |
                               ((uint32_t)pu8Buf[1] << 16U) |
                               ((uint32_t)pu8Buf[2] <<  8U) |
                                (uint32_t)pu8Buf[3];
    s_reqFrame.msgType       = ((uint16_t)pu8Buf[4] << 8U) | (uint16_t)pu8Buf[5];
    s_reqFrame.compartmentId = pu8Buf[6];
    s_reqFrame.dockId        = pu8Buf[7];
    s_reqFrame.commandId     = pu8Buf[8];
    s_reqFrame.payloadLen    = pu8Buf[9];
    s_reqFrame.payload       = &pu8Buf[10];   /* Non-owning pointer into caller's buffer */

    /* Validate total length */
    uint16_t u16Expected = (uint16_t)FRAME_HEADER_SIZE +
                           (uint16_t)s_reqFrame.payloadLen +
                           (uint16_t)FRAME_CRC_SIZE;

    if (u16Len != u16Expected)
    {
        SYS_CONSOLE_PRINT("[TCP] Length mismatch got=%u expected=%u\r\n",
                          (unsigned)u16Len, (unsigned)u16Expected);
        return;
    }
    uint8_t u8CompartmentId = SESSION_GetCompartmentId();
    if (s_reqFrame.compartmentId != u8CompartmentId)
    {
        SYS_CONSOLE_PRINT("[TCP] Compartment mismatch got=%u expected=%u\r\n",
                          (unsigned)s_reqFrame.compartmentId, (unsigned)u8CompartmentId);
        return;
    }
    /* CRC validation */
    uint16_t u16RxCRC   = ((uint16_t)pu8Buf[u16Len - 2U] << 8U) |
                           (uint16_t)pu8Buf[u16Len - 1U];
    uint16_t u16CalcCRC = prv_CalculateCRC16(pu8Buf, u16Len - 2U);

    if (u16RxCRC != u16CalcCRC)
    {
        SYS_CONSOLE_PRINT("[TCP] CRC mismatch rx=0x%04X calc=0x%04X\r\n",
                          (unsigned)u16RxCRC, (unsigned)u16CalcCRC);
        return;
    }

    if (s_reqFrame.msgType != (uint16_t)MSG_TYPE_REQUEST)
    {
        SYS_CONSOLE_PRINT("[TCP] Not a request frame (type=0x%04X)\r\n",
                          (unsigned)s_reqFrame.msgType);
        return;
    }

    // SYS_CONSOLE_PRINT("[TCP] Frame OK cmd=0x%02X dock=%u\r\n",
    //                   (unsigned)s_reqFrame.commandId,
    //                   (unsigned)s_reqFrame.dockId);

    /* BUG FIX: declare u8DockNo outside switch to avoid C99 cross-jump issue */
    uint8_t u8DockNo = s_reqFrame.dockId;

    switch (s_reqFrame.commandId)
    {
        case CMD_GPIO_OPERATION:
            prv_HandleGPIOCommand();
            break;

        case CMD_ANALOG_READ:
            prv_HandleAnalogRead();
            break;

        case CMD_CHARGING_COMMAND:
            prv_HandleChargingCommand(u8DockNo);
            break;

        case CMD_BOOT_MODE_COMMAND:
            prv_HandleBootloaderCommand();
            return; /* Reset — don't reach ResetFrameState */

        case CMD_SOFT_RESET_COMMAND:
            SYS_CONSOLE_PRINT("[SYS] Soft reset\r\n");
            SYS_RESET_SoftwareReset();
            return;

        case CMD_COMMISSIONING_COMMAND:
            prv_HandleCommissioningCommand();
            break;

        default:
            SYS_CONSOLE_PRINT("[TCP] Unknown command 0x%02X\r\n",
                              (unsigned)s_reqFrame.commandId);
            s_outbErrorCode = (char)k_ErrInvalidMsg;
            prv_PrepareDispatchOutbPacket();
            break;
    }

    prv_ResetFrameState();
}

/* ============================================================================
 * Commissioning Mode — Implementation
 * ========================================================================== */

/*
 * Architecture note
 * ─────────────────
 * Normal mode and commissioning mode use DIFFERENT TCP protocols:
 *
 *   Normal mode (prv_ParseAndProcessInbPacket):
 *     UniqueID(4) + MsgType(2) + CmpId(1) + DockId(1) + CmdId(1) +
 *     PayloadLen(1) + Payload(N) + CRC16(2)          ← 10-byte header
 *
 *   Commissioning mode (prv_ParseCommissioningPacket):
 *     Raw 37-byte IOC_Commisioning_t struct sent directly, NO header:
 *       [0]      msgType         0x00=READ / 0x01=WRITE
 *       [1]      u8CompartmentId
 *       [2]      u8MaxDocks
 *       [3..18]  cIpAddress[16]  (null-padded ASCII)
 *       [19..34] cSubnetMask[16] (null-padded ASCII)
 *       [35..36] CRC16-CCITT     (big-endian, covers bytes [0..34])
 *
 * The two parsers are completely separate and must NEVER be swapped.
 *
 * Normal mode enters commissioning mode via CMD_COMMISSIONING_COMMAND (0x07)
 * with a 1-byte payload of 0x01, which IS wrapped in the normal frame.
 * Once in commissioning mode all subsequent packets are raw structs.
 */

/* --------------------------------------------------------------------------
 * prv_StopCurrentServer
 * -------------------------------------------------------------------------- */
/**
 * @brief  Close the normal-mode IO socket gracefully before switching modes.
 */
static void prv_StopCurrentServer(void)
{
    if (s_ioSocket != INVALID_SOCKET)
    {
        SYS_CONSOLE_PRINT("[COMM] Stopping normal server\r\n");
        TCPIP_TCP_Close(s_ioSocket);
        s_ioSocket = INVALID_SOCKET;
    }
}

/* --------------------------------------------------------------------------
 * prv_StartCommissioningServer
 * -------------------------------------------------------------------------- */
/**
 * @brief  Bind the commissioning TCP server to COMM_SERVER_IP:COMM_SERVER_PORT.
 *
 *         Sets the static IP to 192.168.1.22 so the host tool can always reach
 *         the device at a known address regardless of its configured IP.
 */
static void prv_StartCommissioningServer(void)
{
    prv_SetStaticIPAddress(COMM_SERVER_IP, "255.255.255.0");

    s_commSocket = TCPIP_TCP_ServerOpen(IP_ADDRESS_TYPE_IPV4,
                                         COMM_SERVER_PORT, 0U);
    if (s_commSocket == INVALID_SOCKET)
    {
        SYS_CONSOLE_PRINT("[COMM] Failed to open server on port %u\r\n",
                          (unsigned)COMM_SERVER_PORT);
        return;
    }

    SYS_CONSOLE_PRINT("[COMM] Server started at %s:%u\r\n",
                      COMM_SERVER_IP, (unsigned)COMM_SERVER_PORT);
}

/* --------------------------------------------------------------------------
 * prv_HandleCommissioningCommand  (called from normal-mode command switch)
 * -------------------------------------------------------------------------- */
/**
 * @brief  Handle CMD_COMMISSIONING_COMMAND (0x07) received in NORMAL mode.
 *
 *         This function is ONLY called via prv_ParseAndProcessInbPacket() while
 *         the device is still in MODE_NORMAL.  The normal-mode frame wraps a
 *         1-byte payload:
 *           payload[0] = 0x01 (COMM_SUBCMD_ENTER) — the only valid value here.
 *
 *         On receiving 0x01:
 *           1. Closes the normal IO socket (prv_StopCurrentServer).
 *           2. Sets g_systemMode = MODE_COMMISSIONING.
 *           3. Sends a 1-byte ACK (0x00) back through the NORMAL socket
 *              (socket is still open for this one last response).
 *
 *         All subsequent commissioning traffic arrives as raw 37-byte structs
 *         on the dedicated commissioning socket (192.168.1.22:6235) and is
 *         handled by prv_ParseCommissioningPacket(), NOT by this function.
 */
static void prv_HandleCommissioningCommand(void)
{
    if (s_reqFrame.payloadLen < 1U)
    {
        SYS_CONSOLE_PRINT("[COMM] CMD_COMMISSIONING_COMMAND with empty payload\r\n");
        s_outbErrorCode = (char)k_ErrInvalidMsg;
        prv_PrepareDispatchOutbPacket();
        return;
    }

    uint8_t u8SubCmd = s_reqFrame.payload[0];

    if (u8SubCmd != COMM_SUBCMD_ENTER)
    {
        /* Only ENTER (0x01) is valid while in normal mode */
        SYS_CONSOLE_PRINT("[COMM] Unexpected sub-cmd 0x%02X in normal mode\r\n",
                          (unsigned)u8SubCmd);
        s_outbErrorCode = (char)k_ErrInvalidMsg;
        prv_PrepareDispatchOutbPacket();
        return;
    }

    SYS_CONSOLE_PRINT("[COMM] Entered commissioning mode\r\n");

    /* Send ACK on normal socket before closing it */
    uint8_t u8Ack = COMM_RESP_ACK;
    prv_SendResponsePayload(&u8Ack, 1U);

    /* Small delay so the TCP stack can flush the ACK */
    vTaskDelay(pdMS_TO_TICKS(50U));

    prv_StopCurrentServer();
    g_systemMode = MODE_COMMISSIONING;
}

/* --------------------------------------------------------------------------
 * prv_ParseCommissioningPacket  (dedicated commissioning-mode parser)
 * -------------------------------------------------------------------------- */
/**
 * @brief  Parse a raw 37-byte IOC_Commisioning_t packet and dispatch it.
 *
 *         Called exclusively from the commissioning branch of the task loop.
 *         The buffer must contain exactly IOC_COMMISSIONING_STRUCT_SIZE bytes.
 *
 *         Parsing steps:
 *           1. Length check — must be exactly 37 bytes.
 *           2. CRC16-CCITT check — computed over bytes [0..34], compared with
 *              bytes [35..36] (big-endian).
 *           3. Dispatch on msgType (byte [0]):
 *                0x00 → prv_ProcessCommissioningRead()
 *                0x01 → prv_ProcessCommissioningWrite(buf)
 *
 * @param  pu8Buf   Buffer containing the raw struct (caller-owned).
 * @param  u16Len   Number of bytes actually received.
 */
static void prv_ParseCommissioningPacket(uint8_t *pu8Buf, uint16_t u16Len)
{
    if (pu8Buf == NULL)
    {
        SYS_CONSOLE_PRINT("[COMM] NULL buffer\r\n");
        return;
    }

    /* ── Step 1: Length check ─────────────────────────────────────────────── */
    if (u16Len != (uint16_t)IOC_COMMISSIONING_STRUCT_SIZE)
    {
        SYS_CONSOLE_PRINT("[COMM] Length error: got %u expected %u\r\n",
                          (unsigned)u16Len,
                          (unsigned)IOC_COMMISSIONING_STRUCT_SIZE);
        uint8_t u8Nack = COMM_RESP_NACK_PAYLOAD_LEN;
        (void)TCPIP_TCP_ArrayPut(s_commSocket, &u8Nack, 1U);
        return;
    }

    /* ── Step 2: CRC16-CCITT over bytes [0..34] ───────────────────────────── */
    uint16_t u16RxCrc =
        ((uint16_t)pu8Buf[COMM_PAYLOAD_CRC_OFF]      << 8U) |
         (uint16_t)pu8Buf[COMM_PAYLOAD_CRC_OFF + 1U];

    uint16_t u16CalcCrc = prv_CalculateCRC16(pu8Buf,
                              (uint16_t)(IOC_COMMISSIONING_STRUCT_SIZE - 2U));

    if (u16RxCrc != u16CalcCrc)
    {
        SYS_CONSOLE_PRINT("[COMM] CRC error: rx=0x%04X calc=0x%04X\r\n",
                          (unsigned)u16RxCrc, (unsigned)u16CalcCrc);
        uint8_t u8Nack = COMM_RESP_NACK_CRC;
        (void)TCPIP_TCP_ArrayPut(s_commSocket, &u8Nack, 1U);
        return;
    }

    /* ── Step 3: Route on msgType (byte [0]) ─────────────────────────────── */
    uint8_t u8MsgType = pu8Buf[COMM_PAYLOAD_MSGTYPE_OFF];

    SYS_CONSOLE_PRINT("[COMM] Packet OK, msgType=0x%02X\r\n",
                      (unsigned)u8MsgType);

    switch (u8MsgType)
    {
        case COMM_SUBCMD_READ:   /* 0x00 — return current config */
            prv_ProcessCommissioningRead();
            break;

        case COMM_SUBCMD_WRITE:  /* 0x01 — apply new config */
            prv_ProcessCommissioningWrite(pu8Buf);
            break;

        default:
            SYS_CONSOLE_PRINT("[COMM] Unknown msgType 0x%02X\r\n",
                              (unsigned)u8MsgType);
            {
                uint8_t u8Nack = COMM_RESP_NACK_PAYLOAD_LEN;
                (void)TCPIP_TCP_ArrayPut(s_commSocket, &u8Nack, 1U);
            }
            break;
    }
}

/* --------------------------------------------------------------------------
 * prv_SaveIocConfigToFlash
 * -------------------------------------------------------------------------- */
/**
 * @brief  Persist the complete flash_data_t (including updated iocCfg) to flash.
 *
 *         Performs an atomic page-erase then row-write so that ALL existing
 *         fields (digitalOutputs, relayOutputs, serialNumber, canPorts, …) are
 *         preserved alongside the newly written iocCfg.
 *
 *         Both the erase and write operations are guarded by a timeout counter
 *         (ADC_TIMEOUT_CYCLES) to prevent an infinite busy-wait on hardware
 *         failure.
 */
static void prv_SaveIocConfigToFlash(void)
{
    uint32_t u32Timeout;
    uint8_t  au8Buf[sizeof(flash_data_t)];

    writeData.magic = FLASH_MAGIC;   /* Ensure magic marker is always valid */

    (void)memcpy(au8Buf, &writeData, sizeof(flash_data_t));
    DCACHE_CLEAN_BY_ADDR((uint32_t *)au8Buf, sizeof(au8Buf));

    FCW_PageErase(FLASH_ADDRESS);
    u32Timeout = ADC_TIMEOUT_CYCLES;
    while (FCW_IsBusy() && (u32Timeout > 0U))
    {
        u32Timeout--;
    }
    if (u32Timeout == 0U)
    {
        SYS_CONSOLE_PRINT("[COMM] Flash erase timeout!\r\n");
        return;
    }

    FCW_RowWrite((uint32_t *)au8Buf, FLASH_ADDRESS);
    u32Timeout = ADC_TIMEOUT_CYCLES;
    while (FCW_IsBusy() && (u32Timeout > 0U))
    {
        u32Timeout--;
    }
    if (u32Timeout == 0U)
    {
        SYS_CONSOLE_PRINT("[COMM] Flash write timeout!\r\n");
        return;
    }

    SYS_CONSOLE_PRINT("[COMM] IOC config saved to flash\r\n");
}

/* --------------------------------------------------------------------------
 * prv_ProcessCommissioningRead
 * -------------------------------------------------------------------------- */
/**
 * @brief  Respond to a READ request with the current IOC configuration.
 *
 *         Builds a fresh 37-byte IOC_Commisioning_t response from RAM, appends
 *         a valid CRC16-CCITT, and sends it directly on s_commSocket (raw,
 *         no normal-mode frame wrapping).
 *
 *         Response wire layout (37 bytes):
 *           [0]      msgType         = 0x00  (READ echo)
 *           [1]      u8CompartmentId (from writeData.iocCfg)
 *           [2]      u8MaxDocks       (from writeData.iocCfg)
 *           [3..18]  cIpAddress[16]  (null-padded ASCII)
 *           [19..34] cSubnetMask[16] (null-padded ASCII)
 *           [35..36] CRC16-CCITT     (big-endian, covers [0..34])
 */
static void prv_ProcessCommissioningRead(void)
{
    SYS_CONSOLE_PRINT("[COMM] Read config request received\r\n");

    uint8_t au8Resp[IOC_COMMISSIONING_STRUCT_SIZE];
    (void)memset(au8Resp, 0, sizeof(au8Resp));

    /* Byte 0: msgType echoed as READ (0x00) */
    au8Resp[COMM_PAYLOAD_MSGTYPE_OFF]  = COMM_SUBCMD_READ;

    /* Byte 1: CompartmentId */
    au8Resp[COMM_PAYLOAD_COMP_ID_OFF]  = writeData.iocCfg.u8CompartmentId;

    /* Byte 2: MaxDock */
    au8Resp[COMM_PAYLOAD_MAX_DOCK_OFF] = writeData.iocCfg.u8MaxDocks;

    /* Bytes 3..18: IP address (16 bytes, null-padded) */
    (void)memcpy(&au8Resp[COMM_PAYLOAD_IP_OFF],
                 writeData.iocCfg.cIpAddress,
                 sizeof(writeData.iocCfg.cIpAddress));

    /* Bytes 19..34: Subnet mask (16 bytes, null-padded) */
    (void)memcpy(&au8Resp[COMM_PAYLOAD_SUBNET_OFF],
                 writeData.iocCfg.cSubnetMask,
                 sizeof(writeData.iocCfg.cSubnetMask));

    /* Bytes 35..36: CRC16-CCITT over [0..34], big-endian */
    uint16_t u16Crc = prv_CalculateCRC16(au8Resp,
                          (uint16_t)(IOC_COMMISSIONING_STRUCT_SIZE - 2U));
    au8Resp[COMM_PAYLOAD_CRC_OFF]      = (uint8_t)((u16Crc >> 8U) & 0xFFU);
    au8Resp[COMM_PAYLOAD_CRC_OFF + 1U] = (uint8_t)( u16Crc        & 0xFFU);

    SYS_CONSOLE_PRINT("[COMM] Sending config:"
                      " Cmp=%u Dock=%u IP=%s Subnet=%s CRC=0x%04X\r\n",
                      (unsigned)au8Resp[COMM_PAYLOAD_COMP_ID_OFF],
                      (unsigned)au8Resp[COMM_PAYLOAD_MAX_DOCK_OFF],
                      &au8Resp[COMM_PAYLOAD_IP_OFF],
                      &au8Resp[COMM_PAYLOAD_SUBNET_OFF],
                      (unsigned)u16Crc);

    /* Send raw 37-byte struct directly on the commissioning socket */
    (void)TCPIP_TCP_ArrayPut(s_commSocket,
                              au8Resp,
                              (uint16_t)IOC_COMMISSIONING_STRUCT_SIZE);
}

/* --------------------------------------------------------------------------
 * prv_ProcessCommissioningWrite
 * -------------------------------------------------------------------------- */
/**
 * @brief  Validate and apply a WRITE configuration from the client.
 *
 *         The caller (prv_ParseCommissioningPacket) has already validated the
 *         packet length and CRC before calling this function.  pu8Struct points
 *         directly to the start of the 37-byte buffer, so field offsets map
 *         exactly to COMM_PAYLOAD_*_OFF macros.
 *
 *         Field layout reminder:
 *           pu8Struct[0]       = msgType  (0x01, already validated by caller)
 *           pu8Struct[1]       = u8CompartmentId
 *           pu8Struct[2]       = u8MaxDocks
 *           pu8Struct[3..18]   = cIpAddress[16]   (null-padded ASCII)
 *           pu8Struct[19..34]  = cSubnetMask[16]  (null-padded ASCII)
 *           pu8Struct[35..36]  = u16Crc (verified by caller, not re-checked here)
 *
 *         Validation performed here:
 *           1. u8MaxDocks in range [1..COMM_MAX_DOCK_LIMIT].
 *           2. cIpAddress parseable as IPv4 by TCPIP_Helper_StringToIPAddress.
 *           3. cSubnetMask parseable as IPv4 by TCPIP_Helper_StringToIPAddress.
 *
 *         On success:
 *           a. Updates writeData.iocCfg in RAM.
 *           b. Calls prv_SaveIocConfigToFlash() — full page write.
 *           c. Sends 1-byte ACK (0x00) on s_commSocket.
 *           d. Delays 200 ms so TCP stack can flush the ACK.
 *           e. Calls SYS_RESET_SoftwareReset().
 *
 * @param  pu8Struct  Pointer to the validated 37-byte IOC_Commisioning_t buffer.
 */
static void prv_ProcessCommissioningWrite(const uint8_t *pu8Struct)
{
    /* ── Step 1: Extract fields from the validated struct ─────────────────── */
    uint8_t u8CompId  = pu8Struct[COMM_PAYLOAD_COMP_ID_OFF];
    uint8_t u8MaxDock = pu8Struct[COMM_PAYLOAD_MAX_DOCK_OFF];

    /*
     * Copy IP and subnet into local null-terminated char arrays.
     * Copy only 15 bytes and force null at [15] to guard against a sender
     * that fills all 16 bytes without a null terminator.
     */
    char acIp[16];
    char acSubnet[16];
    (void)memcpy(acIp,     &pu8Struct[COMM_PAYLOAD_IP_OFF],     15U);
    (void)memcpy(acSubnet, &pu8Struct[COMM_PAYLOAD_SUBNET_OFF], 15U);
    acIp[15]     = '\0';
    acSubnet[15] = '\0';

    SYS_CONSOLE_PRINT("[COMM] Write request:"
                      " Cmp=%u Dock=%u IP=%s Subnet=%s\r\n",
                      (unsigned)u8CompId, (unsigned)u8MaxDock,
                      acIp, acSubnet);

    /* ── Step 2: Validate MaxDock ─────────────────────────────────────────── */
    if ((u8MaxDock < 1U) || (u8MaxDock > (uint8_t)COMM_MAX_DOCK_LIMIT))
    {
        SYS_CONSOLE_PRINT("[COMM] Invalid MaxDock=%u (valid 1..%u)\r\n",
                          (unsigned)u8MaxDock,
                          (unsigned)COMM_MAX_DOCK_LIMIT);
        uint8_t u8Nack = COMM_RESP_NACK_INVALID_FIELD;
        (void)TCPIP_TCP_ArrayPut(s_commSocket, &u8Nack, 1U);
        return;
    }

    /* ── Step 3: Validate IP address string ───────────────────────────────── */
    IPV4_ADDR sTmpIp;
    if (!TCPIP_Helper_StringToIPAddress(acIp, &sTmpIp))
    {
        SYS_CONSOLE_PRINT("[COMM] Invalid IP address: \"%s\"\r\n", acIp);
        uint8_t u8Nack = COMM_RESP_NACK_INVALID_FIELD;
        (void)TCPIP_TCP_ArrayPut(s_commSocket, &u8Nack, 1U);
        return;
    }

    /* ── Step 4: Validate subnet mask string ──────────────────────────────── */
    IPV4_ADDR sTmpMask;
    if (!TCPIP_Helper_StringToIPAddress(acSubnet, &sTmpMask))
    {
        SYS_CONSOLE_PRINT("[COMM] Invalid subnet mask: \"%s\"\r\n", acSubnet);
        uint8_t u8Nack = COMM_RESP_NACK_INVALID_FIELD;
        (void)TCPIP_TCP_ArrayPut(s_commSocket, &u8Nack, 1U);
        return;
    }

    /* ── Step 5: Update RAM configuration ────────────────────────────────── */
    writeData.iocCfg.u8CompartmentId = u8CompId;
    writeData.iocCfg.u8MaxDocks       = u8MaxDock;
    // Mac Address is fixed and read-only, set it once here to ensure it's always valid in flash
    MAC_WriteToFlash((uint8_t)u8CompId);
    (void)memcpy(writeData.iocCfg.cIpAddress,
                 acIp,
                 sizeof(writeData.iocCfg.cIpAddress));
    writeData.iocCfg.cIpAddress[15] = '\0';

    (void)memcpy(writeData.iocCfg.cSubnetMask,
                 acSubnet,
                 sizeof(writeData.iocCfg.cSubnetMask));
    writeData.iocCfg.cSubnetMask[15] = '\0';

    /* ── Step 6: Persist full flash_data_t to flash ───────────────────────── */
    /*
     * prv_SaveIocConfigToFlash() writes the entire writeData struct as one
     * atomic erase+write.  All existing non-IOC fields (digitalOutputs,
     * relayOutputs, canPorts, rs485Config, serialNumber, …) are preserved.
     */
    prv_SaveIocConfigToFlash();
    SYS_CONSOLE_PRINT("[COMM] Write config success\r\n");

    /* ── Step 7: ACK the client — send raw 1-byte response on comm socket ─── */
    uint8_t u8Ack = COMM_RESP_ACK;
    (void)TCPIP_TCP_ArrayPut(s_commSocket, &u8Ack, 1U);

    /* Allow TCP stack to flush the ACK before resetting */
    vTaskDelay(pdMS_TO_TICKS(200U));

    /* ── Step 8: Reboot — device loads new config from flash on next boot ─── */
    SYS_CONSOLE_PRINT("[COMM] Rebooting system\r\n");
    SYS_RESET_SoftwareReset();
}

/* ============================================================================
 * IO Expander Stubs (implemented elsewhere — stubs for reference)
 * ========================================================================== */

static void prv_vConfigureIOexpanders(void)
{
    /* Delegate to platform-specific expander init */
    (void)IOExpander1_Configure_Ports();
}

static void prv_SetStoredStaticIP(void)
{
    char *pcIp     = SESSION_GetIpAddress();
    char *pcSubnet = SESSION_GetSubnetMask();
    SYS_CONSOLE_PRINT("[NET] Applying stored IP config: IP=%s Subnet=%s\r\n",
                      pcIp, pcSubnet);
    /* Apply IP */
    prv_SetStaticIPAddress(pcIp, pcSubnet);
}
/* ============================================================================
 * Main IO Handler Task
 * ========================================================================== */

/**
 * @brief  FreeRTOS task: IO server.
 *
 *         BUG FIX: `vTaskDelay(RECONNECT_DELAY_MS)` — RECONNECT_DELAY_MS is
 *         in milliseconds but vTaskDelay takes ticks. Wrapped in pdMS_TO_TICKS.
 *         BUG FIX: `vTaskDelay(10)` — raw ticks, replaced with
 *         `pdMS_TO_TICKS(IO_TASK_LOOP_DELAY_MS)`.
 */
static void prv_IOHandlerServerTask(void *pvParameters)
{
    (void)pvParameters;

    prv_MovingAverage_Init_All();
    prv_vConfigureIOexpanders();

    s_ioSocket = TCPIP_TCP_ServerOpen(IP_ADDRESS_TYPE_IPV4, IO_SERVER_PORT_HEV, 0U);
    if (s_ioSocket == INVALID_SOCKET)
    {
        SYS_CONSOLE_PRINT("[IO] Failed to open TCP socket — task exiting\r\n");
        vTaskDelete(NULL);
        return;
    }
    prv_SetStoredStaticIP(); /* Ensure we have a valid IP for clients to connect to */
    SYS_CONSOLE_PRINT("[IO] Task started on port %u\r\n", IO_SERVER_PORT_HEV);

    while (true)
    {
        prv_ReadDigitalInputs();
        ReadAllAnalogInputPins();

        /* ── Commissioning mode: dedicated raw-struct server ─────────────── */
        /*
         * IMPORTANT: commissioning packets are NOT wrapped in the normal TCP
         * frame (no UniqueID / MsgType / CmpId / DockId / CmdId header).
         * The client sends the 37-byte IOC_Commisioning_t struct DIRECTLY.
         *
         * Raw wire layout (37 bytes, no header, no outer CRC wrapper):
         *
         *   Byte  0     : msgType         0x00 = READ, 0x01 = WRITE
         *   Byte  1     : u8CompartmentId
         *   Byte  2     : u8MaxDocks
         *   Bytes 3..18 : cIpAddress[16]  (null-padded ASCII)
         *   Bytes 19..34: cSubnetMask[16] (null-padded ASCII)
         *   Bytes 35..36: u16Crc          (CRC16-CCITT, big-endian, covers [0..34])
         *
         * Example WRITE frame (37 bytes):
         *   01 01 03
         *   31 39 32 2E 31 36 38 2E 31 2E 32 33 31 00 00 00  "192.168.1.231"
         *   32 35 35 2E 32 35 35 2E 32 35 35 2E 30 00 00 00  "255.255.255.0"
         *   B0 C4
         *
         * prv_ParseAndProcessInbPacket() is intentionally NOT used here because
         * it expects the 10-byte normal-mode frame header which does not exist
         * in commissioning packets.  prv_ParseCommissioningPacket() handles this
         * raw format exclusively.
         */
        if (g_systemMode == MODE_COMMISSIONING)
        {
            /* Open the commissioning server the first time we enter this mode */
            if (s_commSocket == INVALID_SOCKET)
            {
                prv_StartCommissioningServer();
            }

            if ((s_commSocket != INVALID_SOCKET) &&
                TCPIP_TCP_IsConnected(s_commSocket))
            {
                /* Buffer sized exactly for one IOC_Commisioning_t struct */
                uint8_t au8CommBuf[IOC_COMMISSIONING_STRUCT_SIZE];
                (void)memset(au8CommBuf, 0, sizeof(au8CommBuf));

                int16_t i16Bytes = TCPIP_TCP_ArrayGet(s_commSocket,
                                                       au8CommBuf,
                                                       (uint16_t)sizeof(au8CommBuf));
                if (i16Bytes > 0)
                {
                    SYS_CONSOLE_PRINT("[COMM] Received %d bytes\r\n",
                                      (int)i16Bytes);
                    prv_ParseCommissioningPacket(au8CommBuf,
                                                 (uint16_t)i16Bytes);
                }

                /* Handle client disconnection */
                if ((!TCPIP_TCP_IsConnected(s_commSocket)) ||
                    TCPIP_TCP_WasDisconnected(s_commSocket))
                {
                    SYS_CONSOLE_PRINT("[COMM] Client disconnected\r\n");
                    TCPIP_TCP_Close(s_commSocket);
                    s_commSocket = INVALID_SOCKET;
                }
            }

            vTaskDelay(pdMS_TO_TICKS(IO_TASK_LOOP_DELAY_MS));
            continue; /* Skip normal-mode socket handling below */
        }

        if (TCPIP_TCP_IsConnected(s_ioSocket))
        {
            uint8_t au8IOBuffer[256] = {0U};
            int16_t i16BytesRead = TCPIP_TCP_ArrayGet(s_ioSocket,
                                                      au8IOBuffer,
                                                      sizeof(au8IOBuffer));
            if (i16BytesRead > 0)
            {
                // SYS_CONSOLE_PRINT("[IO] Received %d bytes\r\n", (int)i16BytesRead);
                prv_ParseAndProcessInbPacket(au8IOBuffer, (uint16_t)i16BytesRead);
            }

            /* Check for disconnection */
            if ((!TCPIP_TCP_IsConnected(s_ioSocket)) ||
                TCPIP_TCP_WasDisconnected(s_ioSocket))
            {
                SYS_CONSOLE_PRINT("[IO] Client disconnected\r\n");
                TCPIP_TCP_Close(s_ioSocket);
                s_ioSocket = INVALID_SOCKET;
            }
        }

        /* Reconnect if socket was lost */
        if (s_ioSocket == INVALID_SOCKET)
        {
            SYS_CONSOLE_PRINT("[IO] Attempting socket reconnect...\r\n");
            s_ioSocket = TCPIP_TCP_ServerOpen(IP_ADDRESS_TYPE_IPV4,
                                               IO_SERVER_PORT_HEV, 0U);
            if (s_ioSocket == INVALID_SOCKET)
            {
                SYS_CONSOLE_PRINT("[IO] Reconnect failed, retrying in %u ms\r\n",
                                  RECONNECT_DELAY_MS);
            }
            else
            {
                SYS_CONSOLE_PRINT("[IO] Reconnected\r\n");
            }
            /* BUG FIX: was vTaskDelay(RECONNECT_DELAY_MS) — ms not ticks */
            vTaskDelay(pdMS_TO_TICKS(RECONNECT_DELAY_MS));
        }

        /* BUG FIX: was vTaskDelay(10) — raw ticks, should be ms */
        vTaskDelay(pdMS_TO_TICKS(IO_TASK_LOOP_DELAY_MS));
    }
}

/* ============================================================================
 * Public Init Function
 * ========================================================================== */

/**
 * @brief  Create the IO handler FreeRTOS task.
 *
 *         BUG FIX: return type changed from void to bool so caller can detect
 *         task creation failure.
 *
 * @return true  on success
 * @return false if xTaskCreate failed
 */
bool vIOHandler(void)
{
    SYS_CONSOLE_PRINT("[IO] Reading flash config\r\n");
    prv_ReadOutputsFromFlash();
    if (xTaskCreate(prv_IOHandlerServerTask,
                    "IO_Handler",
                    IO_SERVER_HANDLER_STACK_DEPTH,
                    NULL,
                    IO_SERVER_HANDLER_TASK_PRIORITY,
                    NULL) != pdPASS)
    {
        SYS_CONSOLE_PRINT("[IO] Task creation failed\r\n");
        return false;
    }
    SYS_CONSOLE_PRINT("[IO] Task created\r\n");
    return true;
}

/* =========================
 * MAC BUILD
 * ========================= */
static void MAC_Build(uint8_t lastByte, uint8_t *mac)
{
    if (lastByte == 0x00 || lastByte == 0xFF)
    {
        lastByte = 0x01;
    }

    mac[0] = APP_MAC_BASE_BYTE0;
    mac[1] = APP_MAC_BASE_BYTE1;
    mac[2] = APP_MAC_BASE_BYTE2;
    mac[3] = APP_MAC_BASE_BYTE3;
    mac[4] = APP_MAC_BASE_BYTE4;
    mac[5] = lastByte;
}

/* =========================
 * INIT
 * ========================= */
bool MAC_Init(void)
{
    return MAC_ReadFromFlash();
}

/* =========================
 * READ FROM FLASH
 * ========================= */
bool MAC_ReadFromFlash(void)
{
    mac_flash_data_t data;
    bool blank = true;

    FCW_Read((uint32_t *)&data,
             sizeof(data),
             MAC_FLASH_ADDRESS);

    /* Fast validation */
    if (data.magic != MAC_FLASH_MAGIC)
    {
        MAC_Build(0x01, gDeviceMac);
        MAC_WriteToFlash(0x01);
        return false;
    }

    /* Check blank */
    uint8_t *p = (uint8_t *)&data;
    for (uint32_t i = 0; i < sizeof(data); i++)
    {
        if (p[i] != 0x00U && p[i] != 0xFFU)
        {
            blank = false;
            break;
        }
    }

    if (blank)
    {
        MAC_Build(0x01, gDeviceMac);
        MAC_WriteToFlash(0x01);
        return false;
    }

    MAC_Build(data.mac_last_byte, gDeviceMac);
    return true;
}

/* =========================
 * WRITE TO FLASH
 * ========================= */
bool MAC_WriteToFlash(uint8_t lastByte)
{
    mac_flash_data_t data;
    uint32_t timeout;

    data.magic = MAC_FLASH_MAGIC;
    data.mac_last_byte = lastByte;
    memset(data.reserved, 0xFF, sizeof(data.reserved));

    DCACHE_CLEAN_BY_ADDR((uint32_t *)&data, sizeof(data));

    FCW_PageErase(MAC_FLASH_ADDRESS);

    timeout = FCW_TIMEOUT_CYCLES;
    while (FCW_IsBusy() && (timeout-- > 0U));

    if (timeout == 0U)
        return false;

    FCW_RowWrite((uint32_t *)&data, MAC_FLASH_ADDRESS);

    timeout = FCW_TIMEOUT_CYCLES;
    while (FCW_IsBusy() && (timeout-- > 0U));

    return (timeout != 0U);
}

/* =========================
 * GET MAC
 * ========================= */
const uint8_t* MAC_Get(void)
{
    return gDeviceMac;
}

/* =========================
 * STRING CONVERT
 * ========================= */
void MAC_ToString(const uint8_t *mac, char *str)
{
    sprintf(str,
        "%02X:%02X:%02X:%02X:%02X:%02X",
        mac[0], mac[1], mac[2],
        mac[3], mac[4], mac[5]);
}

/* =========================
 * PRINT
 * ========================= */
void MAC_Print(const uint8_t *mac)
{
    SYS_CONSOLE_PRINT(
        "MAC: %02X:%02X:%02X:%02X:%02X:%02X\r\n",
        mac[0], mac[1], mac[2],
        mac[3], mac[4], mac[5]);
}
/* ************************************************************************** */
/* End of File                                                                */
/* ************************************************************************** */