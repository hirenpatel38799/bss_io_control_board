/*******************************************************************************
 * File Name   : AppCanHandler.h
 * Company     : Bacancy
 * Summary     : CAN communication handler — header file
 *
 * Description :
 *   Provides function prototypes, macros, and type definitions for managing
 *   up to CANBUS_MAX (6) CAN channels over FreeRTOS SW RX/TX queues.
 *
 *   The number of channels actually initialised at runtime is determined by
 *   SESSION_GetMaxDocks(), clamped to CANBUS_MAX. This replaces the previous
 *   hardcoded 3-channel design.
 *
 *   Architecture per active channel:
 *     RX Drain Task  → polls hardware FIFO every CAN_RX_POLL_DELAY_MS
 *                    → reads available frames into SW RX queue
 *     Server Task    → drains entire SW RX queue each cycle → dispatches
 *                    → drains entire SW TX queue each cycle → transmits
 *
 *   Dock → CAN channel mapping (direct: dock N-1 → CAN N-1):
 *     DOCK_1 → CANBUS_0
 *     DOCK_2 → CANBUS_1
 *     DOCK_3 → CANBUS_2
 *     DOCK_4 → CANBUS_3
 *     DOCK_5 → CANBUS_4
 *     DOCK_6 → CANBUS_5
 *
 *   Future expansion beyond CANBUS_MAX:
 *     1. Add new entries to CANBus_e and increment CANBUS_MAX.
 *     2. Add a CAN_HwOps_t entry in s_canHwOps[] in AppCanHandler.c.
 *     3. Add a message RAM section and CACHE_ALIGN buffer in AppCanHandler.c.
 *     4. Ensure the HAL provides CANx_MessageRAMConfigSet / InterruptGet /
 *        InterruptClear / ErrorGet / RxFifoFillLevelGet / MessageReceiveFifo /
 *        MessageTransmitFifo for the new peripheral index.
 *     No other code needs to change.
 *
 * Version     : 4.0
 *
 * Changes from v3.0:
 *   - REMOVED: hardcoded 3-channel initialisation.
 *   - ADDED:   CAN_HwOps_t — function-pointer table for all HAL operations
 *              per channel, populated once in a static array s_canHwOps[].
 *              The generic task bodies index this table by channel number,
 *              eliminating all switch-case chains over channel index.
 *   - ADDED:   CAN_ActiveChannels() — runtime query of how many channels
 *              were successfully initialised.
 *   - ADDED:   CAN_GetChannelCount() — returns the active channel count
 *              clamped by SESSION_GetMaxDocks() and CANBUS_MAX.
 *   - ADDED:   CAN_GetStats() — safe accessor for per-channel drop counters.
 *   - CHANGED: vCanHandlerInit() now iterates 0..u8ActiveChannels-1 rather
 *              than hardcoding 3 iterations.
 *   - CHANGED: vSendCanTxMsgToQueue() now uses arithmetic mapping
 *              (dock - DOCK_1) → channel instead of a switch-case.
 *   - CHANGED: Per-channel wrapper tasks (vCan0..2RxHandlerTask etc.) are
 *              replaced by a single generic launcher that receives its
 *              channel index as a task parameter — no per-channel wrappers
 *              needed. The public prototypes are kept for backward compatibility
 *              but now forward to the generic launcher.
 *   - KEPT:    All bug fixes from v3.0 (BUG-1 through BUG-6) are preserved.
 *******************************************************************************/

#ifndef APP_CAN_HANDLER_H
#define APP_CAN_HANDLER_H

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 * Includes
 * ========================================================================== */
#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "IOHandler.h"          /* For Dock_e (DOCK_1 … DOCK_6) */
#include "sessionDBHandler.h"   /* For SESSION_GetMaxDocks()     */

/* ============================================================================
 * Maximum Hardware CAN Channels
 *
 * CANBUS_MAX is the absolute ceiling enforced by hardware and enum definition.
 * The runtime count may be lower (SESSION_GetMaxDocks()).
 * To add a 7th channel: add CANBUS_6 to CANBus_e and the HAL entry below.
 * ========================================================================== */
#define CAN_MAX_HW_CHANNELS         ((uint8_t)CANBUS_MAX)  /**< = 6, from CANBus_e */

/* ============================================================================
 * CAN Frame Field Definitions
 * ========================================================================== */
#define CAN_EXT_ID_MASK             (0x1FFFFFFFUL) /**< 29-bit extended ID mask   */
#define CAN_STD_ID_MASK             (0x7FFUL)      /**< 11-bit standard ID mask   */
#define CAN_MAX_DLC                 (8U)           /**< Max data length code      */
#define CAN_ID_BYTE_SIZE            (4U)           /**< ID bytes in raw buffer    */
#define CAN_PAYLOAD_BYTE_SIZE       (8U)           /**< Payload bytes             */
#define CAN_RAW_FRAME_SIZE          (CAN_ID_BYTE_SIZE + CAN_PAYLOAD_BYTE_SIZE)

/* ============================================================================
 * Frame Type Identifiers
 * ========================================================================== */
#define CAN_FRAME_EXTENDED          (1U)
#define CAN_FRAME_STANDARD          (0U)

/* ============================================================================
 * CAN ID Encoding Macros
 * ========================================================================== */
/** @brief Encode a standard 11-bit ID for the TX hardware FIFO */
#define CAN_WRITE_ID(id)            ((uint32_t)(id) << 18U)

/** @brief Decode a standard 11-bit ID from the RX hardware FIFO */
#define CAN_READ_ID(id)             ((uint32_t)(id) >> 18U)

/* ============================================================================
 * FreeRTOS Queue / Task Configuration
 * ========================================================================== */
#define CAN_QUEUE_SIZE              (50U)               /**< SW queue depth        */
#define CAN_RX_HANDLER_STACK_DEPTH  (512U)              /**< RX drain task stack   */
#define CAN_SERVER_HANDLER_STACK_DEPTH (512U)           /**< Server task stack     */
#define CAN_RX_HANDLER_TASK_PRIORITY    (tskIDLE_PRIORITY + 4U)
#define CAN_SERVER_HANDLER_TASK_PRIORITY (tskIDLE_PRIORITY + 3U)

/* ============================================================================
 * Hardware Operations Table
 *
 * CAN_HwOps_t holds function pointers for all HAL operations needed per
 * channel. One instance is stored per supported channel in the private
 * s_canHwOps[] table in AppCanHandler.c.
 *
 * Adding a new channel:
 *   1. Add an entry in s_canHwOps[] pointing to the new peripheral's HAL.
 *   2. No changes needed anywhere else.
 *
 * Why function pointers instead of a switch-case?
 *   A switch-case over channel index must be updated in every function that
 *   touches hardware (RX drain task, server task, CAN_Write, CAN_Write_Struct).
 *   A function-pointer table requires one edit in one place.
 * ========================================================================== */
typedef bool (*CAN_MsgRamConfigFn)(uint8_t *pRam);
typedef bool (*CAN_InterruptGetFn)(uint32_t mask);
typedef void (*CAN_InterruptClrFn)(uint32_t mask);
typedef uint32_t (*CAN_ErrorGetFn)(void);
typedef uint8_t  (*CAN_FifoFillFn)(CAN_RX_FIFO_NUM fifoNum);
typedef bool (*CAN_RxFifoFn)(CAN_RX_FIFO_NUM fifoNum, uint8_t count,
                              CAN_RX_BUFFER *pBuf);
typedef bool (*CAN_TxFifoFn)(uint8_t count, const CAN_TX_BUFFER *pBuf);

/**
 * @brief  Per-channel hardware operations table.
 *
 *         Populated once at compile time in s_canHwOps[]. Indexed by
 *         CANBus_e channel number.
 */
typedef struct
{
    const char         *pcName;         /**< "CAN0" … "CAN5" for log messages  */
    uint8_t            *pu8MessageRAM;  /**< Pointer to channel's message RAM  */
    uint32_t            u32RamSize;     /**< sizeof(CanNMessageRAM)            */
    CAN_MsgRamConfigFn  pfRamConfig;    /**< CANx_MessageRAMConfigSet          */
    CAN_InterruptGetFn  pfIntGet;       /**< CANx_InterruptGet                 */
    CAN_InterruptClrFn  pfIntClear;     /**< CANx_InterruptClear               */
    CAN_ErrorGetFn      pfErrorGet;     /**< CANx_ErrorGet                     */
    CAN_FifoFillFn      pfFifoFill;     /**< CANx_RxFifoFillLevelGet           */
    CAN_RxFifoFn        pfRxFifo;       /**< CANx_MessageReceiveFifo           */
    CAN_TxFifoFn        pfTxFifo;       /**< CANx_MessageTransmitFifo          */
} CAN_HwOps_t;

/* ============================================================================
 * Diagnostic Counter Type
 * ========================================================================== */
/**
 * @brief  Per-channel drop counters, incremented on every SW-queue-full event.
 */
typedef struct
{
    uint32_t u32RxDrops;   /**< Frames dropped: SW RX queue full  */
    uint32_t u32TxDrops;   /**< Frames dropped: SW TX queue full  */
} CAN_ChannelStats_t;

/* ============================================================================
 * Public Data
 * ========================================================================== */
extern QueueHandle_t       xCANRXQueueHandler[CAN_MAX_HW_CHANNELS];
extern QueueHandle_t       xCANTXQueueHandler[CAN_MAX_HW_CHANNELS];
extern CAN_ChannelStats_t  xCANStats[CAN_MAX_HW_CHANNELS];

/* ============================================================================
 * Function Prototypes — Initialisation
 * ========================================================================== */

/**
 * @brief  Initialise CAN message RAM, SW queues, and FreeRTOS tasks for the
 *         number of docks reported by SESSION_GetMaxDocks(), clamped to
 *         CAN_MAX_HW_CHANNELS.
 *
 *         Sequence:
 *           1. Clamp activeChannels = min(SESSION_GetMaxDocks(), CANBUS_MAX).
 *           2. Configure hardware message RAM for each active channel.
 *           3. Create SW RX + TX queues for each active channel.
 *           4. If any queue fails, abort — do not start tasks.
 *           5. Create one RX drain task and one server task per active channel.
 *
 * @return true   All resources for active channels created successfully.
 * @return false  One or more failures — see console for detail.
 */
bool vCanHandlerInit(void);

/* ============================================================================
 * Function Prototypes — Runtime Query
 * ========================================================================== */

/**
 * @brief  Return the number of CAN channels that were successfully initialised.
 *         Valid only after vCanHandlerInit() returns.
 * @return Active channel count (0 if not yet initialised).
 */
uint8_t CAN_GetActiveChannels(void);

/**
 * @brief  Return the channel count that vCanHandlerInit() will target.
 *         = min(SESSION_GetMaxDocks(), CAN_MAX_HW_CHANNELS).
 *         May be called before vCanHandlerInit() to validate configuration.
 * @return Clamped channel count.
 */
uint8_t CAN_GetChannelCount(void);

/**
 * @brief  Safe read of per-channel diagnostic counters.
 * @param  u8Channel  Channel index (0 to CAN_GetActiveChannels()-1).
 * @param  pStats     Output buffer; must not be NULL.
 * @return true on success, false if channel out of range or pStats NULL.
 */
bool CAN_GetStats(uint8_t u8Channel, CAN_ChannelStats_t *pStats);

/* ============================================================================
 * Function Prototypes — Utility
 * ========================================================================== */

/**
 * @brief  Decode the effective CAN ID from a received frame, correctly
 *         handling standard (11-bit) and extended (29-bit) formats.
 *
 *         Always use this instead of accessing pRxBuf->id directly.
 *
 * @param  pRxBuf  Pointer to received CAN frame (must not be NULL).
 * @return Decoded CAN ID.
 */
uint32_t CAN_GetFrameId(const CAN_RX_BUFFER *pRxBuf);

/* ============================================================================
 * Function Prototypes — CAN Write API
 * ========================================================================== */

/**
 * @brief  Transmit a raw byte-array CAN frame on the specified channel.
 *
 *         Buffer layout (min CAN_RAW_FRAME_SIZE bytes):
 *           [0..3]  CAN ID big-endian.  <= CAN_STD_ID_MASK → standard frame.
 *           [4..11] 8-byte payload.
 *
 * @param  u8CanIndex  Channel (0 to CAN_GetActiveChannels()-1).
 * @param  pu8Data     Raw frame buffer.
 * @param  u8Len       Buffer length (>= CAN_RAW_FRAME_SIZE).
 * @return true on success.
 */
bool CAN_Write(uint8_t u8CanIndex, const uint8_t *pu8Data, uint8_t u8Len);

/**
 * @brief  Transmit a structured TX buffer on the specified channel (direct,
 *         bypasses the SW TX queue).
 *
 *         Prefer vSendCanTxMsgToQueue() from application code to avoid
 *         blocking on the hardware FIFO.
 *
 * @param  u8CanIndex  Channel (0 to CAN_GetActiveChannels()-1).
 * @param  pTxBuffer   Populated TX buffer (must not be NULL, DLC <= 8).
 * @return true on success.
 */
bool CAN_WriteStruct(uint8_t u8CanIndex, const CAN_TX_BUFFER *const pTxBuffer);

/**
 * @brief  Enqueue a TX frame for deferred transmission on a dock's CAN channel.
 *
 *         Non-blocking. Frame is dropped (and drop counter incremented) if the
 *         SW TX queue is full.
 *
 *         Dock-to-channel mapping: channel = (u8DockNo - DOCK_1)
 *         Valid for DOCK_1 … DOCK_6, provided the channel was initialised.
 *
 * @param  pCanTxBuffer  Frame to enqueue (must not be NULL).
 * @param  u8DockNo      Dock identifier (DOCK_1 … DOCK_6).
 */
void vSendCanTxMsgToQueue(const CAN_TX_BUFFER *const pCanTxBuffer,
                          uint8_t u8DockNo);

/* ============================================================================
 * Backward-Compatible Per-Channel Task Prototypes
 *
 * These are kept so existing xTaskCreate() call sites compile without change.
 * Internally they all forward to the single generic launcher.
 * New code should call vCanHandlerInit() instead of creating tasks manually.
 * ========================================================================== */
void vCan0RxHandlerTask(void *pvParameters);
void vCan1RxHandlerTask(void *pvParameters);
void vCan2RxHandlerTask(void *pvParameters);
void vCan3RxHandlerTask(void *pvParameters);
void vCan4RxHandlerTask(void *pvParameters);
void vCan5RxHandlerTask(void *pvParameters);

void vCan0HandlerServerTask(void *pvParameters);
void vCan1HandlerServerTask(void *pvParameters);
void vCan2HandlerServerTask(void *pvParameters);
void vCan3HandlerServerTask(void *pvParameters);
void vCan4HandlerServerTask(void *pvParameters);
void vCan5HandlerServerTask(void *pvParameters);

#ifdef __cplusplus
}
#endif

#endif /* APP_CAN_HANDLER_H */

/*******************************************************************************
 * End of File
 *******************************************************************************/