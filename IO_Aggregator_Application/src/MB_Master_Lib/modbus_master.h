/*******************************************************************************
 * @file    modbus_master.h
 * @brief   Modbus RTU Master Stack - Public API Header
 * @details Provides the complete Modbus RTU Master API:
 *            - Initialisation
 *            - Blocking request functions (ReadHoldingRegisters,
 *              WriteSingleRegister, WriteMultipleRegisters)
 *            - Per-channel request/response state machine
 *            - Automatic retry with configurable count
 *            - Timeout management
 *            - Error counters and diagnostics
 *            - Result callback/event notification
 *
 *          Intended use pattern:
 *          1. Call Modbus_Master_Init() once per channel at startup.
 *          2. From your application task, call Modbus_Master_Process()
 *             periodically (e.g. every 5 ms) to drive the state machine.
 *          3. Issue requests via the Modbus_ReadHoldingRegisters(),
 *             Modbus_WriteSingleRegister(), Modbus_WriteMultipleRegisters() APIs.
 *          4. Register a result callback via Modbus_Master_RegisterCallback()
 *             to receive asynchronous completion events.
 *
 * @company BACANCY SYSTEMS PVT. LTD.
 *******************************************************************************/

#ifndef MODBUS_MASTER_H
#define MODBUS_MASTER_H

/* =========================================================================
 * Included Files
 * ========================================================================= */
#include <stdint.h>
#include <stdbool.h>
#include "modbus_rtu.h"
#include "uart_port.h"

/* =========================================================================
 * Configuration — Tune to application requirements
 * ========================================================================= */

/** Maximum number of retry attempts before reporting failure */
#define MODBUS_MASTER_MAX_RETRIES       (3U)

/** Response timeout in milliseconds (master waits this long for a reply) */
#define MODBUS_MASTER_RESPONSE_TIMEOUT_MS  (200U)

/** Inter-request idle gap in milliseconds (silent time between transactions) */
#define MODBUS_MASTER_INTER_REQUEST_GAP_MS (10U)

/* =========================================================================
 * Type Definitions
 * ========================================================================= */

/**
 * @brief  Modbus master state machine states.
 */
typedef enum
{
    MB_MASTER_STATE_IDLE = 0U,   /**< No transaction in progress         */
    MB_MASTER_STATE_SENDING,     /**< Transmitting request frame         */
    MB_MASTER_STATE_WAITING,     /**< Waiting for slave response         */
    MB_MASTER_STATE_PROCESSING,  /**< Parsing & validating response      */
    MB_MASTER_STATE_COMPLETE,    /**< Transaction finished successfully  */
    MB_MASTER_STATE_ERROR        /**< Transaction failed (after retries) */
} ModbusMaster_State_t;

/**
 * @brief  Transaction result / error codes.
 */
typedef enum
{
    MB_RESULT_SUCCESS            = 0U,  /**< Transaction completed OK          */
    MB_RESULT_ERR_TIMEOUT        = 1U,  /**< No response within timeout period */
    MB_RESULT_ERR_CRC            = 2U,  /**< Response CRC mismatch             */
    MB_RESULT_ERR_SLAVE_ADDR     = 3U,  /**< Response from wrong slave         */
    MB_RESULT_ERR_FUNCTION_CODE  = 4U,  /**< Unexpected function code          */
    MB_RESULT_ERR_EXCEPTION      = 5U,  /**< Slave returned exception response */
    MB_RESULT_ERR_FRAME_LENGTH   = 6U,  /**< Response length invalid           */
    MB_RESULT_ERR_OVERFLOW       = 7U,  /**< RX buffer overflow                */
    MB_RESULT_ERR_HW             = 8U,  /**< UART hardware error               */
    MB_RESULT_ERR_BUSY           = 9U,  /**< Master busy (previous request ongoing) */
    MB_RESULT_ERR_PARAM          = 10U  /**< Invalid parameter to API          */
} ModbusMaster_Result_t;

/**
 * @brief  Aggregated error/diagnostic counters (per channel).
 */
typedef struct
{
    uint32_t u32TxRequests;       /**< Total requests transmitted        */
    uint32_t u32RxSuccess;        /**< Successful transactions           */
    uint32_t u32ErrTimeout;       /**< Timeout failures                  */
    uint32_t u32ErrCRC;           /**< CRC errors                        */
    uint32_t u32ErrException;     /**< Slave exception responses         */
    uint32_t u32ErrOverflow;      /**< RX buffer overflows               */
    uint32_t u32ErrHW;            /**< Hardware write failures           */
    uint32_t u32Retries;          /**< Total retry attempts              */
} ModbusMaster_Diagnostics_t;

/**
 * @brief  Transaction completion callback prototype.
 *
 * @details Invoked by Modbus_Master_Process() when a transaction concludes
 *          (success or failure).  Runs in the caller's task context.
 *
 * @param[in]  eChannel   Channel the transaction was on.
 * @param[in]  eResult    Final transaction result.
 * @param[in]  pxResp     Pointer to parsed response (NULL on failure).
 * @param[in]  pvContext  User-supplied context pointer registered with the callback.
 */
typedef void (*ModbusMaster_ResultCallback_t)(UartPort_Channel_t        eChannel,
                                               ModbusMaster_Result_t     eResult,
                                               const ModbusRtu_Response_t *pxResp,
                                               void                     *pvContext);

/**
 * @brief  Pending transaction descriptor (internal, exposed for sizing only).
 */
typedef struct
{
    ModbusRtu_Request_t    xRequest;         /**< Serialised request frame           */
    ModbusFunctionCode_t   eFC;              /**< Expected function code             */
    uint8_t                u8SlaveAddr;      /**< Target slave address               */
    uint8_t                u8RegCount;       /**< Register / coil count in request   */
    uint8_t                u8RetryCount;     /**< Retry attempts so far              */
    uint32_t               u32TimeoutStart;  /**< Tick count when wait began (ms)    */
    bool                   bActive;          /**< Transaction slot is occupied       */
} ModbusMaster_Transaction_t;

/**
 * @brief  Master channel context.
 *         One instance per UartPort_Channel_t.
 */
typedef struct
{
    UartPort_Channel_t          eChannel;     /**< Hardware channel                 */
    ModbusMaster_State_t        eState;       /**< Current FSM state                */
    ModbusMaster_Transaction_t  xTx;         /**< Active transaction                */
    ModbusRtu_RxCtx_t           xRxCtx;      /**< RX accumulator                   */
    ModbusMaster_Diagnostics_t  xDiag;       /**< Error/diagnostic counters         */
    ModbusMaster_ResultCallback_t pfnCallback; /**< Completion callback             */
    void                       *pvCallbackCtx; /**< User context for callback       */
} ModbusMaster_Channel_t;

/* =========================================================================
 * Public API — Lifecycle
 * ========================================================================= */

/**
 * @brief  Initialise the Modbus Master stack for one channel.
 *
 * @details Initialises the master context, calls UartPort_Init() for the
 *          channel, and registers internal ISR callbacks.
 *          Must be called once per channel before any other master API.
 *
 * @param[in,out]  pxMaster   Master context to initialise (caller-allocated).
 * @param[in]      eChannel   Hardware channel to bind to.
 *
 * @return  MB_RESULT_SUCCESS or MB_RESULT_ERR_PARAM.
 */
ModbusMaster_Result_t Modbus_Master_Init(ModbusMaster_Channel_t *pxMaster,
                                          UartPort_Channel_t      eChannel);

/**
 * @brief  Register a result callback for a channel.
 *
 * @param[in,out]  pxMaster    Master context.
 * @param[in]      pfnCallback Callback function (may be NULL to deregister).
 * @param[in]      pvContext   Caller context pointer passed back in callback.
 */
void Modbus_Master_RegisterCallback(ModbusMaster_Channel_t      *pxMaster,
                                     ModbusMaster_ResultCallback_t pfnCallback,
                                     void                         *pvContext);

/**
 * @brief  Drive the master state machine — call from your application task.
 *
 * @details Must be called repeatedly at a rate of at least every
 *          MODBUS_MASTER_RESPONSE_TIMEOUT_MS / 4 milliseconds to ensure
 *          accurate timeout detection.  Safe to call at any rate.
 *
 * @param[in,out]  pxMaster  Master context to process.
 */
void Modbus_Master_Process(ModbusMaster_Channel_t *pxMaster);

/**
 * @brief  Retrieve a copy of the diagnostic counters for a channel.
 *
 * @param[in]  pxMaster   Master context.
 * @param[out] pxDiag     Destination structure for the counter copy.
 */
void Modbus_Master_GetDiagnostics(const ModbusMaster_Channel_t *pxMaster,
                                   ModbusMaster_Diagnostics_t   *pxDiag);

/**
 * @brief  Reset all diagnostic counters to zero.
 *
 * @param[in,out]  pxMaster  Master context.
 */
void Modbus_Master_ResetDiagnostics(ModbusMaster_Channel_t *pxMaster);

/* =========================================================================
 * Public API — Request Submission
 * ========================================================================= */

/**
 * @brief  Submit a Read Holding Registers (FC03) request.
 *
 * @param[in,out]  pxMaster     Master channel context.
 * @param[in]      u8SlaveAddr  Target slave address (1–247).
 * @param[in]      u16RegAddr   Starting register address.
 * @param[in]      u16RegCount  Number of registers to read (1–125).
 *
 * @return  MB_RESULT_SUCCESS   Request queued.
 *          MB_RESULT_ERR_BUSY  Previous transaction still in progress.
 *          MB_RESULT_ERR_PARAM Invalid parameter.
 */
ModbusMaster_Result_t Modbus_ReadHoldingRegisters(ModbusMaster_Channel_t *pxMaster,
                                                   uint8_t                 u8SlaveAddr,
                                                   uint16_t                u16RegAddr,
                                                   uint16_t                u16RegCount);

/**
 * @brief  Submit a Write Single Register (FC06) request.
 *
 * @param[in,out]  pxMaster     Master channel context.
 * @param[in]      u8SlaveAddr  Target slave address (1–247).
 * @param[in]      u16RegAddr   Target register address.
 * @param[in]      u16Value     16-bit value to write.
 *
 * @return  MB_RESULT_SUCCESS   Request queued.
 *          MB_RESULT_ERR_BUSY  Previous transaction still in progress.
 *          MB_RESULT_ERR_PARAM Invalid parameter.
 */
ModbusMaster_Result_t Modbus_WriteSingleRegister(ModbusMaster_Channel_t *pxMaster,
                                                  uint8_t                 u8SlaveAddr,
                                                  uint16_t                u16RegAddr,
                                                  uint16_t                u16Value);

/**
 * @brief  Submit a Write Multiple Registers (FC16) request.
 *
 * @param[in,out]  pxMaster     Master channel context.
 * @param[in]      u8SlaveAddr  Target slave address (1–247).
 * @param[in]      u16RegAddr   Starting register address.
 * @param[in]      pu16Values   Array of register values to write.
 * @param[in]      u8RegCount   Number of registers (1–123).
 *
 * @return  MB_RESULT_SUCCESS   Request queued.
 *          MB_RESULT_ERR_BUSY  Previous transaction still in progress.
 *          MB_RESULT_ERR_PARAM Invalid parameter.
 */
ModbusMaster_Result_t Modbus_WriteMultipleRegisters(ModbusMaster_Channel_t *pxMaster,
                                                     uint8_t                 u8SlaveAddr,
                                                     uint16_t                u16RegAddr,
                                                     const uint16_t         *pu16Values,
                                                     uint8_t                 u8RegCount);

/**
 * @brief  Query whether the master is currently idle (ready to accept a new request).
 *
 * @param[in]  pxMaster  Master channel context.
 *
 * @return  true  Idle — safe to submit a new request.
 *          false Busy — a transaction is in progress.
 */
bool Modbus_Master_IsIdle(const ModbusMaster_Channel_t *pxMaster);

#endif /* MODBUS_MASTER_H */

/*******************************************************************************
 * End of File
 *******************************************************************************/
