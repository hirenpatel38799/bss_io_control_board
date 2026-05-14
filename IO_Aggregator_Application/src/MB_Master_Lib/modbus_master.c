/*******************************************************************************
 * @file    modbus_master.c
 * @brief   Modbus RTU Master Stack - State Machine Implementation
 * @details Implements the complete Modbus RTU Master state machine:
 *
 *   IDLE ──[request submitted]──► SENDING
 *    ▲                                │
 *    │                           [UartPort_Transmit OK]
 *    │                                ▼
 *    │                           WAITING ──[timeout + retries exhausted]──► ERROR
 *    │                                │                       ▲
 *    │                           [frame ready]           [parse fail / retry]
 *    │                                ▼                       │
 *    │                           PROCESSING ─────────────────┘
 *    │                                │
 *    │                           [parse OK]
 *    │                                ▼
 *    └──────────────────────────── COMPLETE
 *
 * @company BACANCY SYSTEMS PVT. LTD.
 *******************************************************************************/

/* =========================================================================
 * Included Files
 * ========================================================================= */
#include "modbus_master.h"
#include "modbus_rtu.h"
#include "modbus_crc.h"
#include "uart_port.h"
#include "FreeRTOS.h"
#include "task.h"
#include "definitions.h"    /* SYS_CONSOLE_PRINT */
#include <string.h>

/* Internal offset alias used in PROCESSING state for clarity */
#define MB_FRAME_OFFSET_SLAVE    (0U)

/* =========================================================================
 * Private Helper: millisecond tick source
 * ========================================================================= */

/**
 * @brief  Return elapsed milliseconds since boot via FreeRTOS tick counter.
 *         Assumes configTICK_RATE_HZ == 1000; adjust if needed.
 */
static inline uint32_t prv_GetTickMs(void)
{
    return (uint32_t)pdTICKS_TO_MS(xTaskGetTickCount());
}

/**
 * @brief  Check if u32Duration ms has elapsed since u32Start.
 */
static inline bool prv_ElapsedMs(uint32_t u32Start, uint32_t u32Duration)
{
    return ((prv_GetTickMs() - u32Start) >= u32Duration);
}

/* =========================================================================
 * Private: ISR callbacks registered with UartPort
 * =========================================================================
 * Each channel gets its own callback pair.  The master context pointer is
 * stored in a small static table indexed by channel so the ISR can reach it.
 * ========================================================================= */

static ModbusMaster_Channel_t *s_apxMasterCtx[UART_PORT_CHANNEL_COUNT];

static void prv_RxByteCallback(UartPort_Channel_t eChannel, uint8_t u8Byte)
{
    ModbusMaster_Channel_t *pxMaster;

    if (eChannel >= UART_PORT_CH_MAX)
    {
        return;
    }

    pxMaster = s_apxMasterCtx[eChannel];

    if (pxMaster != NULL)
    {
        ModbusRtu_RxFeedByte(&pxMaster->xRxCtx, u8Byte);
    }
}

static void prv_FrameEndCallback(UartPort_Channel_t eChannel)
{
    ModbusMaster_Channel_t *pxMaster;

    if (eChannel >= UART_PORT_CH_MAX)
    {
        return;
    }

    pxMaster = s_apxMasterCtx[eChannel];

    if (pxMaster != NULL)
    {
        ModbusRtu_RxFrameEnd(&pxMaster->xRxCtx);
    }
}

/* =========================================================================
 * Private: transaction helpers
 * ========================================================================= */

/**
 * @brief  Conclude a transaction — invoke callback and return to IDLE.
 */
static void prv_CompleteTransaction(ModbusMaster_Channel_t     *pxMaster,
                                     ModbusMaster_Result_t       eResult,
                                     const ModbusRtu_Response_t *pxResp)
{
    /* Update diagnostics */
    if (eResult == MB_RESULT_SUCCESS)
    {
        pxMaster->xDiag.u32RxSuccess++;
    }

    /* Reset RX accumulator for next transaction */
    ModbusRtu_RxCtx_Reset(&pxMaster->xRxCtx);

    /* Clear active transaction flag */
    pxMaster->xTx.bActive = false;

    /* Invoke user callback if registered */
    if (pxMaster->pfnCallback != NULL)
    {
        pxMaster->pfnCallback(pxMaster->eChannel,
                               eResult,
                               pxResp,
                               pxMaster->pvCallbackCtx);
    }

    /* Return to IDLE */
    pxMaster->eState = MB_MASTER_STATE_IDLE;

    SYS_CONSOLE_PRINT("[ModbusMaster CH%u] Transaction done, result=%u\r\n",
                       (unsigned)pxMaster->eChannel, (unsigned)eResult);
}

/**
 * @brief  Retry or fail a transaction.
 *
 * @return  true  Retry was initiated (back to SENDING).
 *          false No retries left — error reported.
 */
static bool prv_RetryOrFail(ModbusMaster_Channel_t *pxMaster,
                              ModbusMaster_Result_t   eErrorResult)
{
    pxMaster->xDiag.u32Retries++;

    if (pxMaster->xTx.u8RetryCount < (uint8_t)MODBUS_MASTER_MAX_RETRIES)
    {
        pxMaster->xTx.u8RetryCount++;

        SYS_CONSOLE_PRINT("[ModbusMaster CH%u] Retry %u/%u\r\n",
                           (unsigned)pxMaster->eChannel,
                           (unsigned)pxMaster->xTx.u8RetryCount,
                           (unsigned)MODBUS_MASTER_MAX_RETRIES);

        /* Reset RX context and go back to SENDING */
        ModbusRtu_RxCtx_Reset(&pxMaster->xRxCtx);
        pxMaster->eState = MB_MASTER_STATE_SENDING;
        return true;
    }

    /* Retries exhausted */
    pxMaster->eState = MB_MASTER_STATE_ERROR;

    /* Update specific error counter */
    switch (eErrorResult)
    {
        case MB_RESULT_ERR_TIMEOUT:   pxMaster->xDiag.u32ErrTimeout++;   break;
        case MB_RESULT_ERR_CRC:       pxMaster->xDiag.u32ErrCRC++;       break;
        case MB_RESULT_ERR_EXCEPTION: pxMaster->xDiag.u32ErrException++;  break;
        case MB_RESULT_ERR_OVERFLOW:  pxMaster->xDiag.u32ErrOverflow++;   break;
        case MB_RESULT_ERR_HW:        pxMaster->xDiag.u32ErrHW++;         break;
        default:                                                            break;
    }

    prv_CompleteTransaction(pxMaster, eErrorResult, NULL);
    return false;
}

/* =========================================================================
 * Public Function Implementations — Lifecycle
 * ========================================================================= */

ModbusMaster_Result_t Modbus_Master_Init(ModbusMaster_Channel_t *pxMaster,
                                          UartPort_Channel_t      eChannel)
{
    UartPort_Status_t ePortStatus;

    if ((pxMaster == NULL) || (eChannel >= UART_PORT_CH_MAX))
    {
        return MB_RESULT_ERR_PARAM;
    }

    /* Zero-init the entire context */
    (void)memset(pxMaster, 0x00, sizeof(ModbusMaster_Channel_t));

    pxMaster->eChannel = eChannel;
    pxMaster->eState   = MB_MASTER_STATE_IDLE;

    ModbusRtu_RxCtx_Reset(&pxMaster->xRxCtx);

    /* Register this context in the ISR dispatch table */
    s_apxMasterCtx[eChannel] = pxMaster;

    /* Initialise hardware port */
    ePortStatus = UartPort_Init(eChannel, prv_RxByteCallback, prv_FrameEndCallback);

    if (ePortStatus != UART_PORT_OK)
    {
        return MB_RESULT_ERR_HW;
    }

    SYS_CONSOLE_PRINT("[ModbusMaster CH%u] Initialised\r\n", (unsigned)eChannel);

    return MB_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */

void Modbus_Master_RegisterCallback(ModbusMaster_Channel_t      *pxMaster,
                                     ModbusMaster_ResultCallback_t pfnCallback,
                                     void                         *pvContext)
{
    if (pxMaster == NULL)
    {
        return;
    }

    pxMaster->pfnCallback    = pfnCallback;
    pxMaster->pvCallbackCtx  = pvContext;
}

/* ------------------------------------------------------------------------- */

void Modbus_Master_Process(ModbusMaster_Channel_t *pxMaster)
{
    UartPort_Status_t    ePortStatus;
    ModbusRtu_Response_t xResp;
    bool                 bParseOk;
    bool                 bFrameReady;

    if (pxMaster == NULL)
    {
        return;
    }

    switch (pxMaster->eState)
    {
        /* ----------------------------------------------------------------- */
        case MB_MASTER_STATE_IDLE:
            /* Nothing to do — waiting for a request to be submitted */
            break;

        /* ----------------------------------------------------------------- */
        case MB_MASTER_STATE_SENDING:
        {
            pxMaster->xDiag.u32TxRequests++;

            SYS_CONSOLE_PRINT("[ModbusMaster CH%u] TX %u bytes → Slave 0x%02X FC=0x%02X\r\n",
                               (unsigned)pxMaster->eChannel,
                               (unsigned)pxMaster->xTx.xRequest.u16Length,
                               (unsigned)pxMaster->xTx.u8SlaveAddr,
                               (unsigned)pxMaster->xTx.eFC);

            ePortStatus = UartPort_Transmit(pxMaster->eChannel,
                                             pxMaster->xTx.xRequest.au8Frame,
                                             pxMaster->xTx.xRequest.u16Length);

            if (ePortStatus != UART_PORT_OK)
            {
                pxMaster->xDiag.u32ErrHW++;
                (void)prv_RetryOrFail(pxMaster, MB_RESULT_ERR_HW);
                break;
            }

            /* Frame sent — start response timeout and enter WAITING */
            pxMaster->xTx.u32TimeoutStart = prv_GetTickMs();
            pxMaster->eState = MB_MASTER_STATE_WAITING;
            break;
        }

        /* ----------------------------------------------------------------- */
        case MB_MASTER_STATE_WAITING:
        {
            /* Check for received frame (set by ISR via RxFrameEnd) */
            taskENTER_CRITICAL();
            bFrameReady = (pxMaster->xRxCtx.eState == MODBUS_RTU_RX_FRAME_READY);
            taskEXIT_CRITICAL();

            if (bFrameReady)
            {
                pxMaster->eState = MB_MASTER_STATE_PROCESSING;
                break;     /* Fall through to PROCESSING on next call */
            }

            /* Check timeout */
            if (prv_ElapsedMs(pxMaster->xTx.u32TimeoutStart,
                               (uint32_t)MODBUS_MASTER_RESPONSE_TIMEOUT_MS))
            {
                SYS_CONSOLE_PRINT("[ModbusMaster CH%u] Timeout (Slave 0x%02X)\r\n",
                                   (unsigned)pxMaster->eChannel,
                                   (unsigned)pxMaster->xTx.u8SlaveAddr);

                (void)prv_RetryOrFail(pxMaster, MB_RESULT_ERR_TIMEOUT);
            }
            break;
        }

        /* ----------------------------------------------------------------- */
        case MB_MASTER_STATE_PROCESSING:
        {
            (void)memset(&xResp, 0x00, sizeof(xResp));

            bParseOk = ModbusRtu_ParseResponse(&pxMaster->xRxCtx,
                                                pxMaster->xTx.u8SlaveAddr,
                                                pxMaster->xTx.eFC,
                                                &xResp);

            if (!bParseOk)
            {
                /* Determine specific error */
                ModbusMaster_Result_t eErr;

                if (pxMaster->xRxCtx.bOverflow)
                {
                    eErr = MB_RESULT_ERR_OVERFLOW;
                }
                else if (!Modbus_CRC16_Validate(pxMaster->xRxCtx.au8Buffer,
                                                 pxMaster->xRxCtx.u16Count))
                {
                    eErr = MB_RESULT_ERR_CRC;
                }
                else if (pxMaster->xRxCtx.au8Buffer[MB_FRAME_OFFSET_SLAVE] !=
                         pxMaster->xTx.u8SlaveAddr)
                {
                    eErr = MB_RESULT_ERR_SLAVE_ADDR;
                }
                else
                {
                    eErr = MB_RESULT_ERR_FRAME_LENGTH;
                }

                SYS_CONSOLE_PRINT("[ModbusMaster CH%u] Parse error %u\r\n",
                                   (unsigned)pxMaster->eChannel, (unsigned)eErr);

                (void)prv_RetryOrFail(pxMaster, eErr);
                break;
            }

            /* Check for exception response */
            if (xResp.bIsException)
            {
                SYS_CONSOLE_PRINT("[ModbusMaster CH%u] Slave 0x%02X exception=0x%02X\r\n",
                                   (unsigned)pxMaster->eChannel,
                                   (unsigned)xResp.u8SlaveAddr,
                                   (unsigned)xResp.eExceptionCode);

                pxMaster->xDiag.u32ErrException++;
                prv_CompleteTransaction(pxMaster, MB_RESULT_ERR_EXCEPTION, &xResp);
                break;
            }

            /* All good */
            pxMaster->eState = MB_MASTER_STATE_COMPLETE;
            prv_CompleteTransaction(pxMaster, MB_RESULT_SUCCESS, &xResp);
            break;
        }

        /* ----------------------------------------------------------------- */
        case MB_MASTER_STATE_COMPLETE:
        case MB_MASTER_STATE_ERROR:
            /* Both states are handled inside prv_CompleteTransaction()
             * which transitions back to IDLE immediately.
             * We should never linger here. */
            pxMaster->eState = MB_MASTER_STATE_IDLE;
            break;

        default:
            pxMaster->eState = MB_MASTER_STATE_IDLE;
            break;
    }
}

/* =========================================================================
 * Public Function Implementations — Diagnostics
 * ========================================================================= */

void Modbus_Master_GetDiagnostics(const ModbusMaster_Channel_t *pxMaster,
                                   ModbusMaster_Diagnostics_t   *pxDiag)
{
    if ((pxMaster == NULL) || (pxDiag == NULL))
    {
        return;
    }

    (void)memcpy(pxDiag, &pxMaster->xDiag, sizeof(ModbusMaster_Diagnostics_t));
}

/* ------------------------------------------------------------------------- */

void Modbus_Master_ResetDiagnostics(ModbusMaster_Channel_t *pxMaster)
{
    if (pxMaster == NULL)
    {
        return;
    }

    (void)memset(&pxMaster->xDiag, 0x00, sizeof(ModbusMaster_Diagnostics_t));
}

/* =========================================================================
 * Public Function Implementations — Request Submission
 * ========================================================================= */

bool Modbus_Master_IsIdle(const ModbusMaster_Channel_t *pxMaster)
{
    if (pxMaster == NULL)
    {
        return false;
    }

    return (pxMaster->eState == MB_MASTER_STATE_IDLE);
}

/* ------------------------------------------------------------------------- */

ModbusMaster_Result_t Modbus_ReadHoldingRegisters(ModbusMaster_Channel_t *pxMaster,
                                                   uint8_t                 u8SlaveAddr,
                                                   uint16_t                u16RegAddr,
                                                   uint16_t                u16RegCount)
{
    bool bBuilt;

    if (pxMaster == NULL)
    {
        return MB_RESULT_ERR_PARAM;
    }

    if (!Modbus_Master_IsIdle(pxMaster))
    {
        return MB_RESULT_ERR_BUSY;
    }

    bBuilt = ModbusRtu_BuildReadHoldingRegisters(&pxMaster->xTx.xRequest,
                                                  u8SlaveAddr,
                                                  u16RegAddr,
                                                  u16RegCount);
    if (!bBuilt)
    {
        return MB_RESULT_ERR_PARAM;
    }

    pxMaster->xTx.u8SlaveAddr  = u8SlaveAddr;
    pxMaster->xTx.eFC          = MB_FC_READ_HOLDING_REGISTERS;
    pxMaster->xTx.u8RegCount   = (uint8_t)u16RegCount;
    pxMaster->xTx.u8RetryCount = 0U;
    pxMaster->xTx.bActive      = true;

    ModbusRtu_RxCtx_Reset(&pxMaster->xRxCtx);
    pxMaster->eState = MB_MASTER_STATE_SENDING;

    return MB_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */

ModbusMaster_Result_t Modbus_WriteSingleRegister(ModbusMaster_Channel_t *pxMaster,
                                                  uint8_t                 u8SlaveAddr,
                                                  uint16_t                u16RegAddr,
                                                  uint16_t                u16Value)
{
    bool bBuilt;

    if (pxMaster == NULL)
    {
        return MB_RESULT_ERR_PARAM;
    }

    if (!Modbus_Master_IsIdle(pxMaster))
    {
        return MB_RESULT_ERR_BUSY;
    }

    bBuilt = ModbusRtu_BuildWriteSingleRegister(&pxMaster->xTx.xRequest,
                                                 u8SlaveAddr,
                                                 u16RegAddr,
                                                 u16Value);
    if (!bBuilt)
    {
        return MB_RESULT_ERR_PARAM;
    }

    pxMaster->xTx.u8SlaveAddr  = u8SlaveAddr;
    pxMaster->xTx.eFC          = MB_FC_WRITE_SINGLE_REGISTER;
    pxMaster->xTx.u8RegCount   = 1U;
    pxMaster->xTx.u8RetryCount = 0U;
    pxMaster->xTx.bActive      = true;

    ModbusRtu_RxCtx_Reset(&pxMaster->xRxCtx);
    pxMaster->eState = MB_MASTER_STATE_SENDING;

    return MB_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */

ModbusMaster_Result_t Modbus_WriteMultipleRegisters(ModbusMaster_Channel_t *pxMaster,
                                                     uint8_t                 u8SlaveAddr,
                                                     uint16_t                u16RegAddr,
                                                     const uint16_t         *pu16Values,
                                                     uint8_t                 u8RegCount)
{
    bool bBuilt;

    if ((pxMaster == NULL) || (pu16Values == NULL))
    {
        return MB_RESULT_ERR_PARAM;
    }

    if (!Modbus_Master_IsIdle(pxMaster))
    {
        return MB_RESULT_ERR_BUSY;
    }

    bBuilt = ModbusRtu_BuildWriteMultipleRegisters(&pxMaster->xTx.xRequest,
                                                    u8SlaveAddr,
                                                    u16RegAddr,
                                                    pu16Values,
                                                    u8RegCount);
    if (!bBuilt)
    {
        return MB_RESULT_ERR_PARAM;
    }

    pxMaster->xTx.u8SlaveAddr  = u8SlaveAddr;
    pxMaster->xTx.eFC          = MB_FC_WRITE_MULTIPLE_REGISTERS;
    pxMaster->xTx.u8RegCount   = u8RegCount;
    pxMaster->xTx.u8RetryCount = 0U;
    pxMaster->xTx.bActive      = true;

    ModbusRtu_RxCtx_Reset(&pxMaster->xRxCtx);
    pxMaster->eState = MB_MASTER_STATE_SENDING;

    return MB_RESULT_SUCCESS;
}
/*******************************************************************************
 * End of File
 *******************************************************************************/
