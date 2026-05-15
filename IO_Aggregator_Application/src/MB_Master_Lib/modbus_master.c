/*******************************************************************************
 * @file    modbus_master.c
 * @brief   Modbus RTU Master Stack — State Machine Implementation
 *
 * Critical ordering fix
 * ──────────────────────────────────────────────────────────────────────────
 * OLD (broken): RxCtx_Reset() was called inside prv_CompleteTransaction()
 *               AFTER the callback, meaning the next request submission
 *               could TX before the context was fully clean.
 *
 * NEW (correct): RxCtx_Reset() is called at the top of the SENDING state,
 *               BEFORE UartPort_Transmit().  This guarantees the accumulator
 *               is empty when the slave's response starts arriving.
 *
 * Parser integration fix
 * ──────────────────────────────────────────────────────────────────────────
 * ModbusRtu_ParseResponse() now returns ModbusRtu_ParseResult_t.
 * The PROCESSING state maps each code to the correct diagnostic counter
 * and logs a human-readable string so you can see "ERR_LENGTH (frame split!)"
 * rather than the generic "Parse error 3".
 *
 * @company BACANCY SYSTEMS PVT. LTD.
 *******************************************************************************/

#include "modbus_master.h"
#include "modbus_rtu.h"
#include "modbus_crc.h"
#include "uart_port.h"
#include "FreeRTOS.h"
#include "task.h"
#include "definitions.h"
#include <string.h>

/* =========================================================================
 * Private helpers
 * ========================================================================= */
static inline uint32_t prv_GetTickMs(void)
{
    return (uint32_t)pdTICKS_TO_MS(xTaskGetTickCount());
}

static inline bool prv_ElapsedMs(uint32_t u32Start, uint32_t u32Duration)
{
    return ((prv_GetTickMs() - u32Start) >= u32Duration);
}

/* =========================================================================
 * ISR dispatch table
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
 * Private: map ParseResult_t → Master result + diagnostics
 * ========================================================================= */
static ModbusMaster_Result_t prv_ParseResultToMasterResult(
                                    ModbusMaster_Channel_t *pxMaster,
                                    ModbusRtu_ParseResult_t ePR)
{
    switch (ePR)
    {
        case MODBUS_RTU_PARSE_OK:
            return MB_RESULT_SUCCESS;

        case MODBUS_RTU_PARSE_EXCEPTION:
            pxMaster->xDiag.u32ErrException++;
            return MB_RESULT_ERR_EXCEPTION;

        case MODBUS_RTU_PARSE_ERR_OVERFLOW:
            pxMaster->xDiag.u32ErrOverflow++;
            return MB_RESULT_ERR_OVERFLOW;

        case MODBUS_RTU_PARSE_ERR_TOO_SHORT:
            pxMaster->xDiag.u32ErrFrameLen++;
            return MB_RESULT_ERR_TOO_SHORT;

        case MODBUS_RTU_PARSE_ERR_LENGTH:
            /*
             * This is the "frame split" case — the TCC timer fired too
             * early and the response was fragmented.  Logged as ERR_FRAME_LENGTH
             * so you can distinguish it from CRC errors.
             */
            pxMaster->xDiag.u32ErrFrameLen++;
            return MB_RESULT_ERR_FRAME_LENGTH;

        case MODBUS_RTU_PARSE_ERR_CRC:
            pxMaster->xDiag.u32ErrCRC++;
            return MB_RESULT_ERR_CRC;

        case MODBUS_RTU_PARSE_ERR_SLAVE_ADDR:
            pxMaster->xDiag.u32ErrSlaveAddr++;
            return MB_RESULT_ERR_SLAVE_ADDR;

        case MODBUS_RTU_PARSE_ERR_FC:
            pxMaster->xDiag.u32ErrFC++;
            return MB_RESULT_ERR_FUNCTION_CODE;

        case MODBUS_RTU_PARSE_ERR_DATA_LEN:
            pxMaster->xDiag.u32ErrFrameLen++;
            return MB_RESULT_ERR_FRAME_LENGTH;

        default:
            return MB_RESULT_ERR_FRAME_LENGTH;
    }
}

/* =========================================================================
 * Private: complete / retry helpers
 * ========================================================================= */
static void prv_CompleteTransaction(ModbusMaster_Channel_t     *pxMaster,
                                     ModbusMaster_Result_t       eResult,
                                     const ModbusRtu_Response_t *pxResp)
{
    if (eResult == MB_RESULT_SUCCESS)
    {
        pxMaster->xDiag.u32RxSuccess++;
    }

    pxMaster->xTx.bActive = false;

    if (pxMaster->pfnCallback != NULL)
    {
        pxMaster->pfnCallback(pxMaster->eChannel,
                               eResult,
                               pxResp,
                               pxMaster->pvCallbackCtx);
    }

    pxMaster->eState = MB_MASTER_STATE_IDLE;
}

static bool prv_RetryOrFail(ModbusMaster_Channel_t *pxMaster,
                              ModbusMaster_Result_t   eErrorResult)
{
    pxMaster->xDiag.u32Retries++;

    if (pxMaster->xTx.u8RetryCount < (uint8_t)MODBUS_MASTER_MAX_RETRIES)
    {
        pxMaster->xTx.u8RetryCount++;

        SYS_CONSOLE_PRINT("[ModbusMaster CH%u] Retry %u/%u after: %u\r\n",
                           (unsigned)pxMaster->eChannel,
                           (unsigned)pxMaster->xTx.u8RetryCount,
                           (unsigned)MODBUS_MASTER_MAX_RETRIES,
                           (unsigned)eErrorResult);

        /*
         * IMPORTANT: RxCtx is reset at the TOP of the SENDING state (below),
         * not here.  This ensures the reset happens as close as possible to
         * the actual transmit, giving minimum window for stale bytes to arrive.
         */
        pxMaster->eState = MB_MASTER_STATE_SENDING;
        return true;
    }

    /* Retries exhausted */
    SYS_CONSOLE_PRINT("[ModbusMaster CH%u] All retries exhausted, err=%u\r\n",
                       (unsigned)pxMaster->eChannel, (unsigned)eErrorResult);

    pxMaster->eState = MB_MASTER_STATE_ERROR;
    prv_CompleteTransaction(pxMaster, eErrorResult, NULL);
    return false;
}

/* =========================================================================
 * Public: Lifecycle
 * ========================================================================= */
ModbusMaster_Result_t Modbus_Master_Init(ModbusMaster_Channel_t *pxMaster,
                                          UartPort_Channel_t      eChannel)
{
    UartPort_Status_t ePortStatus;

    if ((pxMaster == NULL) || (eChannel >= UART_PORT_CH_MAX))
    {
        return MB_RESULT_ERR_PARAM;
    }

    (void)memset(pxMaster, 0x00, sizeof(ModbusMaster_Channel_t));
    pxMaster->eChannel = eChannel;
    pxMaster->eState   = MB_MASTER_STATE_IDLE;

    ModbusRtu_RxCtx_Reset(&pxMaster->xRxCtx);

    s_apxMasterCtx[eChannel] = pxMaster;

    ePortStatus = UartPort_Init(eChannel, prv_RxByteCallback, prv_FrameEndCallback);

    if (ePortStatus != UART_PORT_OK)
    {
        SYS_CONSOLE_PRINT("[ModbusMaster CH%u] Port init failed %u\r\n",
                           (unsigned)eChannel, (unsigned)ePortStatus);
        return MB_RESULT_ERR_HW;
    }

    SYS_CONSOLE_PRINT("[ModbusMaster CH%u] Init OK. T3.5=%lu µs\r\n",
                       (unsigned)eChannel,
                       (unsigned long)UartPort_GetT35Us(eChannel));

    return MB_RESULT_SUCCESS;
}

void Modbus_Master_RegisterCallback(ModbusMaster_Channel_t       *pxMaster,
                                     ModbusMaster_ResultCallback_t pfnCallback,
                                     void                         *pvContext)
{
    if (pxMaster == NULL)
    {
        return;
    }

    pxMaster->pfnCallback   = pfnCallback;
    pxMaster->pvCallbackCtx = pvContext;
}

bool Modbus_Master_IsIdle(const ModbusMaster_Channel_t *pxMaster)
{
    if (pxMaster == NULL)
    {
        return false;
    }

    return (pxMaster->eState == MB_MASTER_STATE_IDLE);
}

/* =========================================================================
 * Public: State machine — call from task every RS485_MASTER_POLL_MS
 * ========================================================================= */
void Modbus_Master_Process(ModbusMaster_Channel_t *pxMaster)
{
    UartPort_Status_t       ePortStatus;
    ModbusRtu_Response_t    xResp;
    ModbusRtu_ParseResult_t eParseResult;
    ModbusMaster_Result_t   eMasterResult;
    bool                    bFrameReady;

    if (pxMaster == NULL)
    {
        return;
    }

    switch (pxMaster->eState)
    {
        /* ----------------------------------------------------------------- */
        case MB_MASTER_STATE_IDLE:
            break;

        /* ----------------------------------------------------------------- */
        case MB_MASTER_STATE_SENDING:
        {
            /*
             * *** KEY FIX: Reset RxCtx HERE, immediately before transmitting.
             *
             * This is the correct position because:
             *   1. Any stale bytes from a previous transaction are discarded.
             *   2. The TCC timer is stopped (inside UartPort_SetTxMode) so
             *      no phantom frame-end can fire during our TX.
             *   3. UartPort_Transmit() flushes the echo after DE drops, so
             *      the accumulator only receives genuine slave response bytes.
             */
            ModbusRtu_RxCtx_Reset(&pxMaster->xRxCtx);

            pxMaster->xDiag.u32TxRequests++;

           /* SYS_CONSOLE_PRINT("[ModbusMaster CH%u] TX %u bytes → Slave 0x%02X FC=0x%02X retry=%u\r\n",
                               (unsigned)pxMaster->eChannel,
                               (unsigned)pxMaster->xTx.xRequest.u16Length,
                               (unsigned)pxMaster->xTx.u8SlaveAddr,
                               (unsigned)pxMaster->xTx.eFC,
                               (unsigned)pxMaster->xTx.u8RetryCount);*/

            /* Log the raw request bytes for debug */
            {
                uint16_t u16i;
                SYS_CONSOLE_PRINT("[ModbusMaster CH%u] TX bytes:", (unsigned)pxMaster->eChannel);
                for (u16i = 0U; u16i < pxMaster->xTx.xRequest.u16Length; u16i++)
                {
                    SYS_CONSOLE_PRINT(" %02X", pxMaster->xTx.xRequest.au8Frame[u16i]);
                }
                SYS_CONSOLE_PRINT("\r\n");
            }

            ePortStatus = UartPort_Transmit(pxMaster->eChannel,
                                             pxMaster->xTx.xRequest.au8Frame,
                                             pxMaster->xTx.xRequest.u16Length);

            if (ePortStatus != UART_PORT_OK)
            {
                pxMaster->xDiag.u32ErrHW++;
                (void)prv_RetryOrFail(pxMaster, MB_RESULT_ERR_HW);
                break;
            }

            pxMaster->xTx.u32TimeoutStart = prv_GetTickMs();
            pxMaster->eState = MB_MASTER_STATE_WAITING;
            break;
        }

        /* ----------------------------------------------------------------- */
        case MB_MASTER_STATE_WAITING:
        {
            taskENTER_CRITICAL();
            bFrameReady = (pxMaster->xRxCtx.eState == MODBUS_RTU_RX_FRAME_READY);
            taskEXIT_CRITICAL();

            if (bFrameReady)
            {
                SYS_CONSOLE_PRINT("[ModbusMaster CH%u] Frame ready, %u bytes:",
                                   (unsigned)pxMaster->eChannel,
                                   (unsigned)pxMaster->xRxCtx.u16Count);
                {
                    uint16_t u16i;
                    for (u16i = 0U; u16i < pxMaster->xRxCtx.u16Count; u16i++)
                    {
                        SYS_CONSOLE_PRINT(" %02X", pxMaster->xRxCtx.au8Buffer[u16i]);
                    }
                    SYS_CONSOLE_PRINT("\r\n");
                }

                pxMaster->eState = MB_MASTER_STATE_PROCESSING;
                /* Fall through to PROCESSING on the next call */
                break;
            }

            if (prv_ElapsedMs(pxMaster->xTx.u32TimeoutStart,
                               (uint32_t)MODBUS_MASTER_RESPONSE_TIMEOUT_MS))
            {
                SYS_CONSOLE_PRINT("[ModbusMaster CH%u] Timeout. Bytes so far: %u\r\n",
                                   (unsigned)pxMaster->eChannel,
                                   (unsigned)pxMaster->xRxCtx.u16Count);

                pxMaster->xDiag.u32ErrTimeout++;
                (void)prv_RetryOrFail(pxMaster, MB_RESULT_ERR_TIMEOUT);
            }
            break;
        }

        /* ----------------------------------------------------------------- */
        case MB_MASTER_STATE_PROCESSING:
        {
            (void)memset(&xResp, 0x00, sizeof(xResp));

            eParseResult = ModbusRtu_ParseResponse(
                               &pxMaster->xRxCtx,
                               pxMaster->xTx.u8SlaveAddr,
                               pxMaster->xTx.eFC,
                               pxMaster->xTx.u8RegCount,    /* ← FIX-C */
                               &xResp);

           /* SYS_CONSOLE_PRINT("[ModbusMaster CH%u] Parse result: %s (bytes=%u, expected=%u)\r\n",
                               (unsigned)pxMaster->eChannel,
                               ModbusRtu_ParseResultStr(eParseResult),
                               (unsigned)pxMaster->xRxCtx.u16Count,
                               (unsigned)ModbusRtu_GetExpectedResponseLen(
                                   pxMaster->xTx.eFC,
                                   pxMaster->xTx.u8RegCount));*/

            if (eParseResult == MODBUS_RTU_PARSE_OK)
            {
                pxMaster->eState = MB_MASTER_STATE_COMPLETE;
                prv_CompleteTransaction(pxMaster, MB_RESULT_SUCCESS, &xResp);
                break;
            }

            if (eParseResult == MODBUS_RTU_PARSE_EXCEPTION)
            {
                SYS_CONSOLE_PRINT("[ModbusMaster CH%u] Exception 0x%02X from slave 0x%02X\r\n",
                                   (unsigned)pxMaster->eChannel,
                                   (unsigned)xResp.eExceptionCode,
                                   (unsigned)xResp.u8SlaveAddr);

                prv_CompleteTransaction(pxMaster, MB_RESULT_ERR_EXCEPTION, &xResp);
                break;
            }

            /* All other parse errors → map to master result, then retry */
            eMasterResult = prv_ParseResultToMasterResult(pxMaster, eParseResult);
            (void)prv_RetryOrFail(pxMaster, eMasterResult);
            break;
        }

        /* ----------------------------------------------------------------- */
        case MB_MASTER_STATE_COMPLETE:
        case MB_MASTER_STATE_ERROR:
            pxMaster->eState = MB_MASTER_STATE_IDLE;
            break;

        default:
            pxMaster->eState = MB_MASTER_STATE_IDLE;
            break;
    }
}

/* =========================================================================
 * Public: Diagnostics
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

void Modbus_Master_ResetDiagnostics(ModbusMaster_Channel_t *pxMaster)
{
    if (pxMaster == NULL)
    {
        return;
    }

    (void)memset(&pxMaster->xDiag, 0x00, sizeof(ModbusMaster_Diagnostics_t));
}

/* =========================================================================
 * Public: Request submission
 * ========================================================================= */
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
                                                  u8SlaveAddr, u16RegAddr, u16RegCount);
    if (!bBuilt)
    {
        return MB_RESULT_ERR_PARAM;
    }

    pxMaster->xTx.u8SlaveAddr  = u8SlaveAddr;
    pxMaster->xTx.eFC          = MB_FC_READ_HOLDING_REGISTERS;
    pxMaster->xTx.u8RegCount   = (uint8_t)u16RegCount;
    pxMaster->xTx.u8RetryCount = 0U;
    pxMaster->xTx.bActive      = true;
    pxMaster->eState            = MB_MASTER_STATE_SENDING;

    return MB_RESULT_SUCCESS;
}

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
                                                 u8SlaveAddr, u16RegAddr, u16Value);
    if (!bBuilt)
    {
        return MB_RESULT_ERR_PARAM;
    }

    pxMaster->xTx.u8SlaveAddr  = u8SlaveAddr;
    pxMaster->xTx.eFC          = MB_FC_WRITE_SINGLE_REGISTER;
    pxMaster->xTx.u8RegCount   = 1U;
    pxMaster->xTx.u8RetryCount = 0U;
    pxMaster->xTx.bActive      = true;
    pxMaster->eState            = MB_MASTER_STATE_SENDING;

    return MB_RESULT_SUCCESS;
}

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
                                                    u8SlaveAddr, u16RegAddr,
                                                    pu16Values, u8RegCount);
    if (!bBuilt)
    {
        return MB_RESULT_ERR_PARAM;
    }

    pxMaster->xTx.u8SlaveAddr  = u8SlaveAddr;
    pxMaster->xTx.eFC          = MB_FC_WRITE_MULTIPLE_REGISTERS;
    pxMaster->xTx.u8RegCount   = u8RegCount;
    pxMaster->xTx.u8RetryCount = 0U;
    pxMaster->xTx.bActive      = true;
    pxMaster->eState            = MB_MASTER_STATE_SENDING;

    return MB_RESULT_SUCCESS;
}

/*******************************************************************************
 * End of File
 *******************************************************************************/