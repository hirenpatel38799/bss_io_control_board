/*******************************************************************************
 * @file    modbus_master.h
 * @brief   Modbus RTU Master Stack — Header
 *
 * Changes vs previous revision
 * ─────────────────────────────
 * • ModbusMaster_Transaction_t now stores u8RegCount so that it can be
 *   passed to ModbusRtu_ParseResponse() for the expected-length pre-check.
 *
 * • ModbusMaster_Process() uses ModbusRtu_ParseResult_t (not bool) from
 *   the parser, maps each code to the correct diagnostic counter, and
 *   logs a descriptive string via ModbusRtu_ParseResultStr().
 *
 * • RxCtx is reset BEFORE transmitting (not after) to ensure no stale
 *   bytes from a previous transaction can pollute the new accumulation.
 *
 * @company BACANCY SYSTEMS PVT. LTD.
 *******************************************************************************/
#ifndef MODBUS_MASTER_H
#define MODBUS_MASTER_H

#include <stdint.h>
#include <stdbool.h>
#include "modbus_rtu.h"
#include "uart_port.h"

/* =========================================================================
 * Configuration
 * ========================================================================= */
#define MODBUS_MASTER_MAX_RETRIES            (3U)
#define MODBUS_MASTER_RESPONSE_TIMEOUT_MS    (500U)  /* raised from 200 ms */
#define MODBUS_MASTER_INTER_REQUEST_GAP_MS   (10U)

/* =========================================================================
 * State machine
 * ========================================================================= */
typedef enum
{
    MB_MASTER_STATE_IDLE       = 0U,
    MB_MASTER_STATE_SENDING    = 1U,
    MB_MASTER_STATE_WAITING    = 2U,
    MB_MASTER_STATE_PROCESSING = 3U,
    MB_MASTER_STATE_COMPLETE   = 4U,
    MB_MASTER_STATE_ERROR      = 5U
} ModbusMaster_State_t;

/* =========================================================================
 * Result codes
 * ========================================================================= */
typedef enum
{
    MB_RESULT_SUCCESS           = 0U,
    MB_RESULT_ERR_TIMEOUT       = 1U,
    MB_RESULT_ERR_CRC           = 2U,
    MB_RESULT_ERR_SLAVE_ADDR    = 3U,
    MB_RESULT_ERR_FUNCTION_CODE = 4U,
    MB_RESULT_ERR_EXCEPTION     = 5U,
    MB_RESULT_ERR_FRAME_LENGTH  = 6U,
    MB_RESULT_ERR_OVERFLOW      = 7U,
    MB_RESULT_ERR_HW            = 8U,
    MB_RESULT_ERR_BUSY          = 9U,
    MB_RESULT_ERR_PARAM         = 10U,
    MB_RESULT_ERR_TOO_SHORT     = 11U
} ModbusMaster_Result_t;

/* =========================================================================
 * Diagnostic counters
 * ========================================================================= */
typedef struct
{
    uint32_t u32TxRequests;
    uint32_t u32RxSuccess;
    uint32_t u32ErrTimeout;
    uint32_t u32ErrCRC;
    uint32_t u32ErrException;
    uint32_t u32ErrOverflow;
    uint32_t u32ErrHW;
    uint32_t u32ErrSlaveAddr;
    uint32_t u32ErrFrameLen;    /* frame-split hits logged here */
    uint32_t u32ErrFC;
    uint32_t u32Retries;
} ModbusMaster_Diagnostics_t;

/* =========================================================================
 * Callback
 * ========================================================================= */
typedef void (*ModbusMaster_ResultCallback_t)(
                    UartPort_Channel_t          eChannel,
                    ModbusMaster_Result_t       eResult,
                    const ModbusRtu_Response_t *pxResp,
                    void                       *pvContext);

/* =========================================================================
 * Transaction descriptor
 * ========================================================================= */
typedef struct
{
    ModbusRtu_Request_t  xRequest;
    ModbusFunctionCode_t eFC;
    uint8_t              u8SlaveAddr;
    uint8_t              u8RegCount;      /* needed for expected-length check */
    uint8_t              u8RetryCount;
    uint32_t             u32TimeoutStart;
    bool                 bActive;
} ModbusMaster_Transaction_t;

/* =========================================================================
 * Channel context
 * ========================================================================= */
typedef struct
{
    UartPort_Channel_t          eChannel;
    ModbusMaster_State_t        eState;
    ModbusMaster_Transaction_t  xTx;
    ModbusRtu_RxCtx_t           xRxCtx;
    ModbusMaster_Diagnostics_t  xDiag;
    ModbusMaster_ResultCallback_t pfnCallback;
    void                       *pvCallbackCtx;
} ModbusMaster_Channel_t;

/* =========================================================================
 * Public API
 * ========================================================================= */
ModbusMaster_Result_t Modbus_Master_Init(ModbusMaster_Channel_t *pxMaster,
                                          UartPort_Channel_t      eChannel);

void Modbus_Master_RegisterCallback(ModbusMaster_Channel_t       *pxMaster,
                                     ModbusMaster_ResultCallback_t pfnCallback,
                                     void                         *pvContext);

void Modbus_Master_Process(ModbusMaster_Channel_t *pxMaster);

void Modbus_Master_GetDiagnostics(const ModbusMaster_Channel_t *pxMaster,
                                   ModbusMaster_Diagnostics_t   *pxDiag);

void Modbus_Master_ResetDiagnostics(ModbusMaster_Channel_t *pxMaster);

bool Modbus_Master_IsIdle(const ModbusMaster_Channel_t *pxMaster);

ModbusMaster_Result_t Modbus_ReadHoldingRegisters(ModbusMaster_Channel_t *pxMaster,
                                                   uint8_t                 u8SlaveAddr,
                                                   uint16_t                u16RegAddr,
                                                   uint16_t                u16RegCount);

ModbusMaster_Result_t Modbus_WriteSingleRegister(ModbusMaster_Channel_t *pxMaster,
                                                  uint8_t                 u8SlaveAddr,
                                                  uint16_t                u16RegAddr,
                                                  uint16_t                u16Value);

ModbusMaster_Result_t Modbus_WriteMultipleRegisters(ModbusMaster_Channel_t *pxMaster,
                                                     uint8_t                 u8SlaveAddr,
                                                     uint16_t                u16RegAddr,
                                                     const uint16_t         *pu16Values,
                                                     uint8_t                 u8RegCount);

#endif /* MODBUS_MASTER_H */
/*******************************************************************************
 * End of File
 *******************************************************************************/