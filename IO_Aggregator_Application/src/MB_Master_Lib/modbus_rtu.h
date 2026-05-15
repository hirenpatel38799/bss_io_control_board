/*******************************************************************************
 * @file    modbus_rtu.h
 * @brief   Modbus RTU Frame Builder & Parser — Header
 *
 * BUG FIXES vs previous revision
 * ─────────────────────────────────
 * FIX-A  ParseResponse() now reports a distinct error code for each
 *        failure mode (CRC, slave addr, FC, length, overflow) so the
 *        master can log the exact cause and the caller can see which
 *        diagnostic counter to increment.
 *
 * FIX-B  Minimum frame length enforced before ANY field access.
 *        Previously, a 1-byte fragment (e.g. the lone 0x91 byte that
 *        arrived after the timer split the frame) would cause the parser
 *        to access buf[-1] for the CRC bytes.
 *
 * FIX-C  Response-length pre-check added.
 *        ParseResponse() now calls ModbusRtu_GetExpectedResponseLen()
 *        and rejects the frame before touching any data if the byte count
 *        does not match the expected length for the sent FC.
 *
 * FIX-D  RxCtx_Reset() added memory barrier to ensure ISR sees the clear.
 *
 * @company BACANCY SYSTEMS PVT. LTD.
 *******************************************************************************/
#ifndef MODBUS_RTU_H
#define MODBUS_RTU_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "uart_port.h"

/* =========================================================================
 * Frame size constants
 * ========================================================================= */
#define MODBUS_RTU_MAX_FRAME_SIZE        (256U)
#define MODBUS_RTU_MIN_FRAME_SIZE        (4U)    /* addr+FC+0data+CRC(2) */
#define MODBUS_RTU_MAX_DATA_BYTES        (250U)

/* =========================================================================
 * Modbus function codes
 * ========================================================================= */
typedef enum
{
    MB_FC_READ_COILS               = 0x01U,
    MB_FC_READ_DISCRETE_INPUTS     = 0x02U,
    MB_FC_READ_HOLDING_REGISTERS   = 0x03U,
    MB_FC_READ_INPUT_REGISTERS     = 0x04U,
    MB_FC_WRITE_SINGLE_COIL        = 0x05U,
    MB_FC_WRITE_SINGLE_REGISTER    = 0x06U,
    MB_FC_WRITE_MULTIPLE_COILS     = 0x0FU,
    MB_FC_WRITE_MULTIPLE_REGISTERS = 0x10U,
    MB_FC_EXCEPTION_OFFSET         = 0x80U
} ModbusFunctionCode_t;

/* =========================================================================
 * Exception codes
 * ========================================================================= */
typedef enum
{
    MB_EX_NONE                   = 0x00U,
    MB_EX_ILLEGAL_FUNCTION       = 0x01U,
    MB_EX_ILLEGAL_DATA_ADDRESS   = 0x02U,
    MB_EX_ILLEGAL_DATA_VALUE     = 0x03U,
    MB_EX_SERVER_DEVICE_FAILURE  = 0x04U,
    MB_EX_ACKNOWLEDGE            = 0x05U,
    MB_EX_SERVER_DEVICE_BUSY     = 0x06U,
    MB_EX_MEMORY_PARITY_ERROR    = 0x08U,
    MB_EX_GATEWAY_PATH_UNAVAIL   = 0x0AU,
    MB_EX_GATEWAY_TARGET_NO_RESP = 0x0BU
} ModbusException_t;

/* =========================================================================
 * FIX-A: Per-field parse error codes
 * =========================================================================
 * Returned by ModbusRtu_ParseResponse() so callers can log the exact cause.
 * ========================================================================= */
typedef enum
{
    MODBUS_RTU_PARSE_OK             = 0U,  /* Frame fully valid             */
    MODBUS_RTU_PARSE_ERR_NULL       = 1U,  /* NULL pointer argument         */
    MODBUS_RTU_PARSE_ERR_OVERFLOW   = 2U,  /* RX buffer overflowed          */
    MODBUS_RTU_PARSE_ERR_TOO_SHORT  = 3U,  /* Fewer bytes than minimum      */
    MODBUS_RTU_PARSE_ERR_LENGTH     = 4U,  /* Actual != expected byte count */
    MODBUS_RTU_PARSE_ERR_CRC        = 5U,  /* CRC mismatch                  */
    MODBUS_RTU_PARSE_ERR_SLAVE_ADDR = 6U,  /* Wrong slave address           */
    MODBUS_RTU_PARSE_ERR_FC         = 7U,  /* Unexpected function code      */
    MODBUS_RTU_PARSE_ERR_DATA_LEN   = 8U,  /* Data byte count field invalid */
    MODBUS_RTU_PARSE_EXCEPTION      = 9U   /* Slave returned exception       */
} ModbusRtu_ParseResult_t;

/* =========================================================================
 * RX FSM states
 * ========================================================================= */
typedef enum
{
    MODBUS_RTU_RX_IDLE        = 0U,
    MODBUS_RTU_RX_RECEIVING   = 1U,
    MODBUS_RTU_RX_FRAME_READY = 2U
} ModbusRtu_RxState_t;

/* =========================================================================
 * Data structures
 * ========================================================================= */
typedef struct
{
    uint8_t              u8SlaveAddr;
    ModbusFunctionCode_t eFuncCode;
    bool                 bIsException;
    ModbusException_t    eExceptionCode;
    uint8_t              au8Data[MODBUS_RTU_MAX_DATA_BYTES];
    uint8_t              u8DataLen;
} ModbusRtu_Response_t;

typedef struct
{
    uint8_t  au8Frame[MODBUS_RTU_MAX_FRAME_SIZE];
    uint16_t u16Length;
} ModbusRtu_Request_t;

typedef struct
{
    uint8_t             au8Buffer[MODBUS_RTU_MAX_FRAME_SIZE];
    uint16_t            u16Count;
    ModbusRtu_RxState_t eState;
    bool                bOverflow;
} ModbusRtu_RxCtx_t;

/* =========================================================================
 * Frame builders
 * ========================================================================= */
bool ModbusRtu_BuildReadHoldingRegisters(ModbusRtu_Request_t *pxReq,
                                          uint8_t              u8SlaveAddr,
                                          uint16_t             u16RegAddr,
                                          uint16_t             u16RegCount);

bool ModbusRtu_BuildWriteSingleRegister(ModbusRtu_Request_t *pxReq,
                                         uint8_t              u8SlaveAddr,
                                         uint16_t             u16RegAddr,
                                         uint16_t             u16Value);

bool ModbusRtu_BuildWriteMultipleRegisters(ModbusRtu_Request_t *pxReq,
                                            uint8_t              u8SlaveAddr,
                                            uint16_t             u16RegAddr,
                                            const uint16_t      *pu16Values,
                                            uint8_t              u8RegCount);

/* =========================================================================
 * RX accumulator
 * ========================================================================= */

/** Reset the accumulator — call BEFORE transmitting the request (not after). */
void ModbusRtu_RxCtx_Reset(ModbusRtu_RxCtx_t *pxCtx);

/** Feed one byte (from ISR callback). */
void ModbusRtu_RxFeedByte(ModbusRtu_RxCtx_t *pxCtx, uint8_t u8Byte);

/** Signal end-of-frame (from timer ISR). */
void ModbusRtu_RxFrameEnd(ModbusRtu_RxCtx_t *pxCtx);

/* =========================================================================
 * FIX-A / FIX-B / FIX-C: hardened parser
 * ─────────────────────────────────────────────────────────────────────────
 * Returns a ModbusRtu_ParseResult_t (not just bool) so the caller knows
 * exactly which validation step failed.
 * pxResp is populated only when the return value is MODBUS_RTU_PARSE_OK
 * or MODBUS_RTU_PARSE_EXCEPTION.
 * ========================================================================= */
ModbusRtu_ParseResult_t ModbusRtu_ParseResponse(const ModbusRtu_RxCtx_t *pxCtx,
                                                  uint8_t                  u8ExpectedAddr,
                                                  ModbusFunctionCode_t     eExpectedFC,
                                                  uint8_t                  u8RegCount,
                                                  ModbusRtu_Response_t    *pxResp);

/** Expected response byte count for a given FC + register count. */
uint16_t ModbusRtu_GetExpectedResponseLen(ModbusFunctionCode_t eFuncCode,
                                           uint8_t              u8RegCount);

/** Human-readable string for a parse result code (for debug logging). */
const char *ModbusRtu_ParseResultStr(ModbusRtu_ParseResult_t eResult);

#endif /* MODBUS_RTU_H */
/*******************************************************************************
 * End of File
 *******************************************************************************/