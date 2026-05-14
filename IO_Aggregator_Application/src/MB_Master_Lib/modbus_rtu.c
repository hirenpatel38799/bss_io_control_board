/*******************************************************************************
 * @file    modbus_rtu.c
 * @brief   Modbus RTU Frame Builder & Parser - Implementation
 * @details Implements frame construction (request) and frame parsing
 *          (response) for the Modbus RTU transport encoding.
 *
 * @company BACANCY SYSTEMS PVT. LTD.
 *******************************************************************************/

/* =========================================================================
 * Included Files
 * ========================================================================= */
#include "modbus_rtu.h"
#include "modbus_crc.h"
#include <string.h>

/* =========================================================================
 * Private Constants
 * ========================================================================= */

/** Modbus slave address range */
#define MB_SLAVE_ADDR_MIN           (1U)
#define MB_SLAVE_ADDR_MAX           (247U)

/** FC03/FC04: max registers per request */
#define MB_READ_REG_MAX             (125U)

/** FC16: max registers per write request */
#define MB_WRITE_MULTI_REG_MAX      (123U)

/* Frame byte offsets common to all PDUs */
#define MB_FRAME_OFFSET_SLAVE       (0U)
#define MB_FRAME_OFFSET_FC          (1U)
#define MB_FRAME_OFFSET_DATA_START  (2U)

/* FC03/FC04 response: fixed overhead bytes (SlaveAddr + FC + ByteCount + CRC) */
#define MB_RD_RESP_OVERHEAD_BYTES   (5U)

/* FC06 response: always 8 bytes (addr + FC + reg_hi + reg_lo + val_hi + val_lo + CRC) */
#define MB_WR_SINGLE_RESP_LEN       (8U)

/* FC16 response: always 8 bytes (addr + FC + start_hi + start_lo + qty_hi + qty_lo + CRC) */
#define MB_WR_MULTI_RESP_LEN        (8U)

/* =========================================================================
 * Private Helper: frame builder utility
 * ========================================================================= */

/**
 * @brief  Write a 16-bit value into a frame buffer in big-endian order.
 */
static inline void prv_WriteU16BE(uint8_t *pu8Dst, uint16_t u16Val)
{
    pu8Dst[0U] = (uint8_t)(u16Val >> 8U);
    pu8Dst[1U] = (uint8_t)(u16Val & 0x00FFU);
}

/* =========================================================================
 * Public Function Definitions — Frame Builders
 * ========================================================================= */

bool ModbusRtu_BuildReadHoldingRegisters(ModbusRtu_Request_t *pxReq,
                                          uint8_t              u8SlaveAddr,
                                          uint16_t             u16RegAddr,
                                          uint16_t             u16RegCount)
{
    if ((pxReq == NULL)                    ||
        (u8SlaveAddr < MB_SLAVE_ADDR_MIN)  ||
        (u8SlaveAddr > MB_SLAVE_ADDR_MAX)  ||
        (u16RegCount == 0U)                ||
        (u16RegCount > (uint16_t)MB_READ_REG_MAX))
    {
        return false;
    }

    pxReq->au8Frame[MB_FRAME_OFFSET_SLAVE] = u8SlaveAddr;
    pxReq->au8Frame[MB_FRAME_OFFSET_FC]    = (uint8_t)MB_FC_READ_HOLDING_REGISTERS;

    prv_WriteU16BE(&pxReq->au8Frame[2U], u16RegAddr);
    prv_WriteU16BE(&pxReq->au8Frame[4U], u16RegCount);

    /* Append CRC over bytes 0..5 and update total length */
    pxReq->u16Length = Modbus_CRC16_Append(pxReq->au8Frame, 6U);

    return true;
}

/* ------------------------------------------------------------------------- */

bool ModbusRtu_BuildWriteSingleRegister(ModbusRtu_Request_t *pxReq,
                                         uint8_t              u8SlaveAddr,
                                         uint16_t             u16RegAddr,
                                         uint16_t             u16Value)
{
    if ((pxReq == NULL)                   ||
        (u8SlaveAddr < MB_SLAVE_ADDR_MIN) ||
        (u8SlaveAddr > MB_SLAVE_ADDR_MAX))
    {
        return false;
    }

    pxReq->au8Frame[MB_FRAME_OFFSET_SLAVE] = u8SlaveAddr;
    pxReq->au8Frame[MB_FRAME_OFFSET_FC]    = (uint8_t)MB_FC_WRITE_SINGLE_REGISTER;

    prv_WriteU16BE(&pxReq->au8Frame[2U], u16RegAddr);
    prv_WriteU16BE(&pxReq->au8Frame[4U], u16Value);

    pxReq->u16Length = Modbus_CRC16_Append(pxReq->au8Frame, 6U);

    return true;
}

/* ------------------------------------------------------------------------- */

bool ModbusRtu_BuildWriteMultipleRegisters(ModbusRtu_Request_t *pxReq,
                                            uint8_t              u8SlaveAddr,
                                            uint16_t             u16RegAddr,
                                            const uint16_t      *pu16Values,
                                            uint8_t              u8RegCount)
{
    uint8_t  u8ByteCount;
    uint16_t u16DataLen;
    uint8_t  u8Idx;
    uint8_t  u8Offset;

    if ((pxReq == NULL)                         ||
        (pu16Values == NULL)                    ||
        (u8SlaveAddr < MB_SLAVE_ADDR_MIN)       ||
        (u8SlaveAddr > MB_SLAVE_ADDR_MAX)       ||
        (u8RegCount == 0U)                      ||
        (u8RegCount > (uint8_t)MB_WRITE_MULTI_REG_MAX))
    {
        return false;
    }

    u8ByteCount = (uint8_t)(u8RegCount * 2U);    /* Two bytes per 16-bit register */
    u16DataLen  = (uint16_t)7U + (uint16_t)u8ByteCount;  /* Header bytes before CRC */

    /* Check total frame fits */
    if ((u16DataLen + 2U) > MODBUS_RTU_MAX_FRAME_SIZE)
    {
        return false;
    }

    pxReq->au8Frame[MB_FRAME_OFFSET_SLAVE] = u8SlaveAddr;
    pxReq->au8Frame[MB_FRAME_OFFSET_FC]    = (uint8_t)MB_FC_WRITE_MULTIPLE_REGISTERS;

    prv_WriteU16BE(&pxReq->au8Frame[2U], u16RegAddr);
    prv_WriteU16BE(&pxReq->au8Frame[4U], (uint16_t)u8RegCount);

    pxReq->au8Frame[6U] = u8ByteCount;   /* Byte count field */

    /* Serialise register values, big-endian */
    u8Offset = 7U;
    for (u8Idx = 0U; u8Idx < u8RegCount; u8Idx++)
    {
        prv_WriteU16BE(&pxReq->au8Frame[u8Offset], pu16Values[u8Idx]);
        u8Offset += 2U;
    }

    pxReq->u16Length = Modbus_CRC16_Append(pxReq->au8Frame, u16DataLen);

    return true;
}

/* =========================================================================
 * Public Function Definitions — RX Parser
 * ========================================================================= */

void ModbusRtu_RxCtx_Reset(ModbusRtu_RxCtx_t *pxCtx)
{
    if (pxCtx == NULL)
    {
        return;
    }

    (void)memset(pxCtx->au8Buffer, 0x00, sizeof(pxCtx->au8Buffer));
    pxCtx->u16Count  = 0U;
    pxCtx->eState    = MODBUS_RTU_RX_IDLE;
    pxCtx->bOverflow = false;
}

/* ------------------------------------------------------------------------- */

void ModbusRtu_RxFeedByte(ModbusRtu_RxCtx_t *pxCtx, uint8_t u8Byte)
{
    /* Called from ISR — must be fast and non-blocking */
    if (pxCtx == NULL)
    {
        return;
    }

    /* If a complete frame is pending processing, discard new bytes until
     * the master task has consumed and reset the context.            */
    if (pxCtx->eState == MODBUS_RTU_RX_FRAME_READY)
    {
        return;
    }

    if (pxCtx->u16Count >= MODBUS_RTU_MAX_FRAME_SIZE)
    {
        pxCtx->bOverflow = true;
        return;
    }

    pxCtx->au8Buffer[pxCtx->u16Count] = u8Byte;
    pxCtx->u16Count++;
    pxCtx->eState = MODBUS_RTU_RX_RECEIVING;
}

/* ------------------------------------------------------------------------- */

void ModbusRtu_RxFrameEnd(ModbusRtu_RxCtx_t *pxCtx)
{
    /* Called from timer ISR — must be fast */
    if (pxCtx == NULL)
    {
        return;
    }

    if (pxCtx->u16Count > 0U)
    {
        pxCtx->eState = MODBUS_RTU_RX_FRAME_READY;
    }
    else
    {
        pxCtx->eState = MODBUS_RTU_RX_IDLE;
    }
}

/* ------------------------------------------------------------------------- */

bool ModbusRtu_ParseResponse(const ModbusRtu_RxCtx_t *pxCtx,
                              uint8_t                  u8ExpectedAddr,
                              ModbusFunctionCode_t     eExpectedFC,
                              ModbusRtu_Response_t    *pxResp)
{
    uint8_t u8FC;
    uint8_t u8DataByteCount;

    /* Parameter guards */
    if ((pxCtx == NULL) || (pxResp == NULL))
    {
        return false;
    }

    /* Overflow during reception */
    if (pxCtx->bOverflow)
    {
        return false;
    }

    /* Absolute minimum frame size check */
    if (pxCtx->u16Count < (uint16_t)MODBUS_RTU_MIN_FRAME_SIZE)
    {
        return false;
    }

    /* Slave address check */
    if (pxCtx->au8Buffer[MB_FRAME_OFFSET_SLAVE] != u8ExpectedAddr)
    {
        return false;
    }

    /* CRC validation — must happen before trusting any other field */
    if (!Modbus_CRC16_Validate(pxCtx->au8Buffer, pxCtx->u16Count))
    {
        return false;
    }

    u8FC = pxCtx->au8Buffer[MB_FRAME_OFFSET_FC];

    /* Exception response detection */
    if ((u8FC & (uint8_t)MB_FC_EXCEPTION_OFFSET) != 0U)
    {
        pxResp->u8SlaveAddr   = pxCtx->au8Buffer[MB_FRAME_OFFSET_SLAVE];
        pxResp->eFuncCode     = (ModbusFunctionCode_t)(u8FC & ~(uint8_t)MB_FC_EXCEPTION_OFFSET);
        pxResp->bIsException  = true;
        pxResp->eExceptionCode = (ModbusException_t)pxCtx->au8Buffer[MB_FRAME_OFFSET_DATA_START];
        pxResp->u8DataLen     = 0U;
        return true;  /* Parsed successfully (exception is a valid response) */
    }

    /* Function code match check */
    if (u8FC != (uint8_t)eExpectedFC)
    {
        return false;
    }

    pxResp->u8SlaveAddr  = pxCtx->au8Buffer[MB_FRAME_OFFSET_SLAVE];
    pxResp->eFuncCode    = eExpectedFC;
    pxResp->bIsException = false;
    pxResp->eExceptionCode = MB_EX_NONE;

    /* Extract data bytes per function code */
    switch (eExpectedFC)
    {
        case MB_FC_READ_HOLDING_REGISTERS:
        case MB_FC_READ_INPUT_REGISTERS:
        case MB_FC_READ_COILS:
        case MB_FC_READ_DISCRETE_INPUTS:
        {
            /* Response: [Addr][FC][ByteCount][Data...][CRC16] */
            u8DataByteCount = pxCtx->au8Buffer[MB_FRAME_OFFSET_DATA_START];

            /* Validate declared length against actual received length */
            if (pxCtx->u16Count != ((uint16_t)u8DataByteCount + (uint16_t)MB_RD_RESP_OVERHEAD_BYTES))
            {
                return false;
            }

            if (u8DataByteCount > MODBUS_RTU_MAX_DATA_BYTES)
            {
                return false;
            }

            (void)memcpy(pxResp->au8Data,
                         &pxCtx->au8Buffer[3U],
                         (size_t)u8DataByteCount);
            pxResp->u8DataLen = u8DataByteCount;
            break;
        }

        case MB_FC_WRITE_SINGLE_REGISTER:
        case MB_FC_WRITE_SINGLE_COIL:
        {
            /* Echo response: 8 bytes total */
            if (pxCtx->u16Count != (uint16_t)MB_WR_SINGLE_RESP_LEN)
            {
                return false;
            }

            (void)memcpy(pxResp->au8Data,
                         &pxCtx->au8Buffer[MB_FRAME_OFFSET_DATA_START],
                         4U);    /* reg_addr(2) + value(2) */
            pxResp->u8DataLen = 4U;
            break;
        }

        case MB_FC_WRITE_MULTIPLE_REGISTERS:
        case MB_FC_WRITE_MULTIPLE_COILS:
        {
            /* Echo response: 8 bytes total */
            if (pxCtx->u16Count != (uint16_t)MB_WR_MULTI_RESP_LEN)
            {
                return false;
            }

            (void)memcpy(pxResp->au8Data,
                         &pxCtx->au8Buffer[MB_FRAME_OFFSET_DATA_START],
                         4U);    /* start_addr(2) + quantity(2) */
            pxResp->u8DataLen = 4U;
            break;
        }

        default:
            /* Unsupported function code */
            return false;
    }

    return true;
}

/* ------------------------------------------------------------------------- */

uint16_t ModbusRtu_GetExpectedResponseLen(ModbusFunctionCode_t eFuncCode,
                                           uint8_t              u8RegCount)
{
    uint16_t u16Len = 0U;

    switch (eFuncCode)
    {
        case MB_FC_READ_HOLDING_REGISTERS:
        case MB_FC_READ_INPUT_REGISTERS:
            /* [Addr(1)][FC(1)][ByteCount(1)][Data(2*n)][CRC(2)] */
            u16Len = (uint16_t)5U + (uint16_t)(u8RegCount * 2U);
            break;

        case MB_FC_READ_COILS:
        case MB_FC_READ_DISCRETE_INPUTS:
            /* Coils/DI: byte count = ceil(n/8) */
            u16Len = (uint16_t)5U + (uint16_t)((u8RegCount + 7U) / 8U);
            break;

        case MB_FC_WRITE_SINGLE_REGISTER:
        case MB_FC_WRITE_SINGLE_COIL:
            u16Len = (uint16_t)MB_WR_SINGLE_RESP_LEN;
            break;

        case MB_FC_WRITE_MULTIPLE_REGISTERS:
        case MB_FC_WRITE_MULTIPLE_COILS:
            u16Len = (uint16_t)MB_WR_MULTI_RESP_LEN;
            break;

        default:
            u16Len = 0U;    /* Unknown / unsupported */
            break;
    }

    return u16Len;
}

/*******************************************************************************
 * End of File
 *******************************************************************************/
