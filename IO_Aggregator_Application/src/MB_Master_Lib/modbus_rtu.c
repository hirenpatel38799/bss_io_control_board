/*******************************************************************************
 * @file    modbus_rtu.c
 * @brief   Modbus RTU Frame Builder & Parser — Implementation
 *
 * Key fixes
 * ─────────────────────────────────────────────────────────────────────────
 * FIX-A  ParseResponse() returns ModbusRtu_ParseResult_t per-field code.
 * FIX-B  Absolute minimum frame length enforced before any field access.
 * FIX-C  Expected-length pre-check via ModbusRtu_GetExpectedResponseLen().
 * FIX-D  RxCtx_Reset() issues __DMB() so the ISR sees the cleared count.
 *
 * @company BACANCY SYSTEMS PVT. LTD.
 *******************************************************************************/

#include "modbus_rtu.h"
#include "modbus_crc.h"
#include <string.h>
#include "definitions.h"    /* SYS_CONSOLE_PRINT */
/* =========================================================================
 * Private constants
 * ========================================================================= */
#define MB_SLAVE_ADDR_MIN           (1U)
#define MB_SLAVE_ADDR_MAX           (247U)
#define MB_READ_REG_MAX             (125U)
#define MB_WRITE_MULTI_REG_MAX      (123U)

#define MB_OFFSET_SLAVE             (0U)
#define MB_OFFSET_FC                (1U)
#define MB_OFFSET_DATA              (2U)

/* FC03/04 response overhead: addr(1)+FC(1)+bytecount(1)+CRC(2) = 5 */
#define MB_RD_RESP_OVERHEAD         (5U)
/* FC06/FC16 response: always exactly 8 bytes */
#define MB_WR_SINGLE_RESP_LEN       (8U)
#define MB_WR_MULTI_RESP_LEN        (8U)
/* Exception response: always exactly 5 bytes */
#define MB_EXCEPTION_RESP_LEN       (5U)

/* =========================================================================
 * Private helper
 * ========================================================================= */
static inline void prv_WriteU16BE(uint8_t *pu8Dst, uint16_t u16Val)
{
    pu8Dst[0U] = (uint8_t)(u16Val >> 8U);
    pu8Dst[1U] = (uint8_t)(u16Val & 0x00FFU);
}

/* =========================================================================
 * Frame builders (unchanged from previous revision — these are correct)
 * ========================================================================= */

bool ModbusRtu_BuildReadHoldingRegisters(ModbusRtu_Request_t *pxReq,
                                          uint8_t              u8SlaveAddr,
                                          uint16_t             u16RegAddr,
                                          uint16_t             u16RegCount)
{
    if ((pxReq == NULL)                   ||
        (u8SlaveAddr < MB_SLAVE_ADDR_MIN) ||
        (u8SlaveAddr > MB_SLAVE_ADDR_MAX) ||
        (u16RegCount == 0U)               ||
        (u16RegCount > (uint16_t)MB_READ_REG_MAX))
    {
        return false;
    }

    pxReq->au8Frame[0U] = u8SlaveAddr;
    pxReq->au8Frame[1U] = (uint8_t)MB_FC_READ_HOLDING_REGISTERS;
    prv_WriteU16BE(&pxReq->au8Frame[2U], u16RegAddr);
    prv_WriteU16BE(&pxReq->au8Frame[4U], u16RegCount);
    pxReq->u16Length = Modbus_CRC16_Append(pxReq->au8Frame, 6U);
    return true;
}

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

    pxReq->au8Frame[0U] = u8SlaveAddr;
    pxReq->au8Frame[1U] = (uint8_t)MB_FC_WRITE_SINGLE_REGISTER;
    prv_WriteU16BE(&pxReq->au8Frame[2U], u16RegAddr);
    prv_WriteU16BE(&pxReq->au8Frame[4U], u16Value);
    pxReq->u16Length = Modbus_CRC16_Append(pxReq->au8Frame, 6U);
    return true;
}

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

    if ((pxReq == NULL)                      ||
        (pu16Values == NULL)                 ||
        (u8SlaveAddr < MB_SLAVE_ADDR_MIN)    ||
        (u8SlaveAddr > MB_SLAVE_ADDR_MAX)    ||
        (u8RegCount == 0U)                   ||
        (u8RegCount > (uint8_t)MB_WRITE_MULTI_REG_MAX))
    {
        return false;
    }

    u8ByteCount = (uint8_t)(u8RegCount * 2U);
    u16DataLen  = (uint16_t)7U + (uint16_t)u8ByteCount;

    if ((u16DataLen + 2U) > MODBUS_RTU_MAX_FRAME_SIZE)
    {
        return false;
    }

    pxReq->au8Frame[0U] = u8SlaveAddr;
    pxReq->au8Frame[1U] = (uint8_t)MB_FC_WRITE_MULTIPLE_REGISTERS;
    prv_WriteU16BE(&pxReq->au8Frame[2U], u16RegAddr);
    prv_WriteU16BE(&pxReq->au8Frame[4U], (uint16_t)u8RegCount);
    pxReq->au8Frame[6U] = u8ByteCount;

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
 * RX accumulator
 * ========================================================================= */

void ModbusRtu_RxCtx_Reset(ModbusRtu_RxCtx_t *pxCtx)
{
    if (pxCtx == NULL)
    {
        return;
    }

    (void)memset(pxCtx->au8Buffer, 0x00U, sizeof(pxCtx->au8Buffer));
    pxCtx->u16Count  = 0U;
    pxCtx->eState    = MODBUS_RTU_RX_IDLE;
    pxCtx->bOverflow = false;

    /* FIX-D: memory barrier so the ISR sees the zeroed count immediately */
    __DMB();
}

/* ------------------------------------------------------------------------- */

void ModbusRtu_RxFeedByte(ModbusRtu_RxCtx_t *pxCtx, uint8_t u8Byte)
{
    /* Called from ISR — must be fast */
    if (pxCtx == NULL)
    {
        return;
    }

    /* Discard bytes if a complete frame is pending processing */
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
    /* Called from timer ISR */
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

/* =========================================================================
 * FIX-A/B/C: Hardened response parser
 * =========================================================================
 * Validation order — MUST be preserved:
 *   1. NULL guard
 *   2. Overflow flag
 *   3. Absolute minimum length (MODBUS_RTU_MIN_FRAME_SIZE)
 *   4. *** FIX-C *** Expected length pre-check (frame-split detection)
 *   5. CRC validation (over actual received bytes)
 *   6. Slave address
 *   7. FC / exception detection
 *   8. Data extraction
 *
 * The MOST IMPORTANT addition is step 4.  Without it, a partial frame
 * (e.g. a single 0x91 byte) passes steps 1-3 and only fails at CRC —
 * producing a misleading "slave=0x91, CRC=0x9191" log.  With step 4 the
 * frame is rejected immediately with MODBUS_RTU_PARSE_ERR_LENGTH and the
 * log clearly says "got 1 byte, expected 8".
 * ========================================================================= */
ModbusRtu_ParseResult_t ModbusRtu_ParseResponse(const ModbusRtu_RxCtx_t *pxCtx,
                                                  uint8_t                  u8ExpectedAddr,
                                                  ModbusFunctionCode_t     eExpectedFC,
                                                  uint8_t                  u8RegCount,
                                                  ModbusRtu_Response_t    *pxResp)
{
    uint16_t u16Expected;
    uint8_t  u8FC;
    uint8_t  u8DataByteCount;

    /* Step 1: NULL guard */
    if ((pxCtx == NULL) || (pxResp == NULL))
    {
        return MODBUS_RTU_PARSE_ERR_NULL;
    }

    /* Step 2: Overflow during reception */
    if (pxCtx->bOverflow)
    {
        return MODBUS_RTU_PARSE_ERR_OVERFLOW;
    }

    /* Step 3: FIX-B — absolute minimum before touching any index */
    if (pxCtx->u16Count < (uint16_t)MODBUS_RTU_MIN_FRAME_SIZE)
    {
        return MODBUS_RTU_PARSE_ERR_TOO_SHORT;
    }

    /*
     * Step 4: FIX-C — expected length pre-check.
     *
     * Peek at buf[1] (function code) to determine whether this might be
     * an exception (which is always 5 bytes) or a normal response.
     * We do NOT trust buf[1] yet (CRC hasn't been validated) but we can
     * use it to pick the expected length:
     *   - If buf[1] has the exception bit set → expect 5 bytes
     *   - Otherwise → expect the length for eExpectedFC + u8RegCount
     *
     * If the actual byte count does not match either expectation, the
     * frame is a fragment (timer fired too early) or noise — reject it.
     */
    u8FC = pxCtx->au8Buffer[MB_OFFSET_FC];

    if ((u8FC & (uint8_t)MB_FC_EXCEPTION_OFFSET) != 0U)
    {
        u16Expected = (uint16_t)MB_EXCEPTION_RESP_LEN;
    }
    else
    {
        u16Expected = ModbusRtu_GetExpectedResponseLen(eExpectedFC, u8RegCount);
    }

    if ((u16Expected > 0U) && (pxCtx->u16Count != u16Expected))
    {
        /* Length mismatch — frame was split or is garbage */
        return MODBUS_RTU_PARSE_ERR_LENGTH;
    }

    /* Step 5: CRC — only now that we trust the byte count */
    if (!Modbus_CRC16_Validate(pxCtx->au8Buffer, pxCtx->u16Count))
    {
        return MODBUS_RTU_PARSE_ERR_CRC;
    }

    /* Step 6: Slave address — CRC is good so we can trust the fields */
    if (pxCtx->au8Buffer[MB_OFFSET_SLAVE] != u8ExpectedAddr)
    {
        return MODBUS_RTU_PARSE_ERR_SLAVE_ADDR;
    }

    /* Step 7: Exception check */
    if ((u8FC & (uint8_t)MB_FC_EXCEPTION_OFFSET) != 0U)
    {
        pxResp->u8SlaveAddr    = pxCtx->au8Buffer[MB_OFFSET_SLAVE];
        pxResp->eFuncCode      = (ModbusFunctionCode_t)(u8FC & ~(uint8_t)MB_FC_EXCEPTION_OFFSET);
        pxResp->bIsException   = true;
        pxResp->eExceptionCode = (ModbusException_t)pxCtx->au8Buffer[MB_OFFSET_DATA];
        pxResp->u8DataLen      = 0U;
        return MODBUS_RTU_PARSE_EXCEPTION;
    }

    /* Step 7b: Function code match (normal response) */
    if (u8FC != (uint8_t)eExpectedFC)
    {
        return MODBUS_RTU_PARSE_ERR_FC;
    }

    /* Step 8: Extract data */
    pxResp->u8SlaveAddr  = pxCtx->au8Buffer[MB_OFFSET_SLAVE];
    pxResp->eFuncCode    = eExpectedFC;
    pxResp->bIsException = false;
    pxResp->eExceptionCode = MB_EX_NONE;

    switch (eExpectedFC)
    {
        case MB_FC_READ_HOLDING_REGISTERS:
        case MB_FC_READ_INPUT_REGISTERS:
        case MB_FC_READ_COILS:
        case MB_FC_READ_DISCRETE_INPUTS:
        {
            /* [addr][FC][bytecount][data...][CRC] */
            u8DataByteCount = pxCtx->au8Buffer[MB_OFFSET_DATA];

            /* Sanity-check the declared byte count matches the frame */
            if (pxCtx->u16Count !=
                ((uint16_t)u8DataByteCount + (uint16_t)MB_RD_RESP_OVERHEAD))
            {
                return MODBUS_RTU_PARSE_ERR_DATA_LEN;
            }

            if (u8DataByteCount > MODBUS_RTU_MAX_DATA_BYTES)
            {
                return MODBUS_RTU_PARSE_ERR_DATA_LEN;
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
            /* Echo: [addr][FC][reg_hi][reg_lo][val_hi][val_lo][CRC] */
            (void)memcpy(pxResp->au8Data,
                         &pxCtx->au8Buffer[MB_OFFSET_DATA],
                         4U);
            pxResp->u8DataLen = 4U;
            break;
        }

        case MB_FC_WRITE_MULTIPLE_REGISTERS:
        case MB_FC_WRITE_MULTIPLE_COILS:
        {
            /* Echo: [addr][FC][start_hi][start_lo][qty_hi][qty_lo][CRC] */
            (void)memcpy(pxResp->au8Data,
                         &pxCtx->au8Buffer[MB_OFFSET_DATA],
                         4U);
            pxResp->u8DataLen = 4U;
            break;
        }

        default:
            return MODBUS_RTU_PARSE_ERR_FC;
    }

    return MODBUS_RTU_PARSE_OK;
}

/* =========================================================================
 * Expected response length
 * ========================================================================= */
uint16_t ModbusRtu_GetExpectedResponseLen(ModbusFunctionCode_t eFuncCode,
                                           uint8_t              u8RegCount)
{
    uint16_t u16Len = 0U;

    switch (eFuncCode)
    {
        case MB_FC_READ_HOLDING_REGISTERS:
        case MB_FC_READ_INPUT_REGISTERS:
            /* addr(1)+FC(1)+bytecount(1)+data(2×n)+CRC(2) */
            u16Len = (uint16_t)5U + (uint16_t)((uint16_t)u8RegCount * 2U);
            break;

        case MB_FC_READ_COILS:
        case MB_FC_READ_DISCRETE_INPUTS:
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
            u16Len = 0U;
            break;
    }

    return u16Len;
}

/* =========================================================================
 * Debug helper
 * ========================================================================= */
const char *ModbusRtu_ParseResultStr(ModbusRtu_ParseResult_t eResult)
{
    switch (eResult)
    {
        case MODBUS_RTU_PARSE_OK:             return "OK";
        case MODBUS_RTU_PARSE_ERR_NULL:       return "ERR_NULL";
        case MODBUS_RTU_PARSE_ERR_OVERFLOW:   return "ERR_OVERFLOW";
        case MODBUS_RTU_PARSE_ERR_TOO_SHORT:  return "ERR_TOO_SHORT";
        case MODBUS_RTU_PARSE_ERR_LENGTH:     return "ERR_LENGTH (frame split!)";
        case MODBUS_RTU_PARSE_ERR_CRC:        return "ERR_CRC";
        case MODBUS_RTU_PARSE_ERR_SLAVE_ADDR: return "ERR_SLAVE_ADDR";
        case MODBUS_RTU_PARSE_ERR_FC:         return "ERR_FC";
        case MODBUS_RTU_PARSE_ERR_DATA_LEN:   return "ERR_DATA_LEN";
        case MODBUS_RTU_PARSE_EXCEPTION:      return "EXCEPTION (valid)";
        default:                              return "UNKNOWN";
    }
}

/*******************************************************************************
 * End of File
 *******************************************************************************/