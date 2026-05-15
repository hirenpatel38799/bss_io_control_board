/*******************************************************************************
 * @file    modbus_crc.c
 * @brief   Modbus CRC-16 Calculation Module - Implementation
 * @details Implements CRC-16/ARC as required by the Modbus RTU specification.
 *          Algorithm: bitwise, no lookup table (ROM-efficient for MCUs).
 *          Polynomial: 0xA001 (reflected / LSB-first form of 0x8005).
 *
 * @company BACANCY SYSTEMS PVT. LTD.
 *******************************************************************************/

/* =========================================================================
 * Included Files
 * ========================================================================= */
#include "modbus_crc.h"
#include "definitions.h"    /* SYS_CONSOLE_PRINT */
/* =========================================================================
 * Private Macro Definitions
 * ========================================================================= */
#define CRC_BITS_PER_BYTE   (8U)

/* =========================================================================
 * Public Function Definitions
 * ========================================================================= */

/**
 * @brief  Compute CRC-16 (Modbus) over a data buffer.
 */
uint16_t Modbus_CRC16_Compute(const uint8_t *pu8Data, uint16_t u16Length)
{
    uint16_t u16CRC  = MODBUS_CRC_INIT_VALUE;
    uint16_t u16Pos;
    uint8_t  u8Bit;

    /* MISRA C:2012 Rule 14.5 — guard against NULL/zero-length input */
    if ((pu8Data == NULL) || (u16Length == 0U))
    {
        return 0U;
    }

    for (u16Pos = 0U; u16Pos < u16Length; u16Pos++)
    {
        u16CRC ^= (uint16_t)pu8Data[u16Pos];   /* XOR current byte into low byte of CRC */

        for (u8Bit = 0U; u8Bit < CRC_BITS_PER_BYTE; u8Bit++)
        {
            if ((u16CRC & 0x0001U) != 0U)       /* If LSB is set */
            {
                u16CRC >>= 1U;
                u16CRC  ^= MODBUS_CRC16_POLYNOMIAL;
            }
            else
            {
                u16CRC >>= 1U;
            }
        }
    }

    return u16CRC;
}

/**
 * @brief  Validate the CRC of a received Modbus RTU frame.
 */
bool Modbus_CRC16_Validate(const uint8_t *pu8Frame, uint16_t u16Length)
{
    uint16_t u16Computed;
    uint16_t u16Received;

    if ((pu8Frame == NULL) || (u16Length < MODBUS_MIN_FRAME_LEN))
    {
        return false;
    }

    /* Compute CRC over all bytes except the trailing CRC pair */
    u16Computed = Modbus_CRC16_Compute(pu8Frame, u16Length - 2U);

    /* Extract received CRC — Modbus RTU: low byte first */
    u16Received = (uint16_t)pu8Frame[u16Length - 2U]
                | ((uint16_t)pu8Frame[u16Length - 1U] << 8U);

    // SYS_CONSOLE_PRINT("Computed CRC=0x%04X, Received CRC=0x%04X\r\n", (unsigned)u16Computed, (unsigned)u16Received);
    return (u16Computed == u16Received);
}

/**
 * @brief  Append a computed CRC to the end of a Modbus frame buffer.
 */
uint16_t Modbus_CRC16_Append(uint8_t *pu8Frame, uint16_t u16DataLen)
{
    uint16_t u16CRC;

    if ((pu8Frame == NULL) || (u16DataLen == 0U))
    {
        return 0U;
    }

    u16CRC = Modbus_CRC16_Compute(pu8Frame, u16DataLen);

    /* Modbus RTU byte order: low byte first */
    pu8Frame[u16DataLen]      = (uint8_t)(u16CRC & 0x00FFU);
    pu8Frame[u16DataLen + 1U] = (uint8_t)(u16CRC >> 8U);

    return (u16DataLen + 2U);   /* Total frame length with CRC */
}

/*******************************************************************************
 * End of File
 *******************************************************************************/
