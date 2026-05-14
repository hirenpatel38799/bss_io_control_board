/*******************************************************************************
 * @file    modbus_crc.h
 * @brief   Modbus CRC-16 Calculation Module - Header
 * @details Provides CRC-16/ARC (Modbus) computation and validation APIs.
 *          Polynomial: 0xA001 (reflected form of 0x8005).
 *
 * @note    This module is intentionally dependency-free and portable.
 *          It can be used on any embedded target without modification.
 *
 * @company BACANCY SYSTEMS PVT. LTD.
 *******************************************************************************/

#ifndef MODBUS_CRC_H
#define MODBUS_CRC_H

/* =========================================================================
 * Included Files
 * ========================================================================= */
#include <stdio.h> 
#include <stdint.h>
#include <stdbool.h>

/* =========================================================================
 * Macro Definitions
 * ========================================================================= */

/** CRC seed value as per Modbus specification */
#define MODBUS_CRC_INIT_VALUE      (0xFFFFU)

/** Modbus CRC polynomial (reflected / LSB-first form) */
#define MODBUS_CRC16_POLYNOMIAL    (0xA001U)

/** Minimum valid Modbus PDU size: SlaveID + FC + 2 CRC bytes */
#define MODBUS_MIN_FRAME_LEN       (4U)

/* =========================================================================
 * Public API
 * ========================================================================= */

/**
 * @brief  Compute CRC-16 (Modbus) over a data buffer.
 *
 * @param[in]  pu8Data    Pointer to input data buffer. Must not be NULL.
 * @param[in]  u16Length  Number of bytes to process.
 *
 * @return  Computed 16-bit CRC value.
 *          Returns 0U if pu8Data is NULL or u16Length is 0.
 *
 * @note   CRC is returned as a 16-bit value.
 *         For Modbus RTU frames, append low byte first, then high byte:
 *         frame[n]   = (uint8_t)(crc & 0xFFU);
 *         frame[n+1] = (uint8_t)(crc >> 8U);
 */
uint16_t Modbus_CRC16_Compute(const uint8_t *pu8Data, uint16_t u16Length);

/**
 * @brief  Validate the CRC of a received Modbus RTU frame.
 *
 * @details Recomputes CRC over bytes [0 .. u16Length-3] and compares against
 *          the last two bytes of the frame (low byte first, per Modbus spec).
 *
 * @param[in]  pu8Frame   Pointer to the full received frame (including CRC bytes).
 * @param[in]  u16Length  Total frame length in bytes (data + 2 CRC bytes).
 *
 * @return  true   CRC is valid.
 *          false  CRC mismatch, NULL pointer, or frame too short.
 */
bool Modbus_CRC16_Validate(const uint8_t *pu8Frame, uint16_t u16Length);

/**
 * @brief  Append a computed CRC to the end of a Modbus frame buffer.
 *
 * @details Computes CRC over bytes [0 .. u16DataLen-1] and writes the result
 *          into pu8Frame[u16DataLen] (low byte) and pu8Frame[u16DataLen+1] (high byte).
 *          Caller must ensure the buffer has at least (u16DataLen + 2) bytes allocated.
 *
 * @param[in,out]  pu8Frame    Buffer containing frame data; CRC is appended in place.
 * @param[in]      u16DataLen  Number of data bytes (excluding CRC).
 *
 * @return  Total frame length including appended CRC (u16DataLen + 2).
 *          Returns 0U if pu8Frame is NULL or u16DataLen is 0.
 */
uint16_t Modbus_CRC16_Append(uint8_t *pu8Frame, uint16_t u16DataLen);

#endif /* MODBUS_CRC_H */

/*******************************************************************************
 * End of File
 *******************************************************************************/
