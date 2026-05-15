/*******************************************************************************
 * @file    uart_port.h
 * @brief   UART / RS485 Hardware Abstraction Layer — Header
 *
 * BUG FIXES vs previous revision
 * ────────────────────────────────
 * FIX-R1  FlushRx() loop logic was INVERTED — only drained 1 echo byte.
 *         Corrected to properly drain the SERCOM FIFO.
 *
 * FIX-R2  bTxActive cleared AFTER FlushRx() drain, not in SetRxMode().
 *         Closes the race window where the ISR accepted echo bytes as data.
 *
 * FIX-R3  TCC period register written in TICKS not microseconds.
 *         New macro UART_PORT_TCC_CLOCK_HZ + UART_PORT_T35_TICKS().
 *         *** YOU MUST SET UART_PORT_TCC_CLOCK_HZ to your actual TCC clock ***
 *         Wrong value = timer fires mid-response = frame split = ERR_LENGTH.
 *
 * FIX-R4  UART_PORT_FIFO_DEPTH added for portable drain loop sizing.
 *
 * @company BACANCY SYSTEMS PVT. LTD.
 *******************************************************************************/
#ifndef UART_PORT_H
#define UART_PORT_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/* =========================================================================
 * Channel identifiers
 * ========================================================================= */
typedef enum
{
    UART_PORT_CH1 = 0U,   /* SERCOM8 — RS485 Bus 1 */
    UART_PORT_CH2 = 1U,   /* SERCOM9 — RS485 Bus 2 */
    UART_PORT_CH_MAX
} UartPort_Channel_t;

typedef enum
{
    UART_PORT_OK          = 0U,
    UART_PORT_ERR_PARAM   = 1U,
    UART_PORT_ERR_BUSY    = 2U,
    UART_PORT_ERR_HW      = 3U,
    UART_PORT_ERR_OVERRUN = 4U
} UartPort_Status_t;

/* =========================================================================
 * Baud rate configuration
 * ========================================================================= */
#define UART_PORT_BAUD_CH1      (9600UL)    /* ← SET to your actual baud rate */
#define UART_PORT_BAUD_CH2      (9600UL)    /* ← SET to your actual baud rate */

/* =========================================================================
 * FIX-R3: TCC clock frequency — CRITICAL, MUST BE SET CORRECTLY
 * ─────────────────────────────────────────────────────────────────────────
 * Set this to the actual clock frequency driving TCC0/TCC1 in Hz.
 * Check your Harmony 3 / ASF4 clock configuration:
 *   - MCC / ATSTART clock configurator → TCC0/TCC1 input clock
 *   - Or read MCLK->APBBMASK and GCLK->PCHCTRL[TCC0_GCLK_ID]
 *
 * Common values:
 *   1,000,000  (1  MHz) — TCC fed from a 1 MHz GCLK (timer tick = 1 µs)
 *   8,000,000  (8  MHz)
 *  48,000,000  (48 MHz) — TCC fed directly from DFLL48M / USB PLL
 * 120,000,000  (120 MHz) — unlikely for TCC; prescaler usually applied
 *
 * *** If you set this wrong, UART_PORT_T35_TICKS() produces the wrong value
 *     and the timer fires either too early (frame split) or too late. ***
 *
 * Example: if your TCC0 GCLK = 48 MHz and no prescaler:
 *   #define UART_PORT_TCC_CLOCK_HZ   (48000000UL)
 *   T35 ticks at 9600 baud = 4011 µs × 48 = 192,528 ticks
 *
 * Example: if your TCC0 GCLK = 1 MHz:
 *   #define UART_PORT_TCC_CLOCK_HZ   (1000000UL)
 *   T35 ticks at 9600 baud = 4011 µs × 1 = 4,011 ticks
 * ========================================================================= */
#define UART_PORT_TCC_CLOCK_HZ    (1000000UL)   /* ← CHANGE THIS to your actual TCC clock */

/* =========================================================================
 * T3.5 timing macros
 * ─────────────────────────────────────────────────────────────────────────
 * UART_PORT_T35_US(baud):    inter-frame gap in MICROSECONDS (for logging).
 * UART_PORT_T35_TICKS(baud): inter-frame gap in TCC TICKS (for the register).
 *
 * Modbus spec clause 2.5.1.1:
 *   baud ≤ 19200: T3.5 = 3.5 × 11 / baud × 1e6  µs  (character-time based)
 *   baud > 19200: T3.5 = 1750 µs fixed minimum
 * ========================================================================= */
#define UART_PORT_T35_US(baud) \
    ((uint32_t)(((baud) <= 19200UL) \
        ? (((uint32_t)38500000UL / (uint32_t)(baud)) + 1U) \
        : 1750U))

/**
 * @brief  T3.5 in TCC register ticks.
 *         ticks = T3.5_us * (TCC_clock_Hz / 1_000_000)
 *         = T3.5_us * TCC_clock_Hz / 1_000_000
 *
 * Note: multiply order chosen to avoid 32-bit overflow for typical values.
 *       At 9600 baud, T35_us=4011.  Max TCC clock before overflow:
 *         4011 × TCC_Hz < 2^32  →  TCC_Hz < ~1.07 GHz  (safe for any real MCU)
 */
#define UART_PORT_T35_TICKS(baud) \
    ((uint32_t)( UART_PORT_T35_US(baud) * ((uint32_t)(UART_PORT_TCC_CLOCK_HZ) / 1000000UL) ))

/**
 * @brief  Minimum DE/RE hold time after last stop bit leaves bus (µs).
 *         500 µs is safe for any baud rate up to 115200.
 */
#define UART_PORT_TX_DE_HOLD_US    (20U)

/**
 * @brief  SAMD5x/E5x SERCOM USART hardware FIFO depth in bytes.
 *         Used to size the drain loop in FlushRx().
 *         SAMD5x = 2; adjust if your SERCOM variant differs.
 */
#define UART_PORT_FIFO_DEPTH       (2U)

/* =========================================================================
 * Callback prototypes
 * ========================================================================= */

/** Called from ISR context for every received byte. Must be ISR-safe. */
typedef void (*UartPort_RxByteCallback_t)(UartPort_Channel_t eChannel,
                                           uint8_t            u8Byte);

/** Called from timer-ISR context on T3.5 silence — frame boundary. */
typedef void (*UartPort_FrameEndCallback_t)(UartPort_Channel_t eChannel);

#define UART_PORT_CHANNEL_COUNT    ((uint8_t)UART_PORT_CH_MAX)

/* =========================================================================
 * Public API
 * ========================================================================= */

/**
 * @brief  Initialise a channel.
 *         Registers SERCOM RX + timer callbacks, arms first Read(),
 *         sets TCC period to T3.5 in TICKS (FIX-R3), puts bus in RX mode.
 */
UartPort_Status_t UartPort_Init(UartPort_Channel_t          eChannel,
                                 UartPort_RxByteCallback_t   pfnRxByte,
                                 UartPort_FrameEndCallback_t pfnFrameEnd);

/**
 * @brief  Transmit a byte block (task context, not ISR).
 *
 * Sequence:
 *   1. Assert DE  (TX mode, bTxActive=true)
 *   2. SERCOM Write + spin on TransmitComplete
 *   3. Hold DE for UART_PORT_TX_DE_HOLD_US µs
 *   4. De-assert DE — bTxActive stays TRUE (FIX-R2)
 *   5. FlushRx — drain FIFO, then clear bTxActive, then re-arm (FIX-R1+R2)
 */
UartPort_Status_t UartPort_Transmit(UartPort_Channel_t  eChannel,
                                     const uint8_t      *pu8Data,
                                     uint16_t            u16Length);

/** Assert DE high — RS485 TX direction. Sets bTxActive=true. */
void UartPort_SetTxMode(UartPort_Channel_t eChannel);

/**
 * @brief  De-assert DE — RS485 RX direction.
 * @note   FIX-R2: Does NOT clear bTxActive. Call UartPort_FlushRx() next,
 *         which clears bTxActive after draining the echo bytes.
 */
void UartPort_SetRxMode(UartPort_Channel_t eChannel);

/**
 * @brief  Drain SERCOM RX FIFO of echo bytes and re-arm for response.
 * @note   FIX-R1: Corrected loop logic — drains until FIFO is genuinely empty.
 *         FIX-R2: Clears bTxActive AFTER drain, not before.
 *         Must be called immediately after UartPort_SetRxMode().
 */
void UartPort_FlushRx(UartPort_Channel_t eChannel);

/** Re-arm the SERCOM Read() for the next incoming byte. */
void UartPort_ArmRx(UartPort_Channel_t eChannel);

/** Return T3.5 timeout in µs for a channel (for diagnostics/logging). */
uint32_t UartPort_GetT35Us(UartPort_Channel_t eChannel);

#endif /* UART_PORT_H */
/*******************************************************************************
 * End of File
 *******************************************************************************/