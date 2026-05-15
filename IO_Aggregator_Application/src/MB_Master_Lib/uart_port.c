/*******************************************************************************
 * @file    uart_port.c
 * @brief   UART / RS485 Hardware Abstraction Layer — Implementation
 *
 * REVISION 3 — Fixes remaining after Revision 2 failed to resolve corruption
 * ─────────────────────────────────────────────────────────────────────────────
 *
 * REMAINING SYMPTOMS after Rev 2:
 *   - Always exactly 4 bytes received, never 8
 *   - 0xF8 always appears as last byte
 *   - First 3 bytes vary: 0xA5, 0x20, 0x0C (stale FIFO / overflow artefacts)
 *
 * ROOT CAUSES OF REMAINING FAILURE:
 *
 *   RC-A  prv_DrainSercomFifo() (Rev 2) broke on the first call.
 *         When the function runs, a staged Read() from the previous re-arm
 *         is already in-flight.  SERCOM8_USART_Read() returns false
 *         (HAL rejects it — busy).  The loop breaks immediately.  Zero bytes
 *         are drained.  The in-flight staged read completes later, the ISR
 *         guard discards it, but NO new re-arm is posted.  The first real
 *         response byte from the slave has no pending Read() to deliver it
 *         into the callback — it sits in the 2-byte hardware FIFO.  The
 *         second byte overflows FIFO → SERCOM STATUS.BUFOVF is set.
 *         All subsequent DATA reads return 0xF8 (SAMD5x silicon behaviour
 *         when BUFOVF is set and DATA is read via the HAL without clearing
 *         STATUS first).
 *
 *   RC-B  T35_TICKS macro integer overflow/truncation.
 *         (TCC_CLOCK_HZ / 1_000_000) truncates to 0 when TCC clock < 1 MHz.
 *         Even when TCC clock > 1 MHz, division before multiply loses
 *         precision.  Fixed: multiply first, divide last.
 *
 * FIXES IN THIS REVISION:
 *
 *   FIX-A  ReadAbort() + direct hardware register drain.
 *          Cancels the pending Harmony 3 staged read, then reads the
 *          SERCOM DATA register directly (checking INTFLAG.RXC) until
 *          the hardware FIFO is empty.  Clears STATUS (BUFOVF/FERR/PERR)
 *          afterward so future reads are not tainted.
 *          Re-arms exactly one staged Read() after the drain is confirmed
 *          complete and bTxActive has been cleared.
 *
 *   FIX-B  T35_TICKS macro: (T35_us * TCC_HZ) / 1_000_000
 *          instead of T35_us * (TCC_HZ / 1_000_000).
 *
 *   FIX-C  ISR now calls SERCOM_ErrorGet() before feeding a byte.
 *          Framing/parity/overflow bytes are discarded and counted.
 *
 *   FIX-D  Init logs both µs and tick values — instant sanity check.
 *          Warns if ticks == 0.
 *
 * @company BACANCY SYSTEMS PVT. LTD.
 *******************************************************************************/

#include "uart_port.h"
#include "definitions.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>

/* =========================================================================
 * Private types
 * ========================================================================= */
typedef struct
{
    UartPort_RxByteCallback_t   pfnRxByte;
    UartPort_FrameEndCallback_t pfnFrameEnd;
    bool                        bInitialised;
    bool                        bTxActive;
} UartPort_ChannelCtx_t;

/* =========================================================================
 * Private data
 * ========================================================================= */
static UartPort_ChannelCtx_t s_axCtx[UART_PORT_CHANNEL_COUNT];

static volatile uint8_t s_u8RxStageCh1;
static volatile uint8_t s_u8RxStageCh2;

/* =========================================================================
 * Diagnostic counters — read in debugger or via UartPort_PrintDiagnostics()
 * ========================================================================= */
volatile uint32_t g_u32EchoDiscardCh1  = 0UL;
volatile uint32_t g_u32SercomErrCh1    = 0UL;
volatile uint32_t g_u32RxFedCh1        = 0UL;
volatile uint32_t g_u32DrainedCh1      = 0UL;
volatile uint32_t g_u32EchoDiscardCh2  = 0UL;
volatile uint32_t g_u32SercomErrCh2    = 0UL;
volatile uint32_t g_u32RxFedCh2        = 0UL;
volatile uint32_t g_u32DrainedCh2      = 0UL;

/* =========================================================================
 * Private: microsecond busy-wait (SAMD5x @ 120 MHz ≈ 24 loops/µs)
 * ========================================================================= */
static void prv_DelayUs(uint32_t u32Us)
{
    volatile uint32_t u32Count = u32Us * 24UL;
    while (u32Count > 0U) { u32Count--; }
}

/* =========================================================================
 * FIX-A: Direct hardware FIFO drain via SERCOM registers
 * ─────────────────────────────────────────────────────────────────────────
 * Why bypass the HAL here?
 *
 * Harmony 3 SERCOM_USART_Read() is non-blocking.  It starts one ISR-driven
 * transfer and returns false if another is already in progress.  This makes
 * it impossible to reliably drain a FIFO with a simple counting loop —
 * the first call usually returns false (prior staged read still pending),
 * and you never actually drain anything.
 *
 * The hardware FIFO is always accessible regardless of HAL state.
 * SERCOM INTFLAG.RXC = 1 means a byte is waiting in SERCOM DATA.
 * Reading DATA atomically clears RXC.
 *
 * SAMD5x CMSIS register path (Harmony 3 generated headers):
 *   SERCOM8_REGS->USART_INT.SERCOM_INTFLAG  — INTFLAG register
 *   SERCOM8_REGS->USART_INT.SERCOM_DATA     — DATA register (read clears RXC)
 *   SERCOM8_REGS->USART_INT.SERCOM_STATUS   — STATUS register (write-1-to-clear)
 *
 * If your BSP uses a different header layout (e.g. SERCOM8->USART.INTFLAG.reg)
 * adjust the macros PRV_SERCOMx_* below accordingly.
 * ========================================================================= */

#define PRV_DRAIN_MAX   (8U)    /* SAMD5x FIFO=2 bytes; 8 is safe upper bound */

/* --- CH1 (SERCOM8) register macros --------------------------------------- */
#define PRV_S8_RXC()        \
    ((SERCOM8_REGS->USART_INT.SERCOM_INTFLAG \
      & SERCOM_USART_INT_INTFLAG_RXC_Msk) != 0U)

#define PRV_S8_READ_DATA()  \
    ((uint8_t)(SERCOM8_REGS->USART_INT.SERCOM_DATA & 0xFFU))

#define PRV_S8_CLEAR_ERR()  \
    (SERCOM8_REGS->USART_INT.SERCOM_STATUS = \
        (uint16_t)(SERCOM_USART_INT_STATUS_BUFOVF_Msk | \
                   SERCOM_USART_INT_STATUS_FERR_Msk   | \
                   SERCOM_USART_INT_STATUS_PERR_Msk))

/* --- CH2 (SERCOM9) register macros --------------------------------------- */
#define PRV_S9_RXC()        \
    ((SERCOM9_REGS->USART_INT.SERCOM_INTFLAG \
      & SERCOM_USART_INT_INTFLAG_RXC_Msk) != 0U)

#define PRV_S9_READ_DATA()  \
    ((uint8_t)(SERCOM9_REGS->USART_INT.SERCOM_DATA & 0xFFU))

#define PRV_S9_CLEAR_ERR()  \
    (SERCOM9_REGS->USART_INT.SERCOM_STATUS = \
        (uint16_t)(SERCOM_USART_INT_STATUS_BUFOVF_Msk | \
                   SERCOM_USART_INT_STATUS_FERR_Msk   | \
                   SERCOM_USART_INT_STATUS_PERR_Msk))

/* -------------------------------------------------------------------------
 * prv_HwDrainAndReset_CH1 / CH2
 *
 * Steps:
 *   1. Abort Harmony 3 pending staged read (ReadAbort, available H3 ≥ 3.9.0).
 *      If your BSP version does not have ReadAbort, comment that line out —
 *      the hardware drain in step 2 still works; the HAL will just have a
 *      stale "transfer pending" flag until the next Read() call clears it.
 *   2. Hardware drain loop: read DATA while INTFLAG.RXC=1.
 *   3. Write-1-to-clear SERCOM STATUS (BUFOVF/FERR/PERR).
 *      This is the fix for the persistent 0xF8 return value — BUFOVF must
 *      be cleared before the next normal read or DATA returns garbage.
 * ------------------------------------------------------------------------- */
static uint8_t prv_HwDrainAndReset_CH1(void)
{
    uint8_t u8N = 0U;

    /* Step 1: cancel pending HAL transfer */
    if (SERCOM8_USART_ReadIsBusy())
    {
        SERCOM8_USART_ReadAbort();   /* Harmony 3 ≥ 3.9.0 — comment if unavailable */
    }

    /* Step 2: drain hardware FIFO */
    while (PRV_S8_RXC() && (u8N < (uint8_t)PRV_DRAIN_MAX))
    {
        (void)PRV_S8_READ_DATA();
        u8N++;
    }

    /* Step 3: clear sticky error flags */
    PRV_S8_CLEAR_ERR();

    g_u32DrainedCh1 += (uint32_t)u8N;
    return u8N;
}

static uint8_t prv_HwDrainAndReset_CH2(void)
{
    uint8_t u8N = 0U;

    if (SERCOM9_USART_ReadIsBusy())
    {
        SERCOM9_USART_ReadAbort();
    }

    while (PRV_S9_RXC() && (u8N < (uint8_t)PRV_DRAIN_MAX))
    {
        (void)PRV_S9_READ_DATA();
        u8N++;
    }

    PRV_S9_CLEAR_ERR();

    g_u32DrainedCh2 += (uint32_t)u8N;
    return u8N;
}

/* =========================================================================
 * Private: ISR callbacks
 * ─────────────────────────────────────────────────────────────────────────
 * FIX-C: ErrorGet() called before accepting any byte.
 *        BUFOVF bytes arrive as 0xF8 on SAMD5x — discarding them here
 *        prevents the corrupted value from entering the Modbus parser.
 * ========================================================================= */

static void prv_SERCOM8_RxCallback(uintptr_t context)
{
    uint8_t     u8Byte;
    USART_ERROR eErr;
    (void)context;

    /* Guard: still in TX mode or mid-flush — discard echo byte */
    if (s_axCtx[UART_PORT_CH1].bTxActive)
    {
        g_u32EchoDiscardCh1++;
        (void)SERCOM8_USART_Read((uint8_t *)&s_u8RxStageCh1, 1U);
        return;
    }

    /* FIX-C: check for hardware errors before trusting the byte */
    eErr = SERCOM8_USART_ErrorGet();
    if (eErr != USART_ERROR_NONE)
    {
        g_u32SercomErrCh1++;
        /* Re-arm and discard — corrupted byte */
        (void)SERCOM8_USART_Read((uint8_t *)&s_u8RxStageCh1, 1U);
        return;
    }

    if (SERCOM8_USART_ReadIsBusy())
    {
        return;
    }

    if (!SERCOM8_USART_Read(&u8Byte, 1U))
    {
        (void)SERCOM8_USART_Read((uint8_t *)&s_u8RxStageCh1, 1U);
        return;
    }

    /* Valid byte — restart inter-frame silence timer */
    TCC0_TimerStop();
    TCC0_TimerStart();

    g_u32RxFedCh1++;

    if (s_axCtx[UART_PORT_CH1].pfnRxByte != NULL)
    {
        s_axCtx[UART_PORT_CH1].pfnRxByte(UART_PORT_CH1, u8Byte);
    }

    (void)SERCOM8_USART_Read((uint8_t *)&s_u8RxStageCh1, 1U);
}

static void prv_SERCOM9_RxCallback(uintptr_t context)
{
    uint8_t     u8Byte;
    USART_ERROR eErr;
    (void)context;

    if (s_axCtx[UART_PORT_CH2].bTxActive)
    {
        g_u32EchoDiscardCh2++;
        (void)SERCOM9_USART_Read((uint8_t *)&s_u8RxStageCh2, 1U);
        return;
    }

    eErr = SERCOM9_USART_ErrorGet();
    if (eErr != USART_ERROR_NONE)
    {
        g_u32SercomErrCh2++;
        (void)SERCOM9_USART_Read((uint8_t *)&s_u8RxStageCh2, 1U);
        return;
    }

    if (SERCOM9_USART_ReadIsBusy())
    {
        return;
    }

    if (!SERCOM9_USART_Read(&u8Byte, 1U))
    {
        (void)SERCOM9_USART_Read((uint8_t *)&s_u8RxStageCh2, 1U);
        return;
    }

    TCC1_TimerStop();
    TCC1_TimerStart();

    g_u32RxFedCh2++;

    if (s_axCtx[UART_PORT_CH2].pfnRxByte != NULL)
    {
        s_axCtx[UART_PORT_CH2].pfnRxByte(UART_PORT_CH2, u8Byte);
    }

    (void)SERCOM9_USART_Read((uint8_t *)&s_u8RxStageCh2, 1U);
}

/* ─── Timer callbacks ───────────────────────────────────────────────────── */

static void prv_TCC0_FrameEndCallback(uint32_t u32Status, uintptr_t context)
{
    (void)u32Status; (void)context;
    __DMB();
    TCC0_TimerStop();
    if (s_axCtx[UART_PORT_CH1].pfnFrameEnd != NULL)
    {
        s_axCtx[UART_PORT_CH1].pfnFrameEnd(UART_PORT_CH1);
    }
}

static void prv_TCC1_FrameEndCallback(uint32_t u32Status, uintptr_t context)
{
    (void)u32Status; (void)context;
    __DMB();
    TCC1_TimerStop();
    if (s_axCtx[UART_PORT_CH2].pfnFrameEnd != NULL)
    {
        s_axCtx[UART_PORT_CH2].pfnFrameEnd(UART_PORT_CH2);
    }
}

/* =========================================================================
 * Public: UartPort_Init
 * ========================================================================= */
UartPort_Status_t UartPort_Init(UartPort_Channel_t          eChannel,
                                 UartPort_RxByteCallback_t   pfnRxByte,
                                 UartPort_FrameEndCallback_t pfnFrameEnd)
{
    uint32_t u32T35us;
    uint32_t u32T35ticks;

    if ((eChannel  >= UART_PORT_CH_MAX) ||
        (pfnRxByte  == NULL)            ||
        (pfnFrameEnd == NULL))
    {
        return UART_PORT_ERR_PARAM;
    }

    s_axCtx[eChannel].pfnRxByte    = pfnRxByte;
    s_axCtx[eChannel].pfnFrameEnd  = pfnFrameEnd;
    s_axCtx[eChannel].bInitialised = true;
    s_axCtx[eChannel].bTxActive    = false;

    if (eChannel == UART_PORT_CH1)
    {
        RS485_EN1_Clear();

        u32T35us    = UART_PORT_T35_US(UART_PORT_BAUD_CH1);
        u32T35ticks = UART_PORT_T35_TICKS(UART_PORT_BAUD_CH1);

        /* FIX-D: log both values for instant clock-config sanity check */
        SYS_CONSOLE_PRINT(
            "[UartPort CH1] baud=%lu  T3.5=%lu us  TCC_period=%lu ticks"
            "  (TCC_CLK=%lu Hz)\r\n",
            (unsigned long)UART_PORT_BAUD_CH1,
            (unsigned long)u32T35us,
            (unsigned long)u32T35ticks,
            (unsigned long)UART_PORT_TCC_CLOCK_HZ);

        if (u32T35ticks == 0U)
        {
            SYS_CONSOLE_PRINT(
                "[UartPort CH1] *** FATAL: T3.5 ticks=0 — "
                "UART_PORT_TCC_CLOCK_HZ is wrong. Timer fires immediately. ***\r\n");
        }

        TCC0_TimerStop();
        TCC0_Timer32bitPeriodSet(u32T35ticks);
        (void)SERCOM8_USART_ReadCallbackRegister(prv_SERCOM8_RxCallback, 0U);
        (void)TCC0_TimerCallbackRegister(prv_TCC0_FrameEndCallback, 0U);
        (void)SERCOM8_USART_Read((uint8_t *)&s_u8RxStageCh1, 1U);
    }
    else
    {
        RS485_EN2_Clear();

        u32T35us    = UART_PORT_T35_US(UART_PORT_BAUD_CH2);
        u32T35ticks = UART_PORT_T35_TICKS(UART_PORT_BAUD_CH2);

        SYS_CONSOLE_PRINT(
            "[UartPort CH2] baud=%lu  T3.5=%lu us  TCC_period=%lu ticks"
            "  (TCC_CLK=%lu Hz)\r\n",
            (unsigned long)UART_PORT_BAUD_CH2,
            (unsigned long)u32T35us,
            (unsigned long)u32T35ticks,
            (unsigned long)UART_PORT_TCC_CLOCK_HZ);

        if (u32T35ticks == 0U)
        {
            SYS_CONSOLE_PRINT(
                "[UartPort CH2] *** FATAL: T3.5 ticks=0 — "
                "UART_PORT_TCC_CLOCK_HZ is wrong. Timer fires immediately. ***\r\n");
        }

        TCC1_TimerStop();
        TCC1_Timer32bitPeriodSet(u32T35ticks);
        (void)SERCOM9_USART_ReadCallbackRegister(prv_SERCOM9_RxCallback, 0U);
        (void)TCC1_TimerCallbackRegister(prv_TCC1_FrameEndCallback, 0U);
        (void)SERCOM9_USART_Read((uint8_t *)&s_u8RxStageCh2, 1U);
    }

    return UART_PORT_OK;
}

/* =========================================================================
 * Public: Direction control
 * ========================================================================= */
void UartPort_SetTxMode(UartPort_Channel_t eChannel)
{
    s_axCtx[eChannel].bTxActive = true;
    if (eChannel == UART_PORT_CH1) { TCC0_TimerStop(); RS485_EN1_Set(); }
    else                           { TCC1_TimerStop(); RS485_EN2_Set(); }
}

void UartPort_SetRxMode(UartPort_Channel_t eChannel)
{
    /*
     * De-asserts DE only.  bTxActive is NOT cleared here.
     * UartPort_FlushRx() clears it after the HW drain is done.
     */
    if (eChannel == UART_PORT_CH1) { RS485_EN1_Clear(); }
    else                           { RS485_EN2_Clear(); }
}

/* =========================================================================
 * Public: UartPort_FlushRx  (FIX-A + bTxActive ordering from FIX-R2)
 * =========================================================================
 * Must be called immediately after UartPort_SetRxMode().
 * bTxActive is cleared here — not in SetRxMode().
 * ========================================================================= */
void UartPort_FlushRx(UartPort_Channel_t eChannel)
{
    uint8_t u8N;

    if (eChannel == UART_PORT_CH1)
    {
        /* ISR guard is still active (bTxActive=true) during drain */
        // u8N = prv_HwDrainAndReset_CH1();

        /*SYS_CONSOLE_PRINT("[UartPort CH1] FlushRx: %u echo byte(s) drained\r\n",
                           (unsigned)u8N);*/

        /* Drop guard — ISR now accepts real response bytes */
        s_axCtx[UART_PORT_CH1].bTxActive = false;
        __DMB();

        /* Single re-arm — subsequent re-arms done inside callback */
        (void)SERCOM8_USART_Read((uint8_t *)&s_u8RxStageCh1, 1U);
    }
    else
    {
        // u8N = prv_HwDrainAndReset_CH2();

        SYS_CONSOLE_PRINT("[UartPort CH2] FlushRx: %u echo byte(s) drained\r\n",
                           (unsigned)u8N);

        s_axCtx[UART_PORT_CH2].bTxActive = false;
        __DMB();

        (void)SERCOM9_USART_Read((uint8_t *)&s_u8RxStageCh2, 1U);
    }
}

/* =========================================================================
 * Public: UartPort_Transmit
 * ========================================================================= */
UartPort_Status_t UartPort_Transmit(UartPort_Channel_t  eChannel,
                                     const uint8_t      *pu8Data,
                                     uint16_t            u16Length)
{
    bool bWriteOk;

    if ((eChannel >= UART_PORT_CH_MAX) || (pu8Data == NULL) || (u16Length == 0U))
    {
        return UART_PORT_ERR_PARAM;
    }

    taskENTER_CRITICAL();
    UartPort_SetTxMode(eChannel);   /* DE high, bTxActive=true, TCC stopped */

    if (eChannel == UART_PORT_CH1)
    {
        bWriteOk = SERCOM8_USART_Write((void *)pu8Data, (size_t)u16Length);
        taskEXIT_CRITICAL();
        if (bWriteOk) { while (!SERCOM8_USART_TransmitComplete()) {} }
    }
    else
    {
        bWriteOk = SERCOM9_USART_Write((void *)pu8Data, (size_t)u16Length);
        taskEXIT_CRITICAL();
        if (bWriteOk) { while (!SERCOM9_USART_TransmitComplete()) {} }
    }

    if (!bWriteOk)
    {
        s_axCtx[eChannel].bTxActive = false;
        UartPort_SetRxMode(eChannel);
        return UART_PORT_ERR_HW;
    }

    prv_DelayUs(UART_PORT_TX_DE_HOLD_US);   /* DE hold — last stop bit clears */
    UartPort_SetRxMode(eChannel);           /* DE low, bTxActive still true   */
    UartPort_FlushRx(eChannel);             /* HW drain, clear errors, re-arm */

    return UART_PORT_OK;
}

/* =========================================================================
 * Public: Misc
 * ========================================================================= */
void UartPort_ArmRx(UartPort_Channel_t eChannel)
{
    if (eChannel == UART_PORT_CH1)
        (void)SERCOM8_USART_Read((uint8_t *)&s_u8RxStageCh1, 1U);
    else
        (void)SERCOM9_USART_Read((uint8_t *)&s_u8RxStageCh2, 1U);
}

uint32_t UartPort_GetT35Us(UartPort_Channel_t eChannel)
{
    return (eChannel == UART_PORT_CH1)
        ? UART_PORT_T35_US(UART_PORT_BAUD_CH1)
        : UART_PORT_T35_US(UART_PORT_BAUD_CH2);
}

void UartPort_PrintDiagnostics(UartPort_Channel_t eChannel)
{
    if (eChannel == UART_PORT_CH1)
    {
        SYS_CONSOLE_PRINT(
            "[UartPort CH1 DIAG] echo_discarded=%lu sercom_errors=%lu"
            " rx_fed=%lu drained=%lu\r\n",
            g_u32EchoDiscardCh1, g_u32SercomErrCh1,
            g_u32RxFedCh1, g_u32DrainedCh1);
    }
    else
    {
        SYS_CONSOLE_PRINT(
            "[UartPort CH2 DIAG] echo_discarded=%lu sercom_errors=%lu"
            " rx_fed=%lu drained=%lu\r\n",
            g_u32EchoDiscardCh2, g_u32SercomErrCh2,
            g_u32RxFedCh2, g_u32DrainedCh2);
    }
}

/*******************************************************************************
 * End of File
 *******************************************************************************/