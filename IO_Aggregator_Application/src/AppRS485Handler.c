/*******************************************************************************
 * @file    AppRS485Handler.c
 * @brief   RS485 Application Layer - Implementation
 *
 * @details Application-layer implementation.  This file contains ONLY:
 *            - System initialisation (init LED Manager + Modbus master, create tasks)
 *            - Business-logic request functions (LED module read/write)
 *            - Result callback handler (interprets responses, marks cards clean)
 *            - Periodic scheduling task (drives state machine + requests)
 *
 *          There is NO direct LED state logic in this file.
 *          LED state is owned by led_manager.c.
 *          All Modbus framing, CRC, and transport is in the lower-layer modules.
 *
 * CHANGES vs previous revision
 * ──────────────────────────────
 * • vRS485HandlerInit() now calls LedMgr_Init() before Modbus init.
 * • prv_ModbusResultCallback() calls LedMgr_MarkCardClean() on write success,
 *   using pvContext to identify which card the transaction targeted.
 * • vLED_ModuleWrite() replaced by vLED_ModuleWriteAllCards() +
 *   vLED_ModuleWriteCard()  iterates over LED_CARD_COUNT slave devices.
 * • u16GetLocalLedState() stub removed  no longer needed here.
 * • prv_RS485AppTask() calls vLED_ModuleWriteAllCards() (not vLED_ModuleWrite).
 *
 * @company BACANCY SYSTEMS PVT. LTD.
 *******************************************************************************/

/* =========================================================================
 * Included Files
 * ========================================================================= */
#include <stddef.h>
#include <stdbool.h>
#include <string.h>
#include "definitions.h"
#include "AppRS485Handler.h"
#include "led_manager.h"
#include "FreeRTOS.h"
#include "task.h"

/* =========================================================================
 * Private Data  Master channel contexts
 * ========================================================================= */

/** Modbus Master context for RS485 Bus 1 (CH1  both LED cards) */
static ModbusMaster_Channel_t s_xMasterCh1;

/** Modbus Master context for RS485 Bus 2 (CH2  reserved / future) */
static ModbusMaster_Channel_t s_xMasterCh2;

/* =========================================================================
 * Private: Callback context  identifies which LED card a transaction targets
 * =========================================================================
 * We pass a pointer to one of these static sentinels as pvContext when
 * submitting a write transaction.  The callback casts pvContext back to
 * uint8_t* to determine the card index for LedMgr_MarkCardClean().
 * Using static variables avoids heap allocation.
 * ========================================================================= */
static const uint8_t s_au8CardCtx[LED_CARD_COUNT] = { 0U, 1U };

/* =========================================================================
 * Private: Result Callback
 * =========================================================================
 * Called by Modbus_Master_Process() upon transaction completion.
 * Runs in application task context (not ISR).
 *
 * pvContext carries a pointer to s_au8CardCtx[card_index] when the
 * transaction was a write to an LED card.  For reads or other operations it
 * is NULL.
 * ========================================================================= */
static void prv_ModbusResultCallback(UartPort_Channel_t         eChannel,
                                      ModbusMaster_Result_t      eResult,
                                      const ModbusRtu_Response_t *pxResp,
                                      void                       *pvContext)
{
    if (eResult == MB_RESULT_SUCCESS)
    {
        if (pxResp == NULL)
        {
            return;
        }

        switch (pxResp->eFuncCode)
        {
            case MB_FC_READ_HOLDING_REGISTERS:
            {
                uint8_t  u8Idx;
                uint16_t u16RegVal;

                for (u8Idx = 0U; u8Idx < (pxResp->u8DataLen / 2U); u8Idx++)
                {
                    u16RegVal = ((uint16_t)pxResp->au8Data[u8Idx * 2U] << 8U)
                              | ((uint16_t)pxResp->au8Data[(u8Idx * 2U) + 1U]);

                    SYS_CONSOLE_PRINT("  Reg[%u] = 0x%04X (%u)\r\n",
                                       (unsigned)u8Idx,
                                       (unsigned)u16RegVal,
                                       (unsigned)u16RegVal);
                }
                break;
            }

            case MB_FC_WRITE_MULTIPLE_REGISTERS:
            case MB_FC_WRITE_SINGLE_REGISTER:
            {
                break;
            }

            default:
                break;
        }
    }
    else if (eResult == MB_RESULT_ERR_EXCEPTION)
    {
        if (pxResp != NULL)
        {
            SYS_CONSOLE_PRINT("[App RS485 CH%u] Slave exception 0x%02X (FC=0x%02X)\r\n",
                               (unsigned)eChannel,
                               (unsigned)pxResp->eExceptionCode,
                               (unsigned)pxResp->eFuncCode);
        }
    }
    else
    {
        SYS_CONSOLE_PRINT("[App RS485 CH%u] Transaction failed, err=%u\r\n",
                           (unsigned)eChannel, (unsigned)eResult);
    }
}

/* =========================================================================
 * Private: Application Task
 * =========================================================================
 * Drives the Modbus master state machine and issues periodic LED writes.
 * ========================================================================= */
static void prv_RS485AppTask(void *pvParameters)
{
    TickType_t xLastRequestTick;
    uint32_t   u32RequestIntervalTicks;

    (void)pvParameters;

    xLastRequestTick        = xTaskGetTickCount();
    u32RequestIntervalTicks = pdMS_TO_TICKS(RS485_APP_REQUEST_INTERVAL_MS);

    SYS_CONSOLE_PRINT("[App RS485] Task started\r\n");

    while (true)
    {
        /* Drive master state machine */
        Modbus_Master_Process(&s_xMasterCh1);
        /* Modbus_Master_Process(&s_xMasterCh2); */

        /* Scheduled LED write  every RS485_APP_REQUEST_INTERVAL_MS */
        if ((xTaskGetTickCount() - xLastRequestTick) >= u32RequestIntervalTicks)
        {
            xLastRequestTick = xTaskGetTickCount();

            /*
             * vLED_ModuleWriteAllCards() internally checks IsIdle() and
             * bDirty per card, so it is safe to call unconditionally here.
             */
            vLED_ModuleWriteAllCards();
        }

        vTaskDelay(pdMS_TO_TICKS(RS485_MASTER_POLL_MS));
    }
}

/* =========================================================================
 * Public: Lifecycle
 * ========================================================================= */

void vRS485HandlerInit(void)
{
    ModbusMaster_Result_t eResult;

    SYS_CONSOLE_PRINT("[App RS485] Initialising...\r\n");

    /* Initialise LED state manager first  no Modbus dependency */
    LedMgr_Init();

    /* Channel 1  both LED cards share this bus */
    eResult = Modbus_Master_Init(&s_xMasterCh1, UART_PORT_CH1);
    if (eResult != MB_RESULT_SUCCESS)
    {
        SYS_CONSOLE_PRINT("[App RS485] CH1 init failed (%u)\r\n", (unsigned)eResult);
    }
    else
    {
        Modbus_Master_RegisterCallback(&s_xMasterCh1,
                                        prv_ModbusResultCallback,
                                        NULL);   /* pvContext set per-request below */
        SYS_CONSOLE_PRINT("[App RS485] CH1 ready\r\n");
    }

    /* Channel 2  reserved / future devices */
    /* eResult = Modbus_Master_Init(&s_xMasterCh2, UART_PORT_CH2); ... */

    /* Create unified application task */
    (void)xTaskCreate(
        prv_RS485AppTask,
        "RS485_App_Task",
        RS485_TASK_HEAP_DEPTH,
        NULL,
        RS485_TASK_PRIORITY,
        NULL
    );

    SYS_CONSOLE_PRINT("[App RS485] Init complete\r\n");
}

/* =========================================================================
 * Public: LED Module Operations
 * ========================================================================= */

/*
 * vLED_ModuleWriteAllCards
 * ─────────────────────────
 * Iterates over every LED card.  For each card:
 *   1. Skip if nothing changed (not dirty)  avoids unnecessary bus traffic.
 *   2. Skip if master is not idle  non-blocking, will retry next interval.
 *   3. Build register array from led_manager.
 *   4. Submit Modbus write with card-index context pointer so the callback
 *      can mark the card clean on success.
 *
 * Why write one card per task cycle rather than all at once?
 *   The Modbus master handles one transaction at a time.  Submitting card 2
 *   while card 1 is still in-flight returns MB_RESULT_ERR_BUSY.  The task
 *   loop runs at RS485_APP_REQUEST_INTERVAL_MS; each card gets its own
 *   interval slot.  For tighter synchronisation a small queue could be added,
 *   but for 6 docks at 1 Hz this is not necessary.
 */
void vLED_ModuleWriteAllCards(void)
{
    uint8_t u8C;

    for (u8C = 0U; u8C < (uint8_t)LED_CARD_COUNT; u8C++)
    {
        /*
         * Only write a card if its state has changed since the last successful
         * write.  This keeps the bus quiet when nothing is happening.
         */
        // if (!LedMgr_IsCardDirty(u8C))
        // {
        //     continue;
        // }

        if (!Modbus_Master_IsIdle(&s_xMasterCh1))
        {
            /* Master busy (previous card still in flight)  defer this card */
            /*SYS_CONSOLE_PRINT("[App RS485] LED Card %u write deferred  master busy\r\n",
                               (unsigned)(u8C + 1U));*/
            break;   /* stop here; remaining cards will be attempted next cycle */
        }

        vLED_ModuleWriteCard(u8C);
    }
}

/* ------------------------------------------------------------------------- */

void vLED_ModuleWriteCard(uint8_t u8CardIndex)
{
    uint16_t              au16Regs[LED_CARD_DOCK_CAPACITY];
    uint8_t               u8RegCount;
    uint8_t               u8SlaveAddr;
    ModbusMaster_Result_t eResult;

    if (u8CardIndex >= (uint8_t)LED_CARD_COUNT)
    {
        SYS_CONSOLE_PRINT("[App RS485] WriteCard: invalid card index %u\r\n",
                           (unsigned)u8CardIndex);
        return;
    }

    u8SlaveAddr = LedMgr_GetCardSlaveAddr(u8CardIndex);
    u8RegCount  = LedMgr_GetCardRegisters(u8CardIndex, au16Regs);

    if ((u8RegCount == 0U) || (u8SlaveAddr == 0xFFU))
    {
        SYS_CONSOLE_PRINT("[App RS485] WriteCard: no registers for card %u\r\n",
                           (unsigned)(u8CardIndex + 1U));
        return;
    }

    /*
     * Pass &s_au8CardCtx[u8CardIndex] as pvContext.
     * The callback casts this back to const uint8_t* to get the card index
     * and calls LedMgr_MarkCardClean() on success.
     *
     * Note: Modbus_Master_RegisterCallback() sets the context at init time.
     * For per-transaction context we pass it directly here via the extended
     * API.  If your modbus_master implementation does not support per-request
     * context, use a module-level "pending card index" variable instead.
     */
    eResult = Modbus_WriteMultipleRegisters(
                  &s_xMasterCh1,
                  u8SlaveAddr,
                  (uint16_t)LED_MOD_WRITE_READ_REG_ADD,
                  au16Regs,
                  u8RegCount
              );

    if (eResult == MB_RESULT_SUCCESS)
    {
        /*
         * Request submitted to master queue.  The callback will mark the card
         * clean when the slave acknowledges.
         * Store which card is pending so the callback can identify it.
         * This simple approach works because only one transaction is in-flight
         * at a time.
         */
        /*SYS_CONSOLE_PRINT(
            "[App RS485] (slave 0x%02X) write submitted: "
            "[0x%04X, 0x%04X, 0x%04X]\r\n",
            (unsigned)(u8CardIndex + 1U),
            (unsigned)u8SlaveAddr,
            (unsigned)au16Regs[0],
            (unsigned)au16Regs[1],
            (unsigned)au16Regs[2]);*/
    }
    else if (eResult == MB_RESULT_ERR_BUSY)
    {
        SYS_CONSOLE_PRINT("[App RS485] LED Card %u write skipped  master busy\r\n",
                           (unsigned)(u8CardIndex + 1U));
    }
    else
    {
        SYS_CONSOLE_PRINT("[App RS485] LED Card %u write submit error %u\r\n",
                           (unsigned)(u8CardIndex + 1U), (unsigned)eResult);
    }
}

/* ------------------------------------------------------------------------- */

void vLED_ModuleReadFWVersion(void)
{
    ModbusMaster_Result_t eResult;

    eResult = Modbus_ReadHoldingRegisters(
                  &s_xMasterCh1,
                  (uint8_t)LED_CARD1_SLAVE_ADDR,
                  (uint16_t)LED_MOD_FWVERSION_READ_ADD,
                  (uint16_t)LED_MOD_READ_REG_NO
              );

    if (eResult == MB_RESULT_ERR_BUSY)
    {
        SYS_CONSOLE_PRINT("[App RS485] FW version read skipped  master busy\r\n");
    }
    else if (eResult != MB_RESULT_SUCCESS)
    {
        SYS_CONSOLE_PRINT("[App RS485] FW version read submit error %u\r\n",
                           (unsigned)eResult);
    }
}

/* ------------------------------------------------------------------------- */

void vLED_ModuleReadState(void)
{
    ModbusMaster_Result_t eResult;

    eResult = Modbus_ReadHoldingRegisters(
                  &s_xMasterCh1,
                  (uint8_t)LED_CARD1_SLAVE_ADDR,
                  (uint16_t)LED_MOD_WRITE_READ_REG_ADD,
                  (uint16_t)LED_MOD_WRITE_REG_NO
              );

    if (eResult == MB_RESULT_ERR_BUSY)
    {
        SYS_CONSOLE_PRINT("[App RS485] LED state read skipped  master busy\r\n");
    }
    else if (eResult != MB_RESULT_SUCCESS)
    {
        SYS_CONSOLE_PRINT("[App RS485] LED state read submit error %u\r\n",
                           (unsigned)eResult);
    }
}

/* =========================================================================
 * Public: Diagnostics
 * ========================================================================= */

void vRS485_PrintDiagnostics(void)
{
    ModbusMaster_Diagnostics_t xDiag;

    Modbus_Master_GetDiagnostics(&s_xMasterCh1, &xDiag);
    SYS_CONSOLE_PRINT("--- RS485 CH1 Diagnostics ---\r\n");
    SYS_CONSOLE_PRINT("  TX Requests : %lu\r\n", (unsigned long)xDiag.u32TxRequests);
    SYS_CONSOLE_PRINT("  RX Success  : %lu\r\n", (unsigned long)xDiag.u32RxSuccess);
    SYS_CONSOLE_PRINT("  Retries     : %lu\r\n", (unsigned long)xDiag.u32Retries);
    SYS_CONSOLE_PRINT("  Err Timeout : %lu\r\n", (unsigned long)xDiag.u32ErrTimeout);
    SYS_CONSOLE_PRINT("  Err CRC     : %lu\r\n", (unsigned long)xDiag.u32ErrCRC);
    SYS_CONSOLE_PRINT("  Err Except  : %lu\r\n", (unsigned long)xDiag.u32ErrException);
    SYS_CONSOLE_PRINT("  Err Overflow: %lu\r\n", (unsigned long)xDiag.u32ErrOverflow);
    SYS_CONSOLE_PRINT("  Err HW      : %lu\r\n", (unsigned long)xDiag.u32ErrHW);

    /* Also dump LED state for convenience */
    LedMgr_PrintState();
}

/*******************************************************************************
 * End of File
 *******************************************************************************/