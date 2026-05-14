/*******************************************************************************
 * @file    AppRS485Handler.c
 * @brief   RS485 Application Layer - Implementation
 * @details Application-layer implementation.  This file contains ONLY:
 *            - System initialisation (init Modbus master, create tasks)
 *            - Business-logic request functions (LED module read/write)
 *            - Result callback handler (interprets responses)
 *            - Periodic scheduling task (drives state machine + requests)
 *
 *          There is NO direct SERCOM, GPIO, TCP, or CRC code in this file.
 *          All those concerns are handled by the lower-layer modules.
 *
 * @company BACANCY SYSTEMS PVT. LTD.
 *******************************************************************************/

/* =========================================================================
 * Included Files
 * ========================================================================= */
#include <stddef.h>
#include <stdbool.h>
#include <string.h>
#include "definitions.h"          /* SYS_CONSOLE_PRINT                    */
#include "AppRS485Handler.h"
// #include "MB_Master_Lib/modbus_rtu.h"
#include "FreeRTOS.h"
#include "task.h"

/* =========================================================================
 * Private Data — Master channel contexts
 * =========================================================================
 * Statically allocated; no heap usage for the master contexts themselves.
 * ========================================================================= */

/** Modbus Master context for RS485 Bus 1 (CH1 — LED/OP Card bus) */
static ModbusMaster_Channel_t s_xMasterCh1;

/** Modbus Master context for RS485 Bus 2 (CH2 — reserved / future) */
static ModbusMaster_Channel_t s_xMasterCh2;

/* =========================================================================
 * Private: Result Callback
 * =========================================================================
 * Called by Modbus_Master_Process() upon transaction completion.
 * Runs in the context of the application task (not ISR).
 * ========================================================================= */

/**
 * @brief  Modbus transaction result callback — interprets slave responses.
 *
 * @details Handles successful responses and all error conditions.
 *          Extend this function to update application state, trigger alarms,
 *          or log diagnostic events.
 */
static void prv_ModbusResultCallback(UartPort_Channel_t         eChannel,
                                      ModbusMaster_Result_t      eResult,
                                      const ModbusRtu_Response_t *pxResp,
                                      void                       *pvContext)
{
    (void)pvContext;    /* Not used in this implementation */

    if (eResult == MB_RESULT_SUCCESS)
    {
        if (pxResp == NULL)
        {
            return;
        }

        SYS_CONSOLE_PRINT("[App RS485 CH%u] Success FC=0x%02X DataLen=%u\r\n",
                           (unsigned)eChannel,
                           (unsigned)pxResp->eFuncCode,
                           (unsigned)pxResp->u8DataLen);

        switch (pxResp->eFuncCode)
        {
            case MB_FC_READ_HOLDING_REGISTERS:
            {
                /* Each register = 2 bytes, big-endian in response */
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

                    /*
                     * TODO: Route register values to appropriate application state.
                     * Example: u16StoreLedModuleRegister(u8Idx, u16RegVal);
                     */
                }
                break;
            }

            case MB_FC_WRITE_SINGLE_REGISTER:
            case MB_FC_WRITE_MULTIPLE_REGISTERS:
                /* Echo response — success confirmation only */
                SYS_CONSOLE_PRINT("[App RS485 CH%u] Write acknowledged\r\n",
                                   (unsigned)eChannel);
                break;

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

        /*
         * TODO: Integrate with your error reporting system:
         * ReportError(eChannel == UART_PORT_CH1 ? ERR_RS485_1_NO_RESPONSE
         *                                       : ERR_RS485_2_NO_RESPONSE);
         */
    }
}

/* =========================================================================
 * Private: Application Task
 * =========================================================================
 * Single FreeRTOS task that:
 *   1. Drives the Modbus master state machine (both channels).
 *   2. Issues periodic business-logic requests on a timer.
 * ========================================================================= */

/**
 * @brief  RS485 Modbus master application task.
 *
 * @details Runs at RS485_TASK_PRIORITY.  Periodically calls:
 *          - Modbus_Master_Process() to drive the state machine
 *          - Application request functions to send Modbus commands
 */
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
        /* --- Drive both master state machines --- */
        // Modbus_Master_Process(&s_xMasterCh1);
        // Modbus_Master_Process(&s_xMasterCh2);

        /* --- Scheduled application requests (every RS485_APP_REQUEST_INTERVAL_MS) --- */
        if ((xTaskGetTickCount() - xLastRequestTick) >= u32RequestIntervalTicks)
        {
            xLastRequestTick = xTaskGetTickCount();

            /*
             * Example: write LED states periodically.
             * Only submit if master is idle (non-blocking check).
             */
            if (Modbus_Master_IsIdle(&s_xMasterCh1))
            {
                vLED_ModuleWrite();
            }
        }

        /* Yield for RS485_MASTER_POLL_MS */
        vTaskDelay(pdMS_TO_TICKS(RS485_MASTER_POLL_MS));
    }
}

/* =========================================================================
 * Public Function Implementations — Lifecycle
 * ========================================================================= */

void vRS485HandlerInit(void)
{
    ModbusMaster_Result_t eResult;

    SYS_CONSOLE_PRINT("[App RS485] Initialising Modbus Master stack...\r\n");

    /* ------------------------------------------------------------------ */
    /* Channel 1 — LED / OP Card bus (SERCOM8)                            */
    /* ------------------------------------------------------------------ */
    eResult = Modbus_Master_Init(&s_xMasterCh1, UART_PORT_CH1);
    if (eResult != MB_RESULT_SUCCESS)
    {
        SYS_CONSOLE_PRINT("[App RS485] CH1 init failed (%u)\r\n", (unsigned)eResult);
    }
    else
    {
        Modbus_Master_RegisterCallback(&s_xMasterCh1,
                                        prv_ModbusResultCallback,
                                        NULL);
        SYS_CONSOLE_PRINT("[App RS485] CH1 ready\r\n");
    }

    /* ------------------------------------------------------------------ */
    /* Channel 2 — Reserved / future slave devices (SERCOM9)              */
    /* ------------------------------------------------------------------ */
    // eResult = Modbus_Master_Init(&s_xMasterCh2, UART_PORT_CH2);
    // if (eResult != MB_RESULT_SUCCESS)
    // {
    //     SYS_CONSOLE_PRINT("[App RS485] CH2 init failed (%u)\r\n", (unsigned)eResult);
    // }
    // else
    // {
    //     Modbus_Master_RegisterCallback(&s_xMasterCh2,
    //                                     prv_ModbusResultCallback,
    //                                     NULL);
    //     SYS_CONSOLE_PRINT("[App RS485] CH2 ready\r\n");
    // }

    /* ------------------------------------------------------------------ */
    /* Create unified application task                                     */
    /* ------------------------------------------------------------------ */
    (void)xTaskCreate(
        prv_RS485AppTask,
        "RS485_App_Task",
        RS485_TASK_HEAP_DEPTH,
        NULL,
        RS485_TASK_PRIORITY,
        NULL
    );

    SYS_CONSOLE_PRINT("[App RS485] Initialisation complete\r\n");
}

/* =========================================================================
 * Public Function Implementations — LED Module Operations
 * ========================================================================= */

void vLED_ModuleWrite(void)
{
    ModbusMaster_Result_t eResult;
    uint16_t              au16Values[LED_MOD_WRITE_REG_NO];
    uint8_t               u8Idx;

    /* Gather local LED state for each dock */
    for (u8Idx = DOCK_1; u8Idx < (uint8_t)MAX_DOCKS; u8Idx++)
    {
        au16Values[u8Idx] = u16GetLocalLedState(u8Idx);
    }

    /* Submit Write Multiple Registers request to LED module */
    eResult = Modbus_WriteMultipleRegisters(
                  &s_xMasterCh1,
                  (uint8_t)LED_MOD_ADD,
                  (uint16_t)LED_MOD_WRITE_READ_REG_ADD,
                  au16Values,
                  (uint8_t)LED_MOD_WRITE_REG_NO
              );

    if (eResult == MB_RESULT_ERR_BUSY)
    {
        SYS_CONSOLE_PRINT("[App RS485] LED write skipped — master busy\r\n");
    }
    else if (eResult != MB_RESULT_SUCCESS)
    {
        SYS_CONSOLE_PRINT("[App RS485] LED write submit error %u\r\n", (unsigned)eResult);
    }
}

/* ------------------------------------------------------------------------- */

void vLED_ModuleReadFWVersion(void)
{
    ModbusMaster_Result_t eResult;

    eResult = Modbus_ReadHoldingRegisters(
                  &s_xMasterCh1,
                  (uint8_t)LED_MOD_ADD,
                  (uint16_t)LED_MOD_FWVERSION_READ_ADD,
                  (uint16_t)LED_MOD_READ_REG_NO
              );

    if (eResult == MB_RESULT_ERR_BUSY)
    {
        SYS_CONSOLE_PRINT("[App RS485] FW version read skipped — master busy\r\n");
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
                  (uint8_t)LED_MOD_ADD,
                  (uint16_t)LED_MOD_WRITE_READ_REG_ADD,
                  (uint16_t)LED_MOD_WRITE_REG_NO
              );

    if (eResult == MB_RESULT_ERR_BUSY)
    {
        SYS_CONSOLE_PRINT("[App RS485] LED state read skipped — master busy\r\n");
    }
    else if (eResult != MB_RESULT_SUCCESS)
    {
        SYS_CONSOLE_PRINT("[App RS485] LED state read submit error %u\r\n",
                           (unsigned)eResult);
    }
}

/* =========================================================================
 * Public Function Implementations — Diagnostics
 * ========================================================================= */

void vRS485_PrintDiagnostics(void)
{
    ModbusMaster_Diagnostics_t xDiag;

    /* Channel 1 */
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

    // /* Channel 2 */
    // Modbus_Master_GetDiagnostics(&s_xMasterCh2, &xDiag);
    // SYS_CONSOLE_PRINT("--- RS485 CH2 Diagnostics ---\r\n");
    // SYS_CONSOLE_PRINT("  TX Requests : %lu\r\n", (unsigned long)xDiag.u32TxRequests);
    // SYS_CONSOLE_PRINT("  RX Success  : %lu\r\n", (unsigned long)xDiag.u32RxSuccess);
    // SYS_CONSOLE_PRINT("  Retries     : %lu\r\n", (unsigned long)xDiag.u32Retries);
    // SYS_CONSOLE_PRINT("  Err Timeout : %lu\r\n", (unsigned long)xDiag.u32ErrTimeout);
    // SYS_CONSOLE_PRINT("  Err CRC     : %lu\r\n", (unsigned long)xDiag.u32ErrCRC);
    // SYS_CONSOLE_PRINT("  Err Except  : %lu\r\n", (unsigned long)xDiag.u32ErrException);
    // SYS_CONSOLE_PRINT("  Err Overflow: %lu\r\n", (unsigned long)xDiag.u32ErrOverflow);
    // SYS_CONSOLE_PRINT("  Err HW      : %lu\r\n", (unsigned long)xDiag.u32ErrHW);
}

/* =========================================================================
 * Stub Implementation — u16GetLocalLedState
 * =========================================================================
 * Replace this with your actual LED state manager implementation.
 * ========================================================================= */

/**
 * @brief  Stub: return LED state for a dock.
 *         Replace with real implementation from LED state module.
 */
uint16_t u16GetLocalLedState(uint8_t u8Dock)
{
    /* Placeholder — return 0 for all docks until real state manager exists */
    (void)u8Dock;
    return 0U;
}

// void RS485_Output_1_Set(void)   { }
// void RS485_Output_1_Clear(void) { }

// void RS485_Output_2_Set(void)   { }
// void RS485_Output_2_Clear(void) { }

// void RS485_Output_3_Set(void)   { }
// void RS485_Output_3_Clear(void) { }

// void RS485_Output_4_Set(void)   { }
// void RS485_Output_4_Clear(void) { }

// void RS485_Output_5_Set(void)   { }
// void RS485_Output_5_Clear(void) { }

// void RS485_Output_6_Set(void)   { }
// void RS485_Output_6_Clear(void) { }

// void RS485_Output_7_Set(void)   { }
// void RS485_Output_7_Clear(void) { }

// void RS485_Output_8_Set(void)   { }
// void RS485_Output_8_Clear(void) { }

// void RS485_Output_9_Set(void)   { }
// void RS485_Output_9_Clear(void) { }

// void RS485_Output_10_Set(void)   { }
// void RS485_Output_10_Clear(void) { }

// void RS485_Output_11_Set(void)   { }
// void RS485_Output_11_Clear(void) { }

// void RS485_Output_12_Set(void)   { }
// void RS485_Output_12_Clear(void) { }

// void RS485_Output_13_Set(void)   { }
// void RS485_Output_13_Clear(void) { }

// void RS485_Output_14_Set(void)   { }
// void RS485_Output_14_Clear(void) { }

// void RS485_Output_15_Set(void)   { }
// void RS485_Output_15_Clear(void) { }

// void RS485_Output_16_Set(void)   { }
// void RS485_Output_16_Clear(void) { }

// void RS485_Output_17_Set(void)   { }
// void RS485_Output_17_Clear(void) { }

// void RS485_Output_18_Set(void)   { }
// void RS485_Output_18_Clear(void) { }

// uint8_t RS485_Output_1_Get(void)  { return 0U; }
// uint8_t RS485_Output_2_Get(void)  { return 0U; }
// uint8_t RS485_Output_3_Get(void)  { return 0U; }
// uint8_t RS485_Output_4_Get(void)  { return 0U; }
// uint8_t RS485_Output_5_Get(void)  { return 0U; }
// uint8_t RS485_Output_6_Get(void)  { return 0U; }
// uint8_t RS485_Output_7_Get(void)  { return 0U; }
// uint8_t RS485_Output_8_Get(void)  { return 0U; }
// uint8_t RS485_Output_9_Get(void)  { return 0U; }
// uint8_t RS485_Output_10_Get(void) { return 0U; }
// uint8_t RS485_Output_11_Get(void) { return 0U; }
// uint8_t RS485_Output_12_Get(void) { return 0U; }
// uint8_t RS485_Output_13_Get(void) { return 0U; }
// uint8_t RS485_Output_14_Get(void) { return 0U; }
// uint8_t RS485_Output_15_Get(void) { return 0U; }
// uint8_t RS485_Output_16_Get(void) { return 0U; }
// uint8_t RS485_Output_17_Get(void) { return 0U; }
// uint8_t RS485_Output_18_Get(void) { return 0U; }
/*******************************************************************************
 * End of File
 *******************************************************************************/