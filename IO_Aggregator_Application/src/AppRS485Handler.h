/*******************************************************************************
 * @file    AppRS485Handler.h
 * @brief   RS485 Application Layer - Header
 * @details Application-layer header for Modbus RTU master operations.
 *          This file exposes ONLY business-logic APIs.
 *          All Modbus framing, CRC, and transport concerns are hidden
 *          in the lower-layer modules (modbus_master, modbus_rtu, uart_port).
 *
 *          Slave device map (Two-Wheeler IO Aggregator):
 *            LED Module (OP Card)  →  Slave address LED_MOD_ADD, Channel CH1
 *
 * @company BACANCY SYSTEMS PVT. LTD.
 *******************************************************************************/

#ifndef APP_RS485_HANDLER_H
#define APP_RS485_HANDLER_H

/* =========================================================================
 * Included Files
 * ========================================================================= */
#include <stdint.h>
#include <stdbool.h>
#include "MB_Master_Lib/modbus_master.h"

/* =========================================================================
 * Slave Device Address Map
 * =========================================================================
 * Add additional slave device addresses here as the system grows.
 * ========================================================================= */

/** LED / OP Card module Modbus slave address */
#define LED_MOD_ADD                     (8U)

/* =========================================================================
 * LED Module Register Map
 * =========================================================================
 * Register addresses are 0-based (Modbus PDU addressing).
 * Physical display address = register address + 1 (for user reference only).
 * ========================================================================= */

/** Firmware version read register start address */
#define LED_MOD_FWVERSION_READ_ADD      (0U)

/** Number of registers to read for firmware version */
#define LED_MOD_READ_REG_NO             (2U)

/** LED state write/read register start address */
#define LED_MOD_WRITE_READ_REG_ADD      (0U)

/** Number of LED state registers to write (one per dock) */
#define LED_MOD_WRITE_REG_NO            (3U)

/* =========================================================================
 * FreeRTOS Task Configuration
 * =========================================================================
 * Both tasks drive the Modbus master state machine and schedule application
 * requests.  Heap depth is in 32-bit words on ARM Cortex-M.
 * ========================================================================= */

#define RS485_TASK_HEAP_DEPTH           (1024U)
#define RS485_TASK_PRIORITY             (4U)

/** Modbus master process() call interval in milliseconds */
#define RS485_MASTER_POLL_MS            (500U)

/** Application request scheduling interval in milliseconds */
#define RS485_APP_REQUEST_INTERVAL_MS   (1000U)

/* =========================================================================
 * Public API — Lifecycle
 * ========================================================================= */

/**
 * @brief  Initialise the RS485 application layer and Modbus master stack.
 *
 * @details Calls Modbus_Master_Init() for each channel, registers result
 *          callbacks, and creates the FreeRTOS application tasks.
 *          Must be called once from the system startup task before the
 *          scheduler is started (or after, from a higher-priority init task).
 *
 * @return  None
 */
void vRS485HandlerInit(void);

/* =========================================================================
 * Public API — LED Module Operations
 * =========================================================================
 * These functions prepare Modbus requests targeting the LED (OP Card) slave.
 * They are non-blocking: they submit a request to the master and return
 * immediately.  The result is delivered via the registered callback.
 * ========================================================================= */

/**
 * @brief  Write current dock LED states to the LED module.
 *
 * @details Reads local LED state for all docks via u16GetLocalLedState()
 *          and issues a Write Multiple Registers (FC16) to the LED module.
 *
 *          Example usage:
 *          @code
 *          vLED_ModuleWrite();   // call periodically from app task
 *          @endcode
 *
 * @return  None
 */
void vLED_ModuleWrite(void);

/**
 * @brief  Read firmware version registers from the LED module.
 *
 * @details Issues a Read Holding Registers (FC03) request.
 *          Result is delivered via the registered Modbus callback.
 *
 * @return  None
 */
void vLED_ModuleReadFWVersion(void);

/**
 * @brief  Read current LED state registers from the LED module.
 *
 * @details Issues a Read Holding Registers (FC03) request for
 *          LED_MOD_WRITE_REG_NO registers starting at LED_MOD_WRITE_READ_REG_ADD.
 *
 * @return  None
 */
void vLED_ModuleReadState(void);

/* =========================================================================
 * Public API — Diagnostic Access
 * ========================================================================= */

/**
 * @brief  Print Modbus master diagnostics for both channels to console.
 *
 * @return  None
 */
void vRS485_PrintDiagnostics(void);

/* =========================================================================
 * Stub: Local state accessor (implement in LED state manager module)
 * =========================================================================
 * Declared here so AppRS485Handler.c can call it.
 * Implementation lives in the LED / dock state management module.
 * ========================================================================= */

/**
 * @brief  Retrieve the locally-cached LED state for a dock.
 *
 * @param[in]  eDock  Dock identifier (DOCK_1 … DOCK_MAX-1).
 *
 * @return  16-bit LED state register value for the specified dock.
 */
uint16_t u16GetLocalLedState(uint8_t u8Dock);



// void RS485_Output_1_Set(void);
// void RS485_Output_1_Clear(void);

// void RS485_Output_2_Set(void);
// void RS485_Output_2_Clear(void);

// void RS485_Output_3_Set(void);
// void RS485_Output_3_Clear(void);

// void RS485_Output_4_Set(void);
// void RS485_Output_4_Clear(void);

// void RS485_Output_5_Set(void);
// void RS485_Output_5_Clear(void);

// void RS485_Output_6_Set(void);
// void RS485_Output_6_Clear(void);

// void RS485_Output_7_Set(void);
// void RS485_Output_7_Clear(void);

// void RS485_Output_8_Set(void);
// void RS485_Output_8_Clear(void);

// void RS485_Output_9_Set(void);
// void RS485_Output_9_Clear(void);

// void RS485_Output_10_Set(void);
// void RS485_Output_10_Clear(void);

// void RS485_Output_11_Set(void);
// void RS485_Output_11_Clear(void);

// void RS485_Output_12_Set(void);
// void RS485_Output_12_Clear(void);

// void RS485_Output_13_Set(void);
// void RS485_Output_13_Clear(void);

// void RS485_Output_14_Set(void);
// void RS485_Output_14_Clear(void);

// void RS485_Output_15_Set(void);
// void RS485_Output_15_Clear(void);

// void RS485_Output_16_Set(void);
// void RS485_Output_16_Clear(void);

// void RS485_Output_17_Set(void);
// void RS485_Output_17_Clear(void);

// void RS485_Output_18_Set(void);
// void RS485_Output_18_Clear(void);

// uint8_t RS485_Output_1_Get(void);
// uint8_t RS485_Output_2_Get(void);
// uint8_t RS485_Output_3_Get(void);
// uint8_t RS485_Output_4_Get(void);
// uint8_t RS485_Output_5_Get(void);
// uint8_t RS485_Output_6_Get(void);
// uint8_t RS485_Output_7_Get(void);
// uint8_t RS485_Output_8_Get(void);
// uint8_t RS485_Output_9_Get(void);
// uint8_t RS485_Output_10_Get(void);
// uint8_t RS485_Output_11_Get(void);
// uint8_t RS485_Output_12_Get(void);
// uint8_t RS485_Output_13_Get(void);
// uint8_t RS485_Output_14_Get(void);
// uint8_t RS485_Output_15_Get(void);
// uint8_t RS485_Output_16_Get(void);
// uint8_t RS485_Output_17_Get(void);
// uint8_t RS485_Output_18_Get(void);
#endif /* APP_RS485_HANDLER_H */

/*******************************************************************************
 * End of File
 *******************************************************************************/
