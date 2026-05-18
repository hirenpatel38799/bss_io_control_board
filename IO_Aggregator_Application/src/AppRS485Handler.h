/*******************************************************************************
 * @file    AppRS485Handler.h
 * @brief   RS485 Application Layer - Header
 * @details Application-layer header for Modbus RTU master operations.
 *          This file exposes ONLY business-logic APIs.
 *          All Modbus framing, CRC, and transport concerns are hidden
 *          in the lower-layer modules (modbus_master, modbus_rtu, uart_port).
 *
 *          Slave device map (Two-Wheeler IO Aggregator):
 *            LED Card 1  →  Slave address LED_CARD1_SLAVE_ADDR, Docks 1–3
 *            LED Card 2  →  Slave address LED_CARD2_SLAVE_ADDR, Docks 4–6
 *          Both cards reside on the same RS485 bus (CH1).
 *
 * CHANGES vs previous revision
 * ──────────────────────────────
 * • LED_MOD_ADD renamed to LED_CARD1_SLAVE_ADDR (8U) — unchanged value,
 *   clearer name.
 * • Added LED_CARD2_SLAVE_ADDR (9U) for the second LED card.
 * • Added vLED_ModuleWriteCard() for per-card write control.
 * • Removed u16GetLocalLedState() stub — now owned by led_manager module.
 * • All dock-count constants removed from here; live in led_manager.h.
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
#include "led_manager.h"

/* =========================================================================
 * Slave Device Address Map
 * =========================================================================
 * LED cards share the same RS485 bus (CH1) but have distinct slave addresses.
 * To add a third card: define LED_CARD3_SLAVE_ADDR and extend
 * vLED_ModuleWriteAllCards() accordingly.
 * ========================================================================= */

/** LED Card 1 — Modbus slave address (handles Docks 1–3) */
#define LED_CARD1_SLAVE_ADDR            (0x08U)

/** LED Card 2 — Modbus slave address (handles Docks 4–6) */
#define LED_CARD2_SLAVE_ADDR            (0x09U)

/* Legacy alias — keep so any existing call site still compiles unchanged */
#define LED_MOD_ADD                     LED_CARD1_SLAVE_ADDR

/* =========================================================================
 * LED Module Register Map
 * =========================================================================
 * Both cards use the same register layout starting at address 0.
 * Card 1 register 0 = Dock 1, register 1 = Dock 2, register 2 = Dock 3.
 * Card 2 register 0 = Dock 4, register 1 = Dock 5, register 2 = Dock 6.
 * ========================================================================= */

/** Firmware version read register start address */
#define LED_MOD_FWVERSION_READ_ADD      (0U)

/** Number of registers to read for firmware version */
#define LED_MOD_READ_REG_NO             (2U)

/** LED state write/read register start address (same for both cards) */
#define LED_MOD_WRITE_READ_REG_ADD      (0U)

/**
 * Number of LED state registers per write transaction.
 * Always equal to LED_CARD_DOCK_CAPACITY (defined in led_manager.h).
 * Use LED_CARD_DOCK_CAPACITY directly in code; this alias is kept for
 * backward compatibility with any existing call sites.
 */
#define LED_MOD_WRITE_REG_NO            LED_CARD_DOCK_CAPACITY

/* =========================================================================
 * FreeRTOS Task Configuration
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
 * @brief  Initialise the RS485 application layer, LED Manager, and Modbus stack.
 *
 * @details Calls LedMgr_Init(), Modbus_Master_Init() for each channel,
 *          registers result callbacks, and creates the FreeRTOS application task.
 *          Must be called once from the system startup task.
 */
void vRS485HandlerInit(void);

/* =========================================================================
 * Public API — LED Module Operations
 * ========================================================================= */

/**
 * @brief  Write current LED states for ALL docks to their respective LED cards.
 *
 * @details Iterates over every card index (0 … LED_CARD_COUNT-1) and submits
 *          a Modbus_WriteMultipleRegisters() for each one, but only if:
 *            (a) the master is idle, AND
 *            (b) that card has at least one dirty (changed) dock.
 *
 *          Non-blocking.  Call periodically from the application task.
 */
void vLED_ModuleWriteAllCards(void);

/**
 * @brief  Write LED states for one specific card to its slave device.
 *
 * @details Fetches register values from LedMgr_GetCardRegisters() and
 *          issues Modbus_WriteMultipleRegisters() to the card's slave address.
 *
 * @param[in]  u8CardIndex  LED card index (0-based).
 */
void vLED_ModuleWriteCard(uint8_t u8CardIndex);

/**
 * @brief  Read firmware version registers from LED Card 1.
 */
void vLED_ModuleReadFWVersion(void);

/**
 * @brief  Read current LED state registers from LED Card 1.
 */
void vLED_ModuleReadState(void);

/* =========================================================================
 * Public API — Diagnostic Access
 * ========================================================================= */

/**
 * @brief  Print Modbus master diagnostics for both channels to console.
 */
void vRS485_PrintDiagnostics(void);

#endif /* APP_RS485_HANDLER_H */

/*******************************************************************************
 * End of File
 *******************************************************************************/