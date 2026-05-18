/*******************************************************************************
 * @file    led_manager.h
 * @brief   LED State Manager — Header
 *
 * @details This module owns ALL LED application state.  It is the single
 *          source of truth for which dock shows which colour and pattern.
 *
 *          Callers (application tasks, charge controllers, etc.) use the
 *          public API to set LED states.  The RS485 transport layer
 *          (AppRS485Handler) calls LedMgr_GetRegisterValue() to fetch the
 *          encoded 16-bit Modbus register for each dock, then submits the
 *          Modbus write independently.
 *
 *          Physical mapping
 *          ─────────────────
 *          One LED card can drive up to LED_CARD_DOCK_CAPACITY docks.
 *          When more docks are needed a second LED card is wired on the
 *          same RS485 bus with a different slave address.
 *
 *          LED Card 1  Slave 0x08  →  Docks DOCK_1 … DOCK_3
 *          LED Card 2  Slave 0x09  →  Docks DOCK_4 … DOCK_6
 *
 *          Register layout (one register per dock, per card):
 *          ┌─────────────────────────────────────────────────────┐
 *          │  Bits [15:12]  reserved (0)                         │
 *          │  Bits [11: 8]  Blue  LED state  (LedState_t, 4-bit) │
 *          │  Bits [ 7: 4]  Green LED state  (LedState_t, 4-bit) │
 *          │  Bits [ 3: 0]  Red   LED state  (LedState_t, 4-bit) │
 *          └─────────────────────────────────────────────────────┘
 *
 * @company BACANCY SYSTEMS PVT. LTD.
 *******************************************************************************/

#ifndef LED_MANAGER_H
#define LED_MANAGER_H

#include <stdint.h>
#include <stdbool.h>
#include "sessionDBHandler.h"   /* For Dock_e */
/* =========================================================================
 * Capacity constants
 * ========================================================================= */

/** Maximum number of docks one LED card can serve */
#define LED_CARD_DOCK_CAPACITY      (3U)

/** Total number of docks in the system (across all LED cards) */
#define LED_TOTAL_DOCK_COUNT        (6U)

/** Number of LED cards required */
#define LED_CARD_COUNT              \
    ((LED_TOTAL_DOCK_COUNT + LED_CARD_DOCK_CAPACITY - 1U) / LED_CARD_DOCK_CAPACITY)

/* =========================================================================
 * LED state enum
 * =========================================================================
 * Values are stored in 4-bit fields inside the Modbus register.
 * Do not exceed 15 (0xF) without widening the bit-field layout.
 * ========================================================================= */
typedef enum
{
    LED_STATE_OFF             = 0U,  /**< LED fully off                    */
    LED_STATE_STEADY          = 1U,  /**< Constant on                      */
    LED_STATE_SLOW_BLINK      = 2U,  /**< ~0.5 Hz blink                    */
    LED_STATE_MEDIUM_BLINK    = 3U,  /**< ~1 Hz blink                      */
    LED_STATE_FAST_BLINK      = 4U,  /**< ~2 Hz blink                      */
    LED_STATE_MAX                    /**< Sentinel — do not use as a state  */
} LedState_t;

/* =========================================================================
 * LED colour enum
 * ========================================================================= */
typedef enum
{
    LED_COLOR_RED   = 0U,
    LED_COLOR_GREEN = 1U,
    LED_COLOR_BLUE  = 2U,
    LED_COLOR_MAX
} LedColor_t;

/* =========================================================================
 * Per-dock LED descriptor
 * =========================================================================
 * Stores the desired state for each of the three colour channels of one dock.
 * bDirty is set whenever any field changes; cleared after the register value
 * has been successfully written to the slave device.
 * ========================================================================= */
typedef struct
{
    LedState_t  eRed;       /**< Red   channel state   */
    LedState_t  eGreen;     /**< Green channel state   */
    LedState_t  eBlue;      /**< Blue  channel state   */
    bool        bDirty;     /**< true when local state differs from last written value */
} LedDockState_t;

/* =========================================================================
 * LED card descriptor
 * =========================================================================
 * Groups the docks that share one physical LED card (slave device).
 * ========================================================================= */
typedef struct
{
    uint8_t        u8SlaveAddr;                          /**< Modbus slave address     */
    LedDockState_t axDocks[LED_CARD_DOCK_CAPACITY];      /**< Per-dock state array     */
} LedCard_t;

/* =========================================================================
 * Public API — Lifecycle
 * ========================================================================= */

/**
 * @brief  Initialise the LED manager.
 *
 * @details Sets all docks to LED_STATE_OFF on all colours.
 *          Must be called once before any other LedMgr_* function.
 */
void LedMgr_Init(void);

/* =========================================================================
 * Public API — State setters
 * ========================================================================= */

/**
 * @brief  Set a single colour channel of one dock to a given state.
 *
 * @details The other two colour channels are not modified.
 *          To set an exclusive colour (one on, others off) use
 *          LedMgr_SetDockColor() instead.
 *
 * @param[in]  eDock    Dock identifier (DOCK_1 … DOCK_MAX-1).
 * @param[in]  eColor   Colour channel (LED_COLOR_RED/GREEN/BLUE).
 * @param[in]  eState   Desired state (LED_STATE_OFF … LED_STATE_FAST_BLINK).
 *
 * @return  true  Parameters valid, state updated.
 *          false Invalid dock, colour, or state — state unchanged.
 */
bool LedMgr_SetChannel(Dock_e  eDock,
                        LedColor_t eColor,
                        LedState_t eState);

/**
 * @brief  Set one dock to display a single colour; turn the others off.
 *
 * @details This is the most common call from application logic:
 *          @code
 *          LedMgr_SetDockColor(DOCK_1, LED_COLOR_BLUE, LED_STATE_STEADY);
 *          @endcode
 *
 * @param[in]  eDock    Dock identifier.
 * @param[in]  eColor   The active colour channel.
 * @param[in]  eState   State for the active channel.
 *
 * @return  true  State updated.
 *          false Invalid parameters.
 */
bool LedMgr_SetDockColor(Dock_e  eDock,
                          LedColor_t eColor,
                          LedState_t eState);

/**
 * @brief  Turn off all colour channels of one dock.
 *
 * @param[in]  eDock  Dock identifier.
 *
 * @return  true on success, false if eDock is out of range.
 */
bool LedMgr_SetDockOff(Dock_e eDock);

/**
 * @brief  Turn off all docks across all LED cards.
 */
void LedMgr_SetAllOff(void);

/* =========================================================================
 * Public API — State getters
 * ========================================================================= */

/**
 * @brief  Get the full state struct for one dock (read-only copy).
 *
 * @param[in]   eDock   Dock identifier.
 * @param[out]  pxOut   Destination struct — filled on success.
 *
 * @return  true  eDock valid, *pxOut filled.
 *          false eDock out of range, *pxOut unchanged.
 */
bool LedMgr_GetDockState(Dock_e eDock, LedDockState_t *pxOut);

/**
 * @brief  Encode the current state of one dock into a 16-bit Modbus register.
 *
 * @details Register layout:
 *          bits [11:8] = Blue state, bits [7:4] = Green state, bits [3:0] = Red state.
 *
 * @param[in]  eDock  Dock identifier.
 *
 * @return  Encoded 16-bit register value.
 *          Returns 0x0000 (all off) if eDock is out of range.
 */
uint16_t LedMgr_GetRegisterValue(Dock_e eDock);

/**
 * @brief  Fill an array of register values for all docks on one LED card.
 *
 * @details Convenience function called by AppRS485Handler before a
 *          Modbus_WriteMultipleRegisters() for one card.
 *
 * @param[in]   u8CardIndex    LED card index (0 = Card 1, 1 = Card 2 …).
 * @param[out]  pu16RegBuf     Caller-allocated buffer of at least
 *                             LED_CARD_DOCK_CAPACITY uint16_t elements.
 *
 * @return  Number of registers filled (LED_CARD_DOCK_CAPACITY), or 0 on error.
 */
uint8_t LedMgr_GetCardRegisters(uint8_t   u8CardIndex,
                                  uint16_t *pu16RegBuf);

/**
 * @brief  Return the Modbus slave address for a given card index.
 *
 * @param[in]  u8CardIndex  Card index (0-based).
 *
 * @return  Slave address, or 0xFF if index out of range.
 */
uint8_t LedMgr_GetCardSlaveAddr(uint8_t u8CardIndex);

/**
 * @brief  Check whether any dock on a card has unsynchronised (dirty) state.
 *
 * @param[in]  u8CardIndex  Card index (0-based).
 *
 * @return  true if at least one dock on the card is dirty.
 */
bool LedMgr_IsCardDirty(uint8_t u8CardIndex);

/**
 * @brief  Mark all docks on a card as clean (synced to the slave).
 *
 * @details Call this from the Modbus success callback after a successful write.
 *
 * @param[in]  u8CardIndex  Card index (0-based).
 */
void LedMgr_MarkCardClean(uint8_t u8CardIndex);

/* =========================================================================
 * Public API — Diagnostics
 * ========================================================================= */

/**
 * @brief  Print full LED state table to SYS_CONSOLE.
 */
void LedMgr_PrintState(void);
/**
 * @brief  Convenience functions for RS485 output mapping.
 *          These are called by AppRS485Handler when processing RS485 write
 *          requests to the LED control registers.  They map the global DOCK_x
 *          identifiers to the appropriate card and colour channel.
 */
bool RS485_Output_1_Set(LedState_t eState);
bool RS485_Output_2_Set(LedState_t eState);
bool RS485_Output_3_Set(LedState_t eState);
bool RS485_Output_4_Set(LedState_t eState);
bool RS485_Output_5_Set(LedState_t eState);
bool RS485_Output_6_Set(LedState_t eState);
bool RS485_Output_7_Set(LedState_t eState);
bool RS485_Output_8_Set(LedState_t eState);
bool RS485_Output_9_Set(LedState_t eState);
bool RS485_Output_10_Set(LedState_t eState);
bool RS485_Output_11_Set(LedState_t eState);
bool RS485_Output_12_Set(LedState_t eState);
bool RS485_Output_13_Set(LedState_t eState);
bool RS485_Output_14_Set(LedState_t eState);
bool RS485_Output_15_Set(LedState_t eState);
bool RS485_Output_16_Set(LedState_t eState);
bool RS485_Output_17_Set(LedState_t eState);
bool RS485_Output_18_Set(LedState_t eState);
#endif /* LED_MANAGER_H */

/*******************************************************************************
 * End of File
 *******************************************************************************/