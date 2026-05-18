/*******************************************************************************
 * @file    led_manager.c
 * @brief   LED State Manager — Implementation
 *
 * @details Owns the canonical LED state for every dock.
 *          Has no knowledge of Modbus, UART, or FreeRTOS.
 *          Pure state management: set, get, encode, dirty-track.
 *
 *          Register encoding per dock (16-bit, one register per dock):
 *          ┌─────────────────────────────────────────────────────┐
 *          │  Bits [15:12]  reserved  (always 0)                 │
 *          │  Bits [11: 8]  Blue  LED state  (LedState_t, 4-bit) │
 *          │  Bits [ 7: 4]  Green LED state  (LedState_t, 4-bit) │
 *          │  Bits [ 3: 0]  Red   LED state  (LedState_t, 4-bit) │
 *          └─────────────────────────────────────────────────────┘
 *
 * @company BACANCY SYSTEMS PVT. LTD.
 *******************************************************************************/

#include "led_manager.h"
#include "definitions.h"    /* SYS_CONSOLE_PRINT */
#include <string.h>

/* =========================================================================
 * Private: Slave address table
 * =========================================================================
 * Index 0  →  LED Card 1  Slave 0x08  (Docks 1–3)
 * Index 1  →  LED Card 2  Slave 0x09  (Docks 4–6)
 *
 * Add more entries here if additional LED cards are connected in future.
 * ========================================================================= */
static const uint8_t s_au8CardSlaveAddr[LED_CARD_COUNT] =
{
    0x08U,   /* Card 0 — LED Card 1 */
    0x09U,   /* Card 1 — LED Card 2 */
};

/* =========================================================================
 * Private: State table
 * =========================================================================
 * Two LED card contexts, each owning LED_CARD_DOCK_CAPACITY dock entries.
 * ========================================================================= */
static LedCard_t s_axCards[LED_CARD_COUNT];

/* =========================================================================
 * Private helpers
 * ========================================================================= */

/**
 * @brief  Resolve a system-wide dock enum to a card index + per-card dock index.
 *
 * @param[in]   eDock          System dock (DOCK_1 … DOCK_MAX).
 * @param[out]  pu8CardIdx     Card index (0 = Card 1, 1 = Card 2).
 * @param[out]  pu8DockInCard  Index within that card's dock array (0–2).
 *
 * @return  true  eDock is valid and both outputs are written.
 *          false eDock is out of range.
 */
static bool prv_ResolveDock(Dock_e  eDock,
                             uint8_t   *pu8CardIdx,
                             uint8_t   *pu8DockInCard)
{
    uint8_t u8D = (uint8_t)eDock;

    if ((u8D >= (uint8_t)LED_TOTAL_DOCK_COUNT) ||
        (pu8CardIdx    == NULL)    ||
        (pu8DockInCard == NULL))
    {
        return false;
    }

    *pu8CardIdx    = u8D / (uint8_t)LED_CARD_DOCK_CAPACITY;
    *pu8DockInCard = u8D % (uint8_t)LED_CARD_DOCK_CAPACITY;
    return true;
}

/**
 * @brief Encode one dock state into a 16-bit Modbus register value.
 *
 * Bit mapping:
 *  Bits 0-3   : Blue
 *  Bits 4-7   : Reserved
 *  Bits 8-11  : Red
 *  Bits 12-15 : Green
 */
static uint16_t prv_EncodeRegister(const LedDockState_t *pxDock)
{
    uint16_t u16Reg = 0U;

    /* Blue LED -> bits 0-3 */
    u16Reg |= ((uint16_t)pxDock->eBlue & 0x000FU);

    /* Reserved bits 4-7 = 0 */

    /* Red LED -> bits 8-11 */
    u16Reg |= (((uint16_t)pxDock->eRed & 0x000FU) << 8U);

    /* Green LED -> bits 12-15 */
    u16Reg |= (((uint16_t)pxDock->eGreen & 0x000FU) << 12U);

    return u16Reg;
}

/* =========================================================================
 * Public: Lifecycle
 * ========================================================================= */

void LedMgr_Init(void)
{
    uint8_t u8C;
    uint8_t u8D;

    (void)memset(s_axCards, 0, sizeof(s_axCards));

    /* Assign slave addresses from the compile-time table */
    for (u8C = 0U; u8C < (uint8_t)LED_CARD_COUNT; u8C++)
    {
        s_axCards[u8C].u8SlaveAddr = s_au8CardSlaveAddr[u8C];

        for (u8D = 0U; u8D < (uint8_t)LED_CARD_DOCK_CAPACITY; u8D++)
        {
            s_axCards[u8C].axDocks[u8D].eRed   = LED_STATE_OFF;
            s_axCards[u8C].axDocks[u8D].eGreen = LED_STATE_OFF;
            s_axCards[u8C].axDocks[u8D].eBlue  = LED_STATE_OFF;
            s_axCards[u8C].axDocks[u8D].bDirty = false;
        }
    }

    SYS_CONSOLE_PRINT("[LedMgr] Initialised: %u card(s), %u docks total\r\n",
                       (unsigned)LED_CARD_COUNT,
                       (unsigned)LED_TOTAL_DOCK_COUNT);
}

/* =========================================================================
 * Public: State setters
 * ========================================================================= */

bool LedMgr_SetChannel(Dock_e  eDock,
                        LedColor_t eColor,
                        LedState_t eState)
{
    uint8_t        u8CardIdx;
    uint8_t        u8DockInCard;
    LedDockState_t *pxDock;

    if ((eColor >= LED_COLOR_MAX) || (eState >= LED_STATE_MAX))
    {
        SYS_CONSOLE_PRINT("[LedMgr] SetChannel: invalid color=%u or state=%u\r\n",
                           (unsigned)eColor, (unsigned)eState);
        return false;
    }

    if (!prv_ResolveDock(eDock, &u8CardIdx, &u8DockInCard))
    {
        SYS_CONSOLE_PRINT("[LedMgr] SetChannel: invalid dock=%u\r\n", (unsigned)eDock);
        return false;
    }

    pxDock = &s_axCards[u8CardIdx].axDocks[u8DockInCard];

    switch (eColor)
    {
        case LED_COLOR_RED:   pxDock->eRed   = eState; break;
        case LED_COLOR_GREEN: pxDock->eGreen = eState; break;
        case LED_COLOR_BLUE:  pxDock->eBlue  = eState; break;
        default: return false;
    }

    pxDock->bDirty = true;
    return true;
}

/* ------------------------------------------------------------------------- */

bool LedMgr_SetDockColor(Dock_e  eDock,
                          LedColor_t eColor,
                          LedState_t eState)
{
    uint8_t        u8CardIdx;
    uint8_t        u8DockInCard;
    LedDockState_t *pxDock;

    if ((eColor >= LED_COLOR_MAX) || (eState >= LED_STATE_MAX))
    {
        SYS_CONSOLE_PRINT("[LedMgr] SetDockColor: invalid color=%u or state=%u\r\n",
                           (unsigned)eColor, (unsigned)eState);
        return false;
    }

    if (!prv_ResolveDock(eDock, &u8CardIdx, &u8DockInCard))
    {
        SYS_CONSOLE_PRINT("[LedMgr] SetDockColor: invalid dock=%u\r\n", (unsigned)eDock);
        return false;
    }

    pxDock = &s_axCards[u8CardIdx].axDocks[u8DockInCard];

    /* Turn all channels off first, then activate the requested one */
    pxDock->eRed   = LED_STATE_OFF;
    pxDock->eGreen = LED_STATE_OFF;
    pxDock->eBlue  = LED_STATE_OFF;

    switch (eColor)
    {
        case LED_COLOR_RED:   pxDock->eRed   = eState; break;
        case LED_COLOR_GREEN: pxDock->eGreen = eState; break;
        case LED_COLOR_BLUE:  pxDock->eBlue  = eState; break;
        default: return false;
    }

    pxDock->bDirty = true;
    return true;
}

/* ------------------------------------------------------------------------- */

bool LedMgr_SetDockOff(Dock_e eDock)
{
    uint8_t        u8CardIdx;
    uint8_t        u8DockInCard;
    LedDockState_t *pxDock;

    if (!prv_ResolveDock(eDock, &u8CardIdx, &u8DockInCard))
    {
        return false;
    }

    pxDock         = &s_axCards[u8CardIdx].axDocks[u8DockInCard];
    pxDock->eRed   = LED_STATE_OFF;
    pxDock->eGreen = LED_STATE_OFF;
    pxDock->eBlue  = LED_STATE_OFF;
    pxDock->bDirty = true;

    return true;
}

/* ------------------------------------------------------------------------- */

void LedMgr_SetAllOff(void)
{
    uint8_t u8C;
    uint8_t u8D;

    for (u8C = 0U; u8C < (uint8_t)LED_CARD_COUNT; u8C++)
    {
        for (u8D = 0U; u8D < (uint8_t)LED_CARD_DOCK_CAPACITY; u8D++)
        {
            LedDockState_t *pxDock = &s_axCards[u8C].axDocks[u8D];
            pxDock->eRed   = LED_STATE_OFF;
            pxDock->eGreen = LED_STATE_OFF;
            pxDock->eBlue  = LED_STATE_OFF;
            pxDock->bDirty = true;
        }
    }
}

/* =========================================================================
 * Public: State getters
 * ========================================================================= */

bool LedMgr_GetDockState(Dock_e eDock, LedDockState_t *pxOut)
{
    uint8_t u8CardIdx;
    uint8_t u8DockInCard;

    if ((pxOut == NULL) || !prv_ResolveDock(eDock, &u8CardIdx, &u8DockInCard))
    {
        return false;
    }

    *pxOut = s_axCards[u8CardIdx].axDocks[u8DockInCard];
    return true;
}

/* ------------------------------------------------------------------------- */

uint16_t LedMgr_GetRegisterValue(Dock_e eDock)
{
    uint8_t u8CardIdx;
    uint8_t u8DockInCard;

    if (!prv_ResolveDock(eDock, &u8CardIdx, &u8DockInCard))
    {
        return 0U;
    }

    return prv_EncodeRegister(&s_axCards[u8CardIdx].axDocks[u8DockInCard]);
}

/* ------------------------------------------------------------------------- */

uint8_t LedMgr_GetCardRegisters(uint8_t u8CardIndex, uint16_t *pu16RegBuf)
{
    uint8_t u8D;

    if ((u8CardIndex >= (uint8_t)LED_CARD_COUNT) || (pu16RegBuf == NULL))
    {
        return 0U;
    }

    for (u8D = 0U; u8D < (uint8_t)LED_CARD_DOCK_CAPACITY; u8D++)
    {
        pu16RegBuf[u8D] = prv_EncodeRegister(&s_axCards[u8CardIndex].axDocks[u8D]);
    }

    return (uint8_t)LED_CARD_DOCK_CAPACITY;
}

/* ------------------------------------------------------------------------- */

uint8_t LedMgr_GetCardSlaveAddr(uint8_t u8CardIndex)
{
    if (u8CardIndex >= (uint8_t)LED_CARD_COUNT)
    {
        return 0xFFU;   /* invalid sentinel */
    }
    return s_axCards[u8CardIndex].u8SlaveAddr;
}

/* ------------------------------------------------------------------------- */

bool LedMgr_IsCardDirty(uint8_t u8CardIndex)
{
    uint8_t u8D;

    if (u8CardIndex >= (uint8_t)LED_CARD_COUNT)
    {
        return false;
    }

    for (u8D = 0U; u8D < (uint8_t)LED_CARD_DOCK_CAPACITY; u8D++)
    {
        if (s_axCards[u8CardIndex].axDocks[u8D].bDirty)
        {
            return true;
        }
    }
    return false;
}

/* ------------------------------------------------------------------------- */

void LedMgr_MarkCardClean(uint8_t u8CardIndex)
{
    uint8_t u8D;

    if (u8CardIndex >= (uint8_t)LED_CARD_COUNT)
    {
        return;
    }

    for (u8D = 0U; u8D < (uint8_t)LED_CARD_DOCK_CAPACITY; u8D++)
    {
        s_axCards[u8CardIndex].axDocks[u8D].bDirty = false;
    }
}

/* =========================================================================
 * Public: Diagnostics
 * ========================================================================= */

void LedMgr_PrintState(void)
{
    static const char * const s_apszState[LED_STATE_MAX] =
    {
        "OFF", "STEADY", "SLOW", "MED", "FAST"
    };
    static const char * const s_apszColor[LED_COLOR_MAX] =
    {
        "R", "G", "B"
    };

    uint8_t u8C;
    uint8_t u8D;

    SYS_CONSOLE_PRINT("=== LED Manager State ===\r\n");

    for (u8C = 0U; u8C < (uint8_t)LED_CARD_COUNT; u8C++)
    {
        SYS_CONSOLE_PRINT("  Card %u (slave 0x%02X):\r\n",
                           (unsigned)(u8C + 1U),
                           (unsigned)s_axCards[u8C].u8SlaveAddr);

        for (u8D = 0U; u8D < (uint8_t)LED_CARD_DOCK_CAPACITY; u8D++)
        {
            const LedDockState_t *pxDock = &s_axCards[u8C].axDocks[u8D];
            uint8_t u8GlobalDock = (u8C * (uint8_t)LED_CARD_DOCK_CAPACITY) + u8D + 1U;

            SYS_CONSOLE_PRINT(
                "    Dock%u: %s=%s  %s=%s  %s=%s  reg=0x%04X  %s\r\n",
                (unsigned)u8GlobalDock,
                s_apszColor[LED_COLOR_RED],
                (pxDock->eRed   < LED_STATE_MAX) ? s_apszState[pxDock->eRed]   : "?",
                s_apszColor[LED_COLOR_GREEN],
                (pxDock->eGreen < LED_STATE_MAX) ? s_apszState[pxDock->eGreen] : "?",
                s_apszColor[LED_COLOR_BLUE],
                (pxDock->eBlue  < LED_STATE_MAX) ? s_apszState[pxDock->eBlue]  : "?",
                (unsigned)prv_EncodeRegister(pxDock),
                pxDock->bDirty ? "[dirty]" : "[clean]");
        }
    }

    SYS_CONSOLE_PRINT("=========================\r\n");
}

bool RS485_Output_1_Set(LedState_t eState)
{   
    uint8_t u8DockNo = DOCK_1 - 1U; /* Convert to 0-based index */
    return LedMgr_SetChannel(u8DockNo, LED_COLOR_RED, eState);
}

bool RS485_Output_2_Set(LedState_t eState)
{   
    uint8_t u8DockNo = DOCK_1 - 1U; /* Convert to 0-based index */
    return LedMgr_SetChannel(u8DockNo, LED_COLOR_GREEN, eState);
}

bool RS485_Output_3_Set(LedState_t eState)
{   
    uint8_t u8DockNo = DOCK_1 - 1U; /* Convert to 0-based index */
    return LedMgr_SetChannel(u8DockNo, LED_COLOR_BLUE, eState);
}

bool RS485_Output_4_Set(LedState_t eState)
{   
    uint8_t u8DockNo = DOCK_2 - 1U; /* Convert to 0-based index */
    return LedMgr_SetChannel(u8DockNo, LED_COLOR_RED, eState);
}

bool RS485_Output_5_Set(LedState_t eState)
{   
    uint8_t u8DockNo = DOCK_2 - 1U; /* Convert to 0-based index */
    return LedMgr_SetChannel(u8DockNo, LED_COLOR_GREEN, eState);
}

bool RS485_Output_6_Set(LedState_t eState)
{   
    uint8_t u8DockNo = DOCK_2 - 1U; /* Convert to 0-based index */
    return LedMgr_SetChannel(u8DockNo, LED_COLOR_BLUE, eState);
}

bool RS485_Output_7_Set(LedState_t eState)
{   
    uint8_t u8DockNo = DOCK_3 - 1U; /* Convert to 0-based index */
    return LedMgr_SetChannel(u8DockNo, LED_COLOR_RED, eState);
}

bool RS485_Output_8_Set(LedState_t eState)
{   
    uint8_t u8DockNo = DOCK_3 - 1U; /* Convert to 0-based index */
    return LedMgr_SetChannel(u8DockNo, LED_COLOR_GREEN, eState);
}

bool RS485_Output_9_Set(LedState_t eState)
{   
    uint8_t u8DockNo = DOCK_3 - 1U; /* Convert to 0-based index */
    return LedMgr_SetChannel(u8DockNo, LED_COLOR_BLUE, eState);
}

bool RS485_Output_10_Set(LedState_t eState)
{   
    uint8_t u8DockNo = DOCK_4 - 1U; /* Convert to 0-based index */
    return LedMgr_SetChannel(u8DockNo, LED_COLOR_RED, eState);
}

bool RS485_Output_11_Set(LedState_t eState)
{   
    uint8_t u8DockNo = DOCK_4 - 1U; /* Convert to 0-based index */
    return LedMgr_SetChannel(u8DockNo, LED_COLOR_GREEN, eState);
}

bool RS485_Output_12_Set(LedState_t eState)
{   
    uint8_t u8DockNo = DOCK_4 - 1U; /* Convert to 0-based index */
    return LedMgr_SetChannel(u8DockNo, LED_COLOR_BLUE, eState);
}

bool RS485_Output_13_Set(LedState_t eState)
{   
    uint8_t u8DockNo = DOCK_5 - 1U; /* Convert to 0-based index */
    return LedMgr_SetChannel(u8DockNo, LED_COLOR_RED, eState);
}

bool RS485_Output_14_Set(LedState_t eState)
{   
    uint8_t u8DockNo = DOCK_5 - 1U; /* Convert to 0-based index */
    return LedMgr_SetChannel(u8DockNo, LED_COLOR_GREEN, eState);
}

bool RS485_Output_15_Set(LedState_t eState)
{   
    uint8_t u8DockNo = DOCK_5 - 1U; /* Convert to 0-based index */
    return LedMgr_SetChannel(u8DockNo, LED_COLOR_BLUE, eState);
}

bool RS485_Output_16_Set(LedState_t eState)
{   
    uint8_t u8DockNo = DOCK_6 - 1U; /* Convert to 0-based index */
    return LedMgr_SetChannel(u8DockNo, LED_COLOR_RED, eState);
}

bool RS485_Output_17_Set(LedState_t eState)
{   
    uint8_t u8DockNo = DOCK_6 - 1U; /* Convert to 0-based index */
    return LedMgr_SetChannel(u8DockNo, LED_COLOR_GREEN, eState);
}

bool RS485_Output_18_Set(LedState_t eState)
{   
    uint8_t u8DockNo = DOCK_6 - 1U; /* Convert to 0-based index */
    return LedMgr_SetChannel(u8DockNo, LED_COLOR_BLUE, eState);
}
/*******************************************************************************
 * End of File
 *******************************************************************************/