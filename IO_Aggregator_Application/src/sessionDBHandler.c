/* ************************************************************************** */
/** @file    sessionDBHandler.c
 *  @brief   Session Database Handler — Implementation
 *
 *  @company BACANCY SYSTEMS PVT. LTD.
 *
 *  @summary
 *    Implements session database reset APIs.
 *
 *  @description
 *    Manages per-dock charging session storage. All session variables are held
 *    in the global `sessionDB[]` array and accessed via macros defined in
 *    sessionDBHandler.h.
 *
 *  Version : 2.0
 *
 *  Changes from v1.0:
 *    - Added bounds guard in SESSION_ResetSession and SESSION_ResetAll
 *    - SESSION_ResetSession now uses memset for the whole struct, which is
 *      safer than resetting fields one-by-one (no risk of forgetting a field)
 *    - Explicit re-application of enum defaults (eChargingState, bPMOnOffStatus)
 *      after memset to ensure correct enum values
 *    - Added SESSION_ResetAll loop safety (strictly < MAX_DOCKS)
 *    - Removed commented-out SESSION_SetChargingState line — unclear intent,
 *      now documented explicitly
 */
/* ************************************************************************** */

/* ============================================================================
 * Includes
 * ========================================================================== */
#include <string.h>
#include "sessionDBHandler.h"

/* ============================================================================
 * Global Session Database
 * ========================================================================== */

/** @brief Per-dock session data store — one entry per physical dock */
SESSION_Data_t sessionDB[MAX_DOCKS];

/* ============================================================================
 * Reset Functions
 * ========================================================================== */

/**
 * @brief  Reset PM-related fields for a given dock to safe defaults.
 *
 *         Clears: PM RX status, temperature, fault code, fault bitmap,
 *                 output voltage, output current, set voltage, set current,
 *                 output power.
 *
 * @param  idx  Dock index. Must satisfy: 0 < idx < MAX_DOCKS.
 *              COMPARTMENT (0) is not a valid dock for PM data.
 */
void SESSION_ResetPMData(uint8_t idx)
{
    if (idx > SESSION_GetMaxDocks())
    {
        return; /* Guard against invalid index */
    }

    SESSION_SetPMRxStatus(idx,      0U);
    SESSION_SetPMTemperature(idx,   0U);
    SESSION_SetPMFaultCode(idx,     0U);
    SESSION_SetPMFaultBitmap(idx,   0UL);
    SESSION_SetPmOutputVoltage(idx, 0.0f);
    SESSION_SetPmOutputCurrent(idx, 0.0f);
    SESSION_SetPmVoltageSetpoint(idx,    0.0f);
    SESSION_SetPmCurrentSetpoint(idx,    0.0f);
    SESSION_SetOutputPower(idx,     0.0f);
}

/**
 * @brief  Reset BMS-related fields for a given dock to safe defaults.
 *
 *         Clears: BMS RX status, temperature, fault bitmap,
 *                 demand voltage, demand current, current SOC, initial SOC.
 *
 * @param  idx  Dock index. Must satisfy: 0 < idx < MAX_DOCKS.
 */
void SESSION_ResetBMSData(uint8_t idx)
{
    if (idx > SESSION_GetMaxDocks())
    {
        return;
    }

    SESSION_SetBMSRxStatus(idx,       0U);
    SESSION_SetBMSTemperature(idx,    0U);
    SESSION_SetBMSFaultBitmap(idx,    0UL);
    SESSION_SetBMSDemandVoltage(idx,  0.0f);
    SESSION_SetBMSDemandCurrent(idx,  0.0f);
    SESSION_SetCurrentSoc(idx,        0U);
    SESSION_SetInitialSoc(idx,        0U);
}

/**
 * @brief  Reset temperature-related fields for a given dock.
 *
 * @param  idx  Dock index. Must satisfy: idx < MAX_DOCKS.
 */
void SESSION_ResetTempData(uint8_t idx)
{
    if (idx > SESSION_GetMaxDocks())
    {
        return;
    }

    SESSION_SetDockTemperature(idx, 0U);
}

/**
 * @brief  Reset the entire session for one dock to safe defaults.
 *
 *         Sequence:
 *           1. memset the whole SESSION_Data_t to zero (clears all numeric fields)
 *           2. Re-apply enum defaults that zero may not represent correctly
 *
 *         Charging state is intentionally reset to CH_STATE_INIT (= 0) so the
 *         state machine restarts cleanly on the next cycle. If you want to
 *         preserve the state across a session reset (e.g. only reset energy
 *         counters), call the sub-reset functions individually instead.
 *
 * @param  idx  Dock index. Must satisfy: idx < MAX_DOCKS.
 */
void SESSION_ResetSession(uint8_t idx)
{
    if (idx > SESSION_GetMaxDocks())
    {
        return;
    }

    /* Zero the entire struct — clears all numeric and flag fields in one shot */
    (void)memset(&sessionDB[idx], 0, sizeof(SESSION_Data_t));

    /* Re-apply enum defaults that require a specific non-zero starting value */
    sessionDB[idx].eChargingState  = CH_STATE_INIT;
    sessionDB[idx].bPMOnOffStatus  = RECTIFIER_OFF;
}

/**
 * @brief  Reset all dock sessions to safe defaults.
 *
 *         Iterates from index 0 (COMPARTMENT) through MAX_DOCKS - 1,
 *         calling SESSION_ResetSession for each slot.
 *
 *         Call once at system startup before any task accesses sessionDB.
 */
void SESSION_ResetAll(void)
{
    for (uint8_t idx = 0U; idx <= (uint8_t)SESSION_GetMaxDocks(); idx++)
    {
        SESSION_ResetSession(idx);
    }
}

/* ************************************************************************** */
/* End of File                                                                */
/* ************************************************************************** */