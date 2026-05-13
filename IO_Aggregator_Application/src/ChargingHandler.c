/******************************************************************************
 * File Name   : ChargingHandler.c
 * Description : Charging Handler module implementation
 *
 * Dual-protocol EV charging state machine supporting:
 *   - PROTOCOL_17017_25  : LEVDC ISO 17017-25 standard CAN protocol
 *   - PROTOCOL_TVS_PROP  : TVS proprietary CAN protocol
 *
 * Active protocol is selected at compile-time in ChargingHandler.h via:
 *   #define CHARGING_PROTOCOL   PROTOCOL_17017_25   (or PROTOCOL_TVS_PROP)
 *
 * State Machine Flow (both protocols follow the same states):
 *   INIT → AUTH_SUCCESS → PARAM_VALIDATE → CONNECTION_CONFIRMED
 *        → INITIALIZE → PRECHARGE → CHARGING → SHUTDOWN
 *        → SESSION_COMPLETE → INIT
 *                          ↘ ERROR ↗
 *
 * Author  : Sarang Parmar
 * Date    : 16-Mar-2026
 * Version : 2.0
 ******************************************************************************/

/******************************************************************************
 * Includes
 ******************************************************************************/
#include "ChargingHandler.h"
#include "definitions.h"
#include "configuration.h"
#include "device.h"
#include "ChargingCommunicationHandler.h"

/******************************************************************************
 * Task Configuration
 ******************************************************************************/
#define CHARGING_TASK_NAME              "CHARGING_TASK"
#define CHARGING_TASK_STACK_SIZE_WORDS  (1024U)
#define CHARGING_TASK_PRIORITY          (tskIDLE_PRIORITY + 2U)
#define CHARGING_TASK_DELAY_MS          (1000U)

/******************************************************************************
 * Fault / Timing Configuration
 ******************************************************************************/
#define FAULT_CHECK_COOLDOWN_MS         (10000U) /**< Cooldown between fault checks (ms)     */
#define ZERO_CURRENT_FAULT_THRESHOLD_MS (30000U) /**< Duration of zero-current before fault  */
#define PRECHARGE_FAIL_THRESHOLD_MS     (30000U) /**< Max time allowed in pre-charge phase   */
#define BMS_COMMS_TIMEOUT_MS            (10000U) /**< BMS timeout before comm fault          */
#define PM_COMMS_TIMEOUT_MS             (10000U) /**< PM  timeout before comm fault          */
#define BMS_FAULT_DEBOUNCE_MS           (2000U)  /**< BMS fault must persist this long       */
#define PM_FAULT_DEBOUNCE_MS            (2000U)  /**< PM  fault must persist this long       */
#define ZERO_CURRENT_THRESHOLD_A        (0.5f)   /**< Current below this = zero-current fault*/
#define ENERGY_CALC_INTERVAL_MS         (1000U)  /**< Energy calculation interval            */
#define PRINT_INTERVAL_MS               (10000U) /**< Console print interval                 */

/******************************************************************************
 * Static Variables
 ******************************************************************************/
static TaskHandle_t xCHARGING_TASK = NULL;

/******************************************************************************
 * Private Function Prototypes — Task & Top-Level
 ******************************************************************************/
static void CHARGING_TASK(void *pvParameters);
static void Charging_StateMachine(uint8_t u8DockNo);
static void vChargingProcessHandler(uint8_t u8DockNo);
static void vUpdateVehiclePMInfo(uint8_t u8DockNo);
static void vEnergyTimeCalculation(uint8_t u8DockNo);
static void updateSystemState(uint8_t u8DockNo);

/******************************************************************************
 * Private Function Prototypes — LED
 ******************************************************************************/
void vSetLedState(uint8_t u8DockNo, uint8_t ledColor, uint8_t ledState);

/******************************************************************************
 * Private Function Prototypes — Stop & Fault
 ******************************************************************************/
static bool bCheckStopCondition(uint8_t u8DockNo);
static bool bCheckFaultCondition(uint8_t u8DockNo);
static bool bCheckEStopFault(uint8_t u8DockNo);
static bool bCheckBMSFault(uint8_t u8DockNo);
static bool bCheckPMFault(uint8_t u8DockNo);
static bool bCheckBMSStatus(uint8_t u8DockNo);
static bool bCheckPMStatus(uint8_t u8DockNo);
static bool bCheckZeroCurrentFault(uint8_t u8DockNo);
static bool bCheckPrechargeFailure(uint8_t u8DockNo);
static bool bGetBMSFaultStatus(uint8_t u8DockNo);
static bool bGetPMFaultStatus(uint8_t u8DockNo);

/******************************************************************************
 * Private Function Prototypes — Debug / Print
 ******************************************************************************/
static const char *CH_GetStateString(CH_State_e state);
static void vPrintSystemData(uint8_t u8DockNo);
static void vPrintStateAndDeviceInfo(uint8_t u8DockNo);
static void vPrintChargingInfo(uint8_t u8DockNo);


/* ============================================================
 * SECTION 1: PROTOCOL-SPECIFIC LIVE DATA STORES & GET/SET
 * ============================================================ */

/* ----------------------------------------------------------
 * 1A. LEVDC ISO 17017-25 Protocol Data Store
 * ---------------------------------------------------------- */
#if (CHARGING_PROTOCOL == PROTOCOL_17017_25)

ChargingMsgFrameInfo_t Charging_LiveInfo[MAX_DOCKS];

/**
 * @brief  Thread-safe read/write of the LEVDC live CAN frame for one dock.
 *
 * @param  u8DockNo    Dock index (0 to MAX_DOCKS-1)
 * @param  psData      Pointer to caller's ChargingMsgFrameInfo_t buffer
 * @param  u8Operation SET_PARA = write live store, GET_PARA = read live store
 * @return true on success, false on NULL pointer or invalid dock index
 */
bool bGetSetLevdcBMSData(uint8_t u8DockNo,
                         ChargingMsgFrameInfo_t *psData,
                         uint8_t u8Operation)
{
    bool bRet = false;

    if ((psData == NULL) || (u8DockNo > SESSION_GetMaxDocks()))
    {
        return false;
    }

    taskENTER_CRITICAL();
    switch (u8Operation)
    {
        case SET_PARA:
            memcpy((void *)&Charging_LiveInfo[u8DockNo],
                   (const void *)psData,
                   sizeof(ChargingMsgFrameInfo_t));
            bRet = true;
            break;

        case GET_PARA:
            memcpy((void *)psData,
                   (const void *)&Charging_LiveInfo[u8DockNo],
                   sizeof(ChargingMsgFrameInfo_t));
            bRet = true;
            break;

        default:
            bRet = false;
            break;
    }
    taskEXIT_CRITICAL();
    return bRet;
}

#endif /* PROTOCOL_17017_25 */

/* ----------------------------------------------------------
 * 1B. TVS Proprietary Protocol Data Store
 * ---------------------------------------------------------- */
#if (CHARGING_PROTOCOL == PROTOCOL_TVS_PROP)

TVS_MsgFrameInfo_t TVS_LiveInfo[MAX_DOCKS];

/* Compile-time struct size assertions — all CAN frames must be exactly 8 bytes */
static_assert(sizeof(TVS_Tx90_Info_t)          == 8U, "TVS_Tx90_Info_t must be 8 bytes");
static_assert(sizeof(TVS_Tx91_ChargeProfile_t) == 8U, "TVS_Tx91_ChargeProfile_t must be 8 bytes");
static_assert(sizeof(TVS_Tx92_FMVersionInfo_t) == 8U, "TVS_Tx92_FMVersionInfo_t must be 8 bytes");
static_assert(sizeof(TVS_Rx100_Status_t)       == 8U, "TVS_Rx100_Status_t must be 8 bytes");
static_assert(sizeof(TVS_Rx101_Profile_t)      == 8U, "TVS_Rx101_Profile_t must be 8 bytes");

/**
 * @brief  Thread-safe read/write of the TVS live CAN frame for one dock.
 *
 * @param  u8DockNo    Dock index (0 to MAX_DOCKS-1)
 * @param  psData      Pointer to caller's TVS_MsgFrameInfo_t buffer
 * @param  u8Operation SET_PARA = write live store, GET_PARA = read live store
 * @return true on success, false on NULL pointer or invalid dock index
 */
bool bGetSetTVSBMSData(uint8_t u8DockNo,
                       TVS_MsgFrameInfo_t *psData,
                       uint8_t u8Operation)
{
    bool bRet = false;

    if ((psData == NULL) || (u8DockNo > SESSION_GetMaxDocks()))
    {
        return false;
    }

    taskENTER_CRITICAL();
    switch (u8Operation)
    {
        case SET_PARA:
            memcpy((void *)&TVS_LiveInfo[u8DockNo],
                   (const void *)psData,
                   sizeof(TVS_MsgFrameInfo_t));
            bRet = true;
            break;

        case GET_PARA:
            memcpy((void *)psData,
                   (const void *)&TVS_LiveInfo[u8DockNo],
                   sizeof(TVS_MsgFrameInfo_t));
            bRet = true;
            break;

        default:
            bRet = false;
            break;
    }
    taskEXIT_CRITICAL();
    return bRet;
}

#endif /* PROTOCOL_TVS_PROP */


/* ============================================================
 * SECTION 2: PROTOCOL-SPECIFIC STATE MACHINE FUNCTIONS
 * ============================================================ */

/* ----------------------------------------------------------
 * 2A. LEVDC ISO 17017-25 State Functions
 *     Prefix: v17017_
 * ---------------------------------------------------------- */
#if (CHARGING_PROTOCOL == PROTOCOL_17017_25)

/* Private prototypes */
static void v17017_SendInitReq        (uint8_t u8DockNo);
static void v17017_AuthSuccess        (uint8_t u8DockNo);
static void v17017_ValidateParameters (uint8_t u8DockNo);
static void v17017_ConnectionConfirmed(uint8_t u8DockNo);
static void v17017_InitializeState    (uint8_t u8DockNo);
static void v17017_PreChargingState   (uint8_t u8DockNo);
static void v17017_StartCharging      (uint8_t u8DockNo);
static void v17017_Shutdown           (uint8_t u8DockNo);
static void v17017_SessionComplete    (uint8_t u8DockNo);
static void v17017_SessionError       (uint8_t u8DockNo);
static bool bIsEvReadyForCharging     (uint8_t u8DockNo);

/**
 * @brief  Check whether EV is ready for energy transfer.
 *         Reads 0x500 frame and evaluates EV status bits.
 */
static bool bIsEvReadyForCharging(uint8_t u8DockNo)
{
    bool bRet = false;
    ChargingMsgFrameInfo_t sFrame = {0};
    (void)bGetSetLevdcBMSData(u8DockNo, &sFrame, GET_PARA);

    if (sFrame.LevdcRX_500ID_Info.u8EvChargingEnable)
    {
        SYS_CONSOLE_PRINT("G%d: EV Charging Enabled\r\n", (int)u8DockNo);
        bRet = true;
    }
    else if (sFrame.LevdcRX_500ID_Info.u8EvConStatus          ||
             sFrame.LevdcRX_500ID_Info.u8EvChargingPosition   ||
             sFrame.LevdcRX_500ID_Info.u8WaitReqToEngTransfer ||
             sFrame.LevdcRX_500ID_Info.u8EnergyTransferError)
    {
        SYS_CONSOLE_PRINT("G%d: EV not ready (ConStatus/Position/WaitReq/TransferErr)\r\n", (int)u8DockNo);
        bRet = false;
    }
    SYS_CONSOLE_PRINT("G%d: bIsEvReadyForCharging = %d\r\n", (int)u8DockNo, (int)bRet);
    return bRet;
}

/**
 * @brief CH_STATE_INIT — Broadcast EVSE identity and wait for EV authentication.
 *
 * Populates:
 *   0x508 - u8EvseStopCtrl = EVSE_NOERROR
 *   0x509 - u8ControlProtocolNum = 1
 *   0x510 - u8EVSEVolatageControlOpt = VOLTAGE_CONTROL_ENABLED
 *
 * Transition: → CH_STATE_AUTH_SUCCESS when AUTH command received from EV.
 */
static void v17017_SendInitReq(uint8_t u8DockNo)
{
    ChargingMsgFrameInfo_t sFrame = {0};

    sFrame.LevdcTX_508ID_Info.u8EvseStopCtrl             = EVSE_NOERROR;
    sFrame.LevdcTX_509ID_Info.u8ControlProtocolNum        = 1U;
    sFrame.LevdcTX_510ID_Info.u8EVSEVolatageControlOpt   = VOLTAGE_CONTROL_ENABLED;

    (void)bGetSetLevdcBMSData(u8DockNo, &sFrame, SET_PARA);

    if (SESSION_GetAuthenticationCommand(u8DockNo) == 1U)
    {
        SESSION_SetChargingState(u8DockNo, CH_STATE_AUTH_SUCCESS);
    }
}

/**
 * @brief CH_STATE_AUTH_SUCCESS — Authentication acknowledged by EV.
 *
 * Transition: → CH_STATE_PARAM_VALIDATE immediately.
 */
static void v17017_AuthSuccess(uint8_t u8DockNo)
{
    SESSION_SetChargingState(u8DockNo, CH_STATE_PARAM_VALIDATE);
}

/**
 * @brief CH_STATE_PARAM_VALIDATE — Validate EV parameters and advertise capability.
 *
 * Guards:
 *   - BMSRxStatus must be true (EV CAN frames received)
 *
 * Populates (0x508):
 *   u8EvseStatus        = EVSE_CHARGING
 *   u16RatedOutputVol   = LEVDC_MAX_VOLTAGE * 10
 *   u16ConfDCvolLimit   = LEVDC_MAX_VOLTAGE * 10
 *   u16AvailOutputCur   = LEVDC_MAX_CURRENT * 10
 *   u8EVCompatible      = EV_COMPATIBLE
 *
 * Transition: → CH_STATE_CONNECTION_CONFIRMED on success.
 */
static void v17017_ValidateParameters(uint8_t u8DockNo)
{
    if (SESSION_GetBMSRxStatus(u8DockNo) == false)
    {
        SESSION_SetStartChargingComm(u8DockNo, false);
        return;
    }

    ChargingMsgFrameInfo_t sFrame = {0};
    (void)bGetSetLevdcBMSData(u8DockNo, &sFrame, GET_PARA);

    sFrame.LevdcTX_508ID_Info.u8EvseStopCtrl         = EVSE_NOERROR;
    sFrame.LevdcTX_508ID_Info.u8EvseStatus           = EVSE_CHARGING;
    sFrame.LevdcTX_508ID_Info.u16RatedOutputVol      = (uint16_t)(LEVDC_MAX_VOLTAGE * FACTOR_10);
    sFrame.LevdcTX_508ID_Info.u16ConfDCvolLimit      = (uint16_t)(LEVDC_MAX_VOLTAGE * FACTOR_10);
    sFrame.LevdcTX_508ID_Info.u16AvailOutputCur      = (uint16_t)(LEVDC_MAX_CURRENT * FACTOR_10);
    sFrame.LevdcTX_508ID_Info.u8EVCompatible         = EV_COMPATIBLE;
    sFrame.LevdcTX_509ID_Info.u8AvailDCOutputPower   = 0xFFU;
    sFrame.LevdcTX_509ID_Info.u16RemainChargeTime    = 0xFFFFU;

    SESSION_SetStartChargingComm(u8DockNo, true);
    SESSION_SetChargingState(u8DockNo, CH_STATE_CONNECTION_CONFIRMED);
    (void)bGetSetLevdcBMSData(u8DockNo, &sFrame, SET_PARA);
}

/**
 * @brief CH_STATE_CONNECTION_CONFIRMED — Latch connector and validate EV demand.
 *
 * Reads from 0x500:
 *   u16DcOutputVolTarget → u16OutputVoltage
 *   u16ReqDcCurrent      → u16OutputCurrent
 *   u16DcOutputVoltLimit → u16BatVoltMaxLimit
 *
 * Condition: If BatVoltLimit ≤ LEVDC_MAX_VOLTAGE AND ReqCurrent ≤ LEVDC_MAX_CURRENT
 *            → advance to CH_STATE_INITIALIZE.
 */
static void v17017_ConnectionConfirmed(uint8_t u8DockNo)
{
    ChargingMsgFrameInfo_t sFrame = {0};
    (void)bGetSetLevdcBMSData(u8DockNo, &sFrame, GET_PARA);

    sFrame.LevdcTX_508ID_Info.u8ConLatchStatus       = GUN_LATCHED;
    sFrame.LevdcTX_508ID_Info.u8ChargingSysError     = EVSE_NOERROR;
    sFrame.LevdcTX_508ID_Info.u8EvseMalFunctionError = EVSE_NOERROR;
    sFrame.LevdcTX_508ID_Info.u8EvseStatus           = EVSE_STANDBY;
    sFrame.LevdcTX_508ID_Info.u8EvseStopCtrl         = EVSE_NOERROR;

    uint16_t u16OutputVoltage   = (uint16_t)(sFrame.LevdcRX_500ID_Info.u16DcOutputVolTarget / FACTOR_10);
    uint16_t u16OutputCurrent   = (uint16_t)(sFrame.LevdcRX_500ID_Info.u16ReqDcCurrent      / FACTOR_10);
    uint16_t u16BatVoltMaxLimit = (uint16_t)(sFrame.LevdcRX_500ID_Info.u16DcOutputVoltLimit  / FACTOR_10);

    (void)bGetSetLevdcBMSData(u8DockNo, &sFrame, SET_PARA);

    SYS_CONSOLE_PRINT("G%d -> Demand V: %d  I: %d  BMS_RX: %d  BatVLim: %d\r\n",
                      (int)u8DockNo,
                      (int)u16OutputVoltage,
                      (int)u16OutputCurrent,
                      (int)SESSION_GetBMSRxStatus(u8DockNo),
                      (int)u16BatVoltMaxLimit);

    if ((u16BatVoltMaxLimit <= LEVDC_MAX_VOLTAGE) && (u16OutputCurrent <= LEVDC_MAX_CURRENT))
    {
        SESSION_SetChargingState(u8DockNo, CH_STATE_INITIALIZE);
    }
}

/**
 * @brief CH_STATE_INITIALIZE — Wait for EV ready-for-charging signal.
 *
 * Reads EV status bits from 0x500 via bIsEvReadyForCharging().
 * Transition: EV ready   → CH_STATE_PRECHARGE
 *             EV not ready → CH_STATE_CONNECTION_CONFIRMED (retry)
 */
static void v17017_InitializeState(uint8_t u8DockNo)
{
    if (bIsEvReadyForCharging(u8DockNo))
    {
        SESSION_SetChargingState(u8DockNo, CH_STATE_PRECHARGE);
    }
    else
    {
        SESSION_SetChargingState(u8DockNo, CH_STATE_CONNECTION_CONFIRMED);
    }
}

/**
 * @brief CH_STATE_PRECHARGE — Signal EVSE ready and turn on rectifier.
 *
 * Populates:
 *   0x508 - u8EVSEReadyForCharge = EVSE_READY_FOR_CHARGE
 *   0x509 - u8AvailDCOutputPower = LEVDC_RATED_DC_OP_POWER
 *   0x509 - u16RemainChargeTime  = EV's estimated charging time
 *
 * Transition: → CH_STATE_CHARGING immediately.
 */
static void v17017_PreChargingState(uint8_t u8DockNo)
{
    ChargingMsgFrameInfo_t sFrame = {0};
    (void)bGetSetLevdcBMSData(u8DockNo, &sFrame, GET_PARA);

    sFrame.LevdcTX_508ID_Info.u8EVSEReadyForCharge      = EVSE_READY_FOR_CHARGE;
    sFrame.LevdcTX_509ID_Info.u8AvailDCOutputPower      = (uint8_t)LEVDC_RATED_DC_OP_POWER;
    sFrame.LevdcTX_509ID_Info.u16RemainChargeTime       = sFrame.LevdcRX_501ID_Info.u16EstimatedChargingTime;

    (void)bGetSetLevdcBMSData(u8DockNo, &sFrame, SET_PARA);

    SESSION_SetPMState(u8DockNo, RECTIFIER_ON);
    SESSION_SetChargingState(u8DockNo, CH_STATE_CHARGING);
}

/**
 * @brief CH_STATE_CHARGING — Main active charging loop.
 *
 * Populates each cycle:
 *   0x508 - u8EvseStatus          = EVSE_CHARGING
 *   0x509 - u16EVSEoutputVoltage  = PM output voltage * 10
 *   0x509 - u16EVSEoutputCurrent  = PM output current * 10
 *
 * Turns on AC and DC relays.
 * Transition: stays in CH_STATE_CHARGING (stop detected by vChargingProcessHandler).
 */
static void v17017_StartCharging(uint8_t u8DockNo)
{
    ChargingMsgFrameInfo_t sFrame = {0};
    (void)bGetSetLevdcBMSData(u8DockNo, &sFrame, GET_PARA);

    sFrame.LevdcTX_508ID_Info.u8EvseStatus          = EVSE_CHARGING;
    sFrame.LevdcTX_509ID_Info.u16EVSEoutputVoltage  = (uint16_t)(SESSION_GetPmOutputVoltage(u8DockNo) * FACTOR_10);
    sFrame.LevdcTX_509ID_Info.u16EVSEoutputCurrent  = (uint16_t)(SESSION_GetPmOutputCurrent(u8DockNo) * FACTOR_10);

    bGPIO_Operation(DO_DC_RELAY_ON, u8DockNo, GPIO_WRITE);

    (void)bGetSetLevdcBMSData(u8DockNo, &sFrame, SET_PARA);

    SESSION_SetChargingState(u8DockNo, CH_STATE_CHARGING);
}

/**
 * @brief CH_STATE_SHUTDOWN — Safe shutdown of EVSE output.
 *
 * Actions:
 *   - Turns off AC and DC relays
 *   - Turns off rectifier
 *   - Signals EVSE standby + gun unlatched to EV
 *   - Waits LEVDC_SHUTOFF_DELAY_MS before clearing live data
 *
 * Transition: → CH_STATE_SESSION_COMPLETE.
 */
static void v17017_Shutdown(uint8_t u8DockNo)
{
    // bGPIO_Operation(DO_AC_RELAY_OFF, u8DockNo);
    bGPIO_Operation(DO_DC_RELAY_OFF, u8DockNo, GPIO_WRITE);
    SESSION_SetPMState(u8DockNo, RECTIFIER_OFF);

    ChargingMsgFrameInfo_t sFrame = {0};
    (void)bGetSetLevdcBMSData(u8DockNo, &sFrame, GET_PARA);

    sFrame.LevdcTX_508ID_Info.u8EvseStatus       = EVSE_STANDBY;
    sFrame.LevdcTX_508ID_Info.u8ConLatchStatus   = GUN_UNLATCHED;
    sFrame.LevdcTX_508ID_Info.u8EvseStopCtrl     = EVSE_ERROR;
    sFrame.LevdcTX_508ID_Info.u16AvailOutputCur  = 0U;
    sFrame.LevdcTX_508ID_Info.u16RatedOutputVol  = 0U;

    (void)bGetSetLevdcBMSData(u8DockNo, &sFrame, SET_PARA);

    vTaskDelay(LEVDC_SHUTOFF_DELAY_MS);
    SESSION_SetStartChargingComm(u8DockNo, false);

    (void)memset(&sFrame, 0, sizeof(sFrame));
    (void)bGetSetLevdcBMSData(u8DockNo, &sFrame, SET_PARA);

    SESSION_SetChargingState(u8DockNo, CH_STATE_SESSION_COMPLETE);
}

/**
 * @brief CH_STATE_SESSION_COMPLETE — End-of-session cleanup.
 *
 * Transition: Fault present → CH_STATE_ERROR
 *             No fault      → CH_STATE_INIT
 */
static void v17017_SessionComplete(uint8_t u8DockNo)
{
    if (SESSION_GetSystemFaultBitmap(u8DockNo) != SYSTEM_FAULT_NONE)
    {
        SESSION_SetChargingState(u8DockNo, CH_STATE_ERROR);
    }
    else
    {
        SESSION_SetChargingState(u8DockNo, CH_STATE_INIT);
    }
}

/**
 * @brief CH_STATE_ERROR — Hold in error until fault clears.
 *
 * Transition: Fault cleared → CH_STATE_INIT
 */
static void v17017_SessionError(uint8_t u8DockNo)
{
    if (SESSION_GetSystemFaultBitmap(u8DockNo) == SYSTEM_FAULT_NONE)
    {
        SESSION_SetChargingState(u8DockNo, CH_STATE_INIT);
    }
}

#endif /* PROTOCOL_17017_25 */


/* ----------------------------------------------------------
 * 2B. TVS Proprietary Protocol State Functions
 *     Prefix: vTVS_
 * ---------------------------------------------------------- */
#if (CHARGING_PROTOCOL == PROTOCOL_TVS_PROP)

/* Private prototypes */
static void vTVS_SendInitReq        (uint8_t u8DockNo);
static void vTVS_AuthSuccess        (uint8_t u8DockNo);
static void vTVS_ValidateParameters (uint8_t u8DockNo);
static void vTVS_ConnectionConfirmed(uint8_t u8DockNo);
static void vTVS_InitializeState    (uint8_t u8DockNo);
static void vTVS_PreChargingState   (uint8_t u8DockNo);
static void vTVS_StartCharging      (uint8_t u8DockNo);
static void vTVS_Shutdown           (uint8_t u8DockNo);
static void vTVS_SessionComplete    (uint8_t u8DockNo);
static void vTVS_SessionError       (uint8_t u8DockNo);

/**
 * @brief CH_STATE_INIT — Broadcast charger identity and firmware version to BMS.
 *
 * Populates (0x90):
 *   u8ChargerType = TVS_CHARGER_TYPE_3KW_DELTA_OFFBOARD
 *   u8ErrorState  = TVS_CHARGER_ERR_NONE
 *   u8Fan/Output/Derating = all OFF/NONE
 *
 * Populates (0x92): full firmware version fields from FW_VERSION_* macros.
 *
 * Transition: → CH_STATE_AUTH_SUCCESS when BMS sets AUTH command = 1.
 */
static void vTVS_SendInitReq(uint8_t u8DockNo)
{
    TVS_MsgFrameInfo_t sFrame = {0};
    (void)bGetSetTVSBMSData(u8DockNo, &sFrame, GET_PARA);

    /* Charger identity (0x90) */
    sFrame.TVS_Tx90_ChargerInfo.u8ChargerType = (uint8_t)TVS_CHARGER_TYPE_3KW_DELTA_OFFBOARD;
    sFrame.TVS_Tx90_ChargerInfo.u8ErrorState  = (uint8_t)TVS_CHARGER_ERR_NONE;
    sFrame.TVS_Tx90_ChargerInfo.u8Fan         = (uint8_t)TVS_FAN_OFF;
    sFrame.TVS_Tx90_ChargerInfo.u8Output      = (uint8_t)TVS_OUTPUT_OFF;
    sFrame.TVS_Tx90_ChargerInfo.u8Derating    = (uint8_t)TVS_DERATING_NONE;

    (void)bGetSetTVSBMSData(u8DockNo, &sFrame, SET_PARA);

    if (SESSION_GetAuthenticationCommand(u8DockNo) == 1U)
    {
        SESSION_SetChargingState(u8DockNo, CH_STATE_AUTH_SUCCESS);
    }
}

/**
 * @brief CH_STATE_AUTH_SUCCESS — BMS authentication acknowledged.
 *
 * Transition: → CH_STATE_PARAM_VALIDATE immediately.
 */
static void vTVS_AuthSuccess(uint8_t u8DockNo)
{
    SESSION_SetChargingState(u8DockNo, CH_STATE_PARAM_VALIDATE);
}

/**
 * @brief CH_STATE_PARAM_VALIDATE — Verify BMS profile received and within limits.
 *
 * Guards:
 *   - BMSRxStatus must be true (0x100 and 0x101 frames received from BMS)
 *
 * Reads from 0x101:
 *   u16MaxChargeVoltage, u16MaxChargeCurrent (decoded via 0.001 factor)
 *
 * Populates (0x91):
 *   u16ChargingVoltage = TVS_MAX_VOLTAGE encoded
 *   u16ChargingCurrent = TVS_MAX_CURRENT encoded
 *
 * Transition: → CH_STATE_CONNECTION_CONFIRMED on success.
 */
static void vTVS_ValidateParameters(uint8_t u8DockNo)
{
    if (SESSION_GetBMSRxStatus(u8DockNo) == false)
    {
        SESSION_SetStartChargingComm(u8DockNo, false);
        return;
    }

    TVS_MsgFrameInfo_t sFrame = {0};
    (void)bGetSetTVSBMSData(u8DockNo, &sFrame, GET_PARA);

    float fMaxVoltage = TVS_DECODE_BMS_PROFILE_VOLTAGE(sFrame.TVS_Rx101_BMSProfile.u16MaxChargeVoltage);
    float fMaxCurrent = TVS_DECODE_BMS_PROFILE_CURRENT(sFrame.TVS_Rx101_BMSProfile.u16MaxChargeCurrent);

    SYS_CONSOLE_PRINT("G%d -> BMS Profile V: %.3f  I: %.3f\r\n",
                      (int)u8DockNo, (double)fMaxVoltage, (double)fMaxCurrent);

    /* Advertise charger capability back to BMS */
    // sFrame.TVS_Tx91_ChargeProfile.u16ChargingVoltage = TVS_ENCODE_CHARGER_VOLTAGE(TVS_MAX_VOLTAGE);
    // sFrame.TVS_Tx91_ChargeProfile.u16ChargingCurrent = TVS_ENCODE_CHARGER_CURRENT(TVS_MAX_CURRENT);
    sFrame.TVS_Tx90_ChargerInfo.u8ErrorState         = (uint8_t)TVS_CHARGER_ERR_NONE;
    sFrame.TVS_Tx90_ChargerInfo.u8Output             = (uint8_t)TVS_OUTPUT_OFF;

    SESSION_SetStartChargingComm(u8DockNo, true);
    SESSION_SetChargingState(u8DockNo, CH_STATE_CONNECTION_CONFIRMED);
    (void)bGetSetTVSBMSData(u8DockNo, &sFrame, SET_PARA);
}

/**
 * @brief CH_STATE_CONNECTION_CONFIRMED — Validate BMS demand against charger limits.
 *
 * Reads from 0x101 (decoded physical values):
 *   fDemandVoltage, fDemandCurrent, fCutOffCurrent, fPreChargeCurrent
 *
 * Stores decoded values in session DB for use by fault/energy monitors.
 *
 * Condition: fDemandVoltage > 0 AND fDemandCurrent > 0
 *            AND both within TVS_MAX_VOLTAGE / TVS_MAX_CURRENT
 *            → advance to CH_STATE_INITIALIZE.
 */
static void vTVS_ConnectionConfirmed(uint8_t u8DockNo)
{
    TVS_MsgFrameInfo_t sFrame = {0};
    (void)bGetSetTVSBMSData(u8DockNo, &sFrame, GET_PARA);

    float fDemandVoltage    = TVS_DECODE_BMS_PROFILE_VOLTAGE(sFrame.TVS_Rx101_BMSProfile.u16MaxChargeVoltage);
    float fDemandCurrent    = TVS_DECODE_BMS_PROFILE_CURRENT(sFrame.TVS_Rx101_BMSProfile.u16MaxChargeCurrent);
    float fCutOffCurrent    = TVS_DECODE_BMS_PROFILE_CURRENT(sFrame.TVS_Rx101_BMSProfile.u16CutOffChargeCurrent);
    float fPreChargeCurrent = TVS_DECODE_BMS_PROFILE_CURRENT(sFrame.TVS_Rx101_BMSProfile.u16PreChargeCurrent);

    SESSION_SetBMSDemandVoltage(u8DockNo, fDemandVoltage);
    SESSION_SetBMSDemandCurrent(u8DockNo, fDemandCurrent);

    SYS_CONSOLE_PRINT("G%d -> Demand V: %.3f  I: %.3f  CutOff: %.3f  PreChg: %.3f\r\n",
                      (int)u8DockNo,
                      (double)fDemandVoltage,
                      (double)fDemandCurrent,
                      (double)fCutOffCurrent,
                      (double)fPreChargeCurrent);

    sFrame.TVS_Tx90_ChargerInfo.u8ErrorState = (uint8_t)TVS_CHARGER_ERR_NONE;
    sFrame.TVS_Tx90_ChargerInfo.u8Output     = (uint8_t)TVS_OUTPUT_OFF;
    (void)bGetSetTVSBMSData(u8DockNo, &sFrame, SET_PARA);

    if ((fDemandVoltage > 0.0f) && (fDemandCurrent > 0.0f) &&
        (fDemandVoltage <= TVS_MAX_VOLTAGE) && (fDemandCurrent <= TVS_MAX_CURRENT))
    {
        SESSION_SetChargingState(u8DockNo, CH_STATE_INITIALIZE);
    }
}

/**
 * @brief CH_STATE_INITIALIZE — Gate to pre-charge once BMS reports no error.
 *
 * Reads from 0x100:
 *   u8ErrorState — must be TVS_BMS_ERR_NONE to proceed.
 *
 * Transition: BMS OK     → CH_STATE_PRECHARGE
 *             BMS error  → CH_STATE_CONNECTION_CONFIRMED (retry)
 */
static void vTVS_InitializeState(uint8_t u8DockNo)
{
    TVS_MsgFrameInfo_t sFrame = {0};
    (void)bGetSetTVSBMSData(u8DockNo, &sFrame, GET_PARA);

    if (sFrame.TVS_Rx100_BMSStatus.u8ErrorState == (uint8_t)TVS_BMS_ERR_NONE)
    {
        SESSION_SetChargingState(u8DockNo, CH_STATE_PRECHARGE);
    }
    else
    {
        SYS_CONSOLE_PRINT("G%d: BMS error in INIT state (%d), retrying\r\n",
                          (int)u8DockNo,
                          (int)sFrame.TVS_Rx100_BMSStatus.u8ErrorState);
        SESSION_SetChargingState(u8DockNo, CH_STATE_CONNECTION_CONFIRMED);
    }
}

/**
 * @brief CH_STATE_PRECHARGE — Apply BMS pre-charge current and enable output.
 *
 * Reads from 0x101:
 *   u16PreChargeCurrent → encoded into 0x91 u16ChargingCurrent
 *   u16MaxChargeVoltage → encoded into 0x91 u16ChargingVoltage
 *
 * Sets (0x90): Fan ON, Output ON, Derating NONE.
 * Turns on rectifier via SESSION_SetPMState.
 *
 * Transition: → CH_STATE_CHARGING immediately.
 */
static void vTVS_PreChargingState(uint8_t u8DockNo)
{
    TVS_MsgFrameInfo_t sFrame = {0};
    (void)bGetSetTVSBMSData(u8DockNo, &sFrame, GET_PARA);

    float fPreChargeCurrent = TVS_DECODE_BMS_PROFILE_CURRENT(sFrame.TVS_Rx101_BMSProfile.u16PreChargeCurrent);
    float fTargetVoltage    = TVS_DECODE_BMS_PROFILE_VOLTAGE(sFrame.TVS_Rx101_BMSProfile.u16MaxChargeVoltage);

    sFrame.TVS_Tx91_ChargeProfile.u16ChargingCurrent = TVS_ENCODE_CHARGER_CURRENT(fPreChargeCurrent);
    sFrame.TVS_Tx91_ChargeProfile.u16ChargingVoltage = TVS_ENCODE_CHARGER_VOLTAGE(fTargetVoltage);
    sFrame.TVS_Tx90_ChargerInfo.u8Output             = (uint8_t)TVS_OUTPUT_ON;
    sFrame.TVS_Tx90_ChargerInfo.u8Fan                = (uint8_t)TVS_FAN_ON;
    sFrame.TVS_Tx90_ChargerInfo.u8Derating           = (uint8_t)TVS_DERATING_NONE;

    SYS_CONSOLE_PRINT("G%d -> PreCharge I: %.3f  V: %.3f\r\n",
                      (int)u8DockNo, (double)fPreChargeCurrent, (double)fTargetVoltage);

    (void)bGetSetTVSBMSData(u8DockNo, &sFrame, SET_PARA);

    SESSION_SetPMState(u8DockNo, RECTIFIER_ON);
    SESSION_SetChargingState(u8DockNo, CH_STATE_CHARGING);
}

/**
 * @brief CH_STATE_CHARGING — Main charging loop: apply demand, report output, monitor.
 *
 * Each cycle:
 *   1. Reads 0x100 (BMS live status): current, voltage, SOC, temp, error
 *   2. Reads 0x101 (BMS profile): demand V, demand I, cut-off I
 *   3. Checks cut-off current — if BMS current ≤ cut-off → SHUTDOWN
 *   4. Applies thermal derating if charger temp ≥ TVS_TEMP_DERATING_THRESHOLD
 *   5. Encodes demand V and I into 0x91 and writes to live store
 *   6. Updates session DB with live PM output and SOC
 *   7. Increments rolling counter in 0x90
 *   8. Turns on AC and DC relays
 *
 * Transition: stays in CH_STATE_CHARGING (stop/fault detected by
 *             vChargingProcessHandler → Charging_StateMachine)
 */
static void vTVS_StartCharging(uint8_t u8DockNo)
{
    TVS_MsgFrameInfo_t sFrame = {0};
    (void)bGetSetTVSBMSData(u8DockNo, &sFrame, GET_PARA);
    float fOutputVoltage = SESSION_GetPmOutputVoltage(u8DockNo);
    float fOutputCurrent = SESSION_GetPmOutputCurrent(u8DockNo);

    /* --- Apply BMS demand to charger profile (0x91) --- */
    sFrame.TVS_Tx91_ChargeProfile.u16ChargingCurrent = TVS_ENCODE_CHARGER_CURRENT(fOutputCurrent);
    sFrame.TVS_Tx91_ChargeProfile.u16ChargingVoltage = TVS_ENCODE_CHARGER_VOLTAGE(fOutputVoltage);

    /* --- Rolling counter increment (0x90) --- */
    sFrame.TVS_Tx90_ChargerInfo.u8Counter++;

    /* --- Enable relays --- */
    // bGPIO_Operation(DO_AC_RELAY_ON, u8DockNo);
    bGPIO_Operation(DO_DC_RELAY_ON, u8DockNo, GPIO_WRITE);
    
    (void)bGetSetTVSBMSData(u8DockNo, &sFrame, SET_PARA);

    /* Stay in CHARGING — stop/fault path handled by vChargingProcessHandler */
    SESSION_SetChargingState(u8DockNo, CH_STATE_CHARGING);
}

/**
 * @brief CH_STATE_SHUTDOWN — Safe ramp-down and output disable.
 *
 * Actions:
 *   - Turns off AC and DC relays
 *   - Turns off rectifier
 *   - Zeroes charge profile in 0x91
 *   - Disables output and fan in 0x90
 *   - Waits TVS_SHUTOFF_DELAY_MS
 *   - Clears entire live frame
 *
 * Transition: → CH_STATE_SESSION_COMPLETE.
 */
static void vTVS_Shutdown(uint8_t u8DockNo)
{
    // bGPIO_Operation(DO_AC_RELAY_OFF, u8DockNo);
    bGPIO_Operation(DO_DC_RELAY_OFF, u8DockNo, GPIO_WRITE);
    SESSION_SetPMState(u8DockNo, RECTIFIER_OFF);

    TVS_MsgFrameInfo_t sFrame = {0};
    (void)bGetSetTVSBMSData(u8DockNo, &sFrame, GET_PARA);

    /* Zero out charge profile (0x91) */
    sFrame.TVS_Tx91_ChargeProfile.u16ChargingCurrent = 0U;
    sFrame.TVS_Tx91_ChargeProfile.u16ChargingVoltage = 0U;

    /* Safe output state (0x90) */
    sFrame.TVS_Tx90_ChargerInfo.u8Output   = (uint8_t)TVS_OUTPUT_OFF;
    sFrame.TVS_Tx90_ChargerInfo.u8Fan      = (uint8_t)TVS_FAN_OFF;
    sFrame.TVS_Tx90_ChargerInfo.u8Derating = (uint8_t)TVS_DERATING_NONE;

    (void)bGetSetTVSBMSData(u8DockNo, &sFrame, SET_PARA);

    vTaskDelay(TVS_SHUTOFF_DELAY_MS);
    SESSION_SetStartChargingComm(u8DockNo, false);

    (void)memset(&sFrame, 0, sizeof(sFrame));
    (void)bGetSetTVSBMSData(u8DockNo, &sFrame, SET_PARA);

    SESSION_SetChargingState(u8DockNo, CH_STATE_SESSION_COMPLETE);
}

/**
 * @brief CH_STATE_SESSION_COMPLETE — Post-session cleanup and fault check.
 *
 * Transition: Fault present → CH_STATE_ERROR
 *             No fault      → CH_STATE_INIT
 */
static void vTVS_SessionComplete(uint8_t u8DockNo)
{
    if (SESSION_GetSystemFaultBitmap(u8DockNo) != SYSTEM_FAULT_NONE)
    {
        SESSION_SetChargingState(u8DockNo, CH_STATE_ERROR);
    }
    else
    {
        SESSION_SetChargingState(u8DockNo, CH_STATE_INIT);
    }
}

/**
 * @brief CH_STATE_ERROR — Hold in error state until system fault bitmap clears.
 *
 * Transition: Fault cleared → CH_STATE_INIT
 */
static void vTVS_SessionError(uint8_t u8DockNo)
{
    if (SESSION_GetSystemFaultBitmap(u8DockNo) == SYSTEM_FAULT_NONE)
    {
        SESSION_SetChargingState(u8DockNo, CH_STATE_INIT);
    }
}

#endif /* PROTOCOL_TVS_PROP */


/* ============================================================
 * SECTION 3: PROTOCOL-SELECTED STATE MACHINE DISPATCHER
 * ============================================================ */

/**
 * @brief  Routes the current charging state to the correct protocol handler.
 *
 *         The active protocol (17017-25 or TVS) is selected at compile-time.
 *         Both protocols expose the same ten state handlers mapped to the
 *         same CH_State_e values, so the dispatcher is identical for both.
 *
 * @param  u8DockNo  Dock index (0 to MAX_DOCKS-1)
 */
static void Charging_StateMachine(uint8_t u8DockNo)
{
#if (CHARGING_PROTOCOL == PROTOCOL_17017_25)
    switch (SESSION_GetChargingState(u8DockNo))
    {
        case CH_STATE_INIT:                 v17017_SendInitReq(u8DockNo);         break;
        case CH_STATE_AUTH_SUCCESS:         v17017_AuthSuccess(u8DockNo);         break;
        case CH_STATE_PARAM_VALIDATE:       v17017_ValidateParameters(u8DockNo);  break;
        case CH_STATE_CONNECTION_CONFIRMED: v17017_ConnectionConfirmed(u8DockNo); break;
        case CH_STATE_INITIALIZE:           v17017_InitializeState(u8DockNo);     break;
        case CH_STATE_PRECHARGE:            v17017_PreChargingState(u8DockNo);    break;
        case CH_STATE_CHARGING:             v17017_StartCharging(u8DockNo);       break;
        case CH_STATE_SHUTDOWN:             v17017_Shutdown(u8DockNo);            break;
        case CH_STATE_SESSION_COMPLETE:     v17017_SessionComplete(u8DockNo);     break;
        case CH_STATE_ERROR:                v17017_SessionError(u8DockNo);        break;
        default: SESSION_SetChargingState(u8DockNo, CH_STATE_INIT);               break;
    }

#elif (CHARGING_PROTOCOL == PROTOCOL_TVS_PROP)
    switch (SESSION_GetChargingState(u8DockNo))
    {
        case CH_STATE_INIT:                 vTVS_SendInitReq(u8DockNo);         break;
        case CH_STATE_AUTH_SUCCESS:         vTVS_AuthSuccess(u8DockNo);         break;
        case CH_STATE_PARAM_VALIDATE:       vTVS_ValidateParameters(u8DockNo);  break;
        case CH_STATE_CONNECTION_CONFIRMED: vTVS_ConnectionConfirmed(u8DockNo); break;
        case CH_STATE_INITIALIZE:           vTVS_InitializeState(u8DockNo);     break;
        case CH_STATE_PRECHARGE:            vTVS_PreChargingState(u8DockNo);    break;
        case CH_STATE_CHARGING:             vTVS_StartCharging(u8DockNo);       break;
        case CH_STATE_SHUTDOWN:             vTVS_Shutdown(u8DockNo);            break;
        case CH_STATE_SESSION_COMPLETE:     vTVS_SessionComplete(u8DockNo);     break;
        case CH_STATE_ERROR:                vTVS_SessionError(u8DockNo);        break;
        default: SESSION_SetChargingState(u8DockNo, CH_STATE_INIT);             break;
    }
#endif
}


/* ============================================================
 * SECTION 4: PROTOCOL-SELECTED VEHICLE & PM INFO UPDATE
 * ============================================================ */

/**
 * @brief  Update session DB with live vehicle demand and SOC values.
 *
 *         Only active during CH_STATE_CHARGING.
 *         For LEVDC: reads demand V/I and SOC from 0x500/0x501.
 *         For TVS:   reads demand V/I and SOC from 0x101/0x100.
 *
 * @param  u8DockNo  Dock index
 */
static void vUpdateVehiclePMInfo(uint8_t u8DockNo)
{
    // if (SESSION_GetChargingState(u8DockNo) != CH_STATE_CHARGING)
    // {
    //     return;
    // }

#if (CHARGING_PROTOCOL == PROTOCOL_17017_25)
    ChargingMsgFrameInfo_t sFrame = {0};
    (void)bGetSetLevdcBMSData(u8DockNo, &sFrame, GET_PARA);

    float    fDemandVoltage  = (sFrame.LevdcRX_500ID_Info.u16DcOutputVolTarget * FACTOR_0_1);
    float    fDemandCurrent  = (sFrame.LevdcRX_500ID_Info.u16ReqDcCurrent      * FACTOR_0_1);
    uint16_t u16EstChrgTime  = sFrame.LevdcRX_501ID_Info.u16EstimatedChargingTime;
    uint16_t u16CurrentSoc   = sFrame.LevdcRX_501ID_Info.u8ChargingRate;
    int16_t s16BMSTemp   = 0U; // LEVDC profile doesn't include temperature, could be added if needed

    SESSION_SetBMSDemandVoltage(u8DockNo, fDemandVoltage);
    SESSION_SetBMSDemandCurrent(u8DockNo, fDemandCurrent);
    SESSION_SetPmVoltageSetpoint(u8DockNo, fDemandVoltage);
    SESSION_SetPmCurrentSetpoint(u8DockNo, fDemandCurrent);
    SESSION_SetEstimatedChargingTime(u8DockNo, u16EstChrgTime);
    SESSION_SetCurrentSoc(u8DockNo, u16CurrentSoc);
    SESSION_SetBMSTemperature(u8DockNo, (uint8_t)s16BMSTemp);

    if ((SESSION_GetInitialSoc(u8DockNo) == 0U) && (u16CurrentSoc != 0U))
    {
        SESSION_SetInitialSoc(u8DockNo, u16CurrentSoc);
        SYS_CONSOLE_PRINT("G%d: Initial SOC = %d%%\r\n", (int)u8DockNo, (int)u16CurrentSoc);
    }

#elif (CHARGING_PROTOCOL == PROTOCOL_TVS_PROP)
    TVS_MsgFrameInfo_t sFrame = {0};
    (void)bGetSetTVSBMSData(u8DockNo, &sFrame, GET_PARA);

    /* Decode physical values from BMS profile & status */
    float    fDemandVoltage = TVS_DECODE_BMS_PROFILE_VOLTAGE(sFrame.TVS_Rx101_BMSProfile.u16MaxChargeVoltage);
    float    fDemandCurrent = TVS_DECODE_BMS_PROFILE_CURRENT(sFrame.TVS_Rx101_BMSProfile.u16MaxChargeCurrent);
    uint16_t u16EstChrgTime  = 0U; // TVS profile doesn't include estimated time
    uint16_t u16CurrentSoc  = sFrame.TVS_Rx100_BMSStatus.u8SOC;
    int16_t s16BMSTemp   = TVS_DECODE_BMS_TEMPERATURE(sFrame.TVS_Rx100_BMSStatus.u8Temperature);

    SESSION_SetBMSDemandVoltage(u8DockNo, fDemandVoltage);
    SESSION_SetBMSDemandCurrent(u8DockNo, fDemandCurrent);
    SESSION_SetPmVoltageSetpoint(u8DockNo, fDemandVoltage);
    SESSION_SetPmCurrentSetpoint(u8DockNo, fDemandCurrent);
    SESSION_SetEstimatedChargingTime(u8DockNo, u16EstChrgTime);
    SESSION_SetCurrentSoc(u8DockNo, u16CurrentSoc);
    SESSION_SetBMSTemperature(u8DockNo, (uint8_t)s16BMSTemp);

    if ((SESSION_GetInitialSoc(u8DockNo) == 0U) && (u16CurrentSoc != 0U))
    {
        SESSION_SetInitialSoc(u8DockNo, u16CurrentSoc);
        SYS_CONSOLE_PRINT("G%d: Initial SOC = %d%%\r\n", (int)u8DockNo, (int)u16CurrentSoc);
    }
#endif


}


/* ============================================================
 * SECTION 5: PROTOCOL-SELECTED BMS FAULT STATUS
 * ============================================================ */

/**
 * @brief  Check BMS-side fault conditions.
 *
 *         For LEVDC: evaluates error bits in 0x500 (EnergyTransferError,
 *                    EvChargingStopControl, BatteryOverVoltage, etc.)
 *         For TVS:   evaluates BMS_ErrorState in 0x100
 *                    (0=OK, 1=Stop Charging)
 *
 * @param  u8DockNo  Dock index
 * @return true if BMS fault is active
 */
static bool bGetBMSFaultStatus(uint8_t u8DockNo)
{
#if (CHARGING_PROTOCOL == PROTOCOL_17017_25)
    ChargingMsgFrameInfo_t sFrame = {0};
    if (bGetSetLevdcBMSData(u8DockNo, &sFrame, GET_PARA) == false)
    {
        SYS_CONSOLE_PRINT("G%d: BMS data read failed\r\n", (int)u8DockNo);
        return true;
    }

    uint32_t u32FaultCode = 0U;

    if (sFrame.LevdcRX_500ID_Info.u8EnergyTransferError)
    {
        SYS_CONSOLE_PRINT("G%d: BMS EnergyTransferError\r\n", (int)u8DockNo);
        SET_BIT(u32FaultCode, BMS_FAULT_ENERGY_TRANSFER_ERROR);
    }
    if (sFrame.LevdcRX_500ID_Info.u8BatteryOverVol)
    {
        SYS_CONSOLE_PRINT("G%d: BMS BatteryOverVoltage\r\n", (int)u8DockNo);
        SET_BIT(u32FaultCode, BMS_FAULT_BATTERY_OVERVOLT);
    }
    if (sFrame.LevdcRX_500ID_Info.u8BatteryUnderVol)
    {
        SYS_CONSOLE_PRINT("G%d: BMS BatteryUnderVoltage\r\n", (int)u8DockNo);
        SET_BIT(u32FaultCode, BMS_FAULT_BATTERY_UNDERVOLT);
    }
    if (sFrame.LevdcRX_500ID_Info.u8BatterCurrentDeviError)
    {
        SYS_CONSOLE_PRINT("G%d: BMS CurrentDeviation\r\n", (int)u8DockNo);
        SET_BIT(u32FaultCode, BMS_FAULT_BATTERY_CURRENT_DEVIATION);
    }
    if (sFrame.LevdcRX_500ID_Info.u8BatterVoltageDeviError)
    {
        SYS_CONSOLE_PRINT("G%d: BMS VoltageDeviation (warning)\r\n", (int)u8DockNo);
        SET_BIT(u32FaultCode, BMS_FAULT_BATTERY_VOLTAGE_DEVIATION);
    }
    if (sFrame.LevdcRX_500ID_Info.u8EvChargingStopControl)
    {
        SYS_CONSOLE_PRINT("G%d: BMS ChargingStopControl\r\n", (int)u8DockNo);
        SET_BIT(u32FaultCode, BMS_FAULT_CHARGING_STOP_CONTROL);
    }
    if (sFrame.LevdcRX_500ID_Info.u8HighBatteryTemp)
    {
        SYS_CONSOLE_PRINT("G%d: BMS HighBatteryTemp (warning)\r\n", (int)u8DockNo);
        SET_BIT(u32FaultCode, BMS_WARN_HIGH_TEMP);
    }
    SESSION_SetBMSFaultBitmap(u8DockNo, u32FaultCode);
    return ((u32FaultCode & 0x0000FFFFUL) != 0U);

#elif (CHARGING_PROTOCOL == PROTOCOL_TVS_PROP)
    TVS_MsgFrameInfo_t sFrame = {0};
    (void)bGetSetTVSBMSData(u8DockNo, &sFrame, GET_PARA);

    if (sFrame.TVS_Rx100_BMSStatus.u8ErrorState == (uint8_t)TVS_BMS_ERR_STOP_CHARGING)
    {
        SYS_CONSOLE_PRINT("G%d -> BMS Stop Charging request\r\n", (int)u8DockNo);
        uint32_t u32FaultCode = 0U;
        SET_BIT(u32FaultCode, (1U << 0)); // Define appropriate bit for TVS BMS error
        SESSION_SetBMSFaultBitmap(u8DockNo, u32FaultCode);
        return true;
    }
    return false;
#endif
}


/* ============================================================
 * SECTION 6: COMMON FAULT / STOP HANDLING
 * (protocol-independent — uses session DB and GPIO)
 * ============================================================ */

/**
 * @brief  Top-level stop condition check called each task cycle.
 *
 *         Checks in order:
 *           1. User stop (auth command = 0 during charging)
 *           2. Fault conditions (with cooldown to avoid rapid toggling)
 *
 * @param  u8DockNo  Dock index
 * @return true if a stop/fault condition requires state machine intervention
 */
static bool bCheckStopCondition(uint8_t u8DockNo)
{
    static uint32_t u32FaultCheckCooldownTick[MAX_DOCKS] = {0};
    uint32_t u32CurrentTick = xTaskGetTickCount();
    CH_State_e eState = SESSION_GetChargingState(u8DockNo);

    /* 1. User stop command */
    if ((eState == CH_STATE_CHARGING) && (SESSION_GetAuthenticationCommand(u8DockNo) == 0U))
    {
        SYS_CONSOLE_PRINT("[GUN %d] User Stop Command Received\r\n", (int)u8DockNo);
        SESSION_SetSessionEndReason(u8DockNo, STOP_REASON_MCU_REQUEST);
        return true;
    }

    /* 2. Fault check (with cooldown) */
    if (u32CurrentTick >= u32FaultCheckCooldownTick[u8DockNo])
    {
        if (bCheckFaultCondition(u8DockNo) == true)
        {
            u32FaultCheckCooldownTick[u8DockNo] = u32CurrentTick +
                                                   pdMS_TO_TICKS(FAULT_CHECK_COOLDOWN_MS);
            SESSION_SetSessionEndReason(u8DockNo, STOP_REASON_FAULT);
            return true;
        }
    }
    return false;
}

/**
 * @brief  Evaluate all fault sources and compose system fault bitmap.
 *
 *         Fault sources checked:
 *           - E-Stop GPIO
 *           - BMS fault (protocol-specific via bGetBMSFaultStatus)
 *           - PM fault (fault code bitmap from session DB)
 *           - BMS CAN timeout
 *           - PM CAN timeout
 *           - Zero-current fault (sustained low current during charging)
 *           - Pre-charge timeout
 *
 * @param  u8DockNo  Dock index
 * @return true if any critical fault (bits 0–15 of bitmap) is set
 */
static bool bCheckFaultCondition(uint8_t u8DockNo)
{
    uint32_t u32SystemFault = 0U;

    if (bCheckEStopFault(u8DockNo))
    {
        SET_BIT(u32SystemFault, SYSTEM_FAULT_ESTOP_TRIGGERED);
    }
    // if (bCheckBMSFault(u8DockNo))
    // {
    //     SET_BIT(u32SystemFault, SYSTEM_FAULT_BMS_ERROR);
    // }
    // if (bCheckPMFault(u8DockNo))
    // {
    //     SET_BIT(u32SystemFault, SYSTEM_FAULT_PM_ERROR);
    // }
    if (bCheckBMSStatus(u8DockNo))
    {
        SET_BIT(u32SystemFault, SYSTEM_FAULT_BMS_COMMUNICATION_FAILURE);
    }
    if (bCheckPMStatus(u8DockNo))
    {
        SET_BIT(u32SystemFault, SYSTEM_FAULT_PM_COMMUNICATION_FAILURE);
    }
    // if (bCheckZeroCurrentFault(u8DockNo))
    // {
    //     SET_BIT(u32SystemFault, SYSTEM_FAULT_PM_ZERO_CURRENT);
    // }
    if (bCheckPrechargeFailure(u8DockNo))
    {
        SET_BIT(u32SystemFault, SYSTEM_FAULT_PRECHARGE_FAILURE);
    }

    SESSION_SetSystemFaultBitmap(u8DockNo, u32SystemFault);
    return ((u32SystemFault & 0x0000FFFFUL) != 0U);
}

/** @brief Check Emergency Stop GPIO input. */
static bool bCheckEStopFault(uint8_t u8DockNo)
{
    return (bGPIO_Operation(DI_E_STOP_STATUS, u8DockNo, GPIO_READ) == true);
}

/**
 * @brief  Debounced BMS fault check.
 *         BMS fault must persist for BMS_FAULT_DEBOUNCE_MS before triggering.
 */
static bool bCheckBMSFault(uint8_t u8DockNo)
{
    static uint32_t u32BMSFaultTicks[MAX_DOCKS] = {0};
    uint32_t u32CurrentTick = xTaskGetTickCount();
    CH_State_e eState = SESSION_GetChargingState(u8DockNo);
    bool bBMSFault = bGetBMSFaultStatus(u8DockNo);

    if ((bBMSFault == true) && (eState == CH_STATE_CHARGING))
    {
        if (u32BMSFaultTicks[u8DockNo] == 0U)
        {
            u32BMSFaultTicks[u8DockNo] = u32CurrentTick + pdMS_TO_TICKS(BMS_FAULT_DEBOUNCE_MS);
        }
        if (u32CurrentTick >= u32BMSFaultTicks[u8DockNo])
        {
            SYS_CONSOLE_PRINT("G%d: BMS fault after %lu ms\r\n", (int)u8DockNo, BMS_FAULT_DEBOUNCE_MS);
            u32BMSFaultTicks[u8DockNo] = 0U;
            return true;
        }
    }
    else
    {
        u32BMSFaultTicks[u8DockNo] = 0U;
    }
    return false;
}

/**
 * @brief  Debounced PM fault check.
 *         PM fault must persist for PM_FAULT_DEBOUNCE_MS before triggering.
 */
static bool bCheckPMFault(uint8_t u8DockNo)
{
    static uint32_t u32PMFaultTicks[MAX_DOCKS] = {0};
    uint32_t u32CurrentTick = xTaskGetTickCount();
    CH_State_e eState = SESSION_GetChargingState(u8DockNo);
    bool bPMFault = bGetPMFaultStatus(u8DockNo);

    if ((bPMFault == true) && (eState == CH_STATE_CHARGING))
    {
        if (u32PMFaultTicks[u8DockNo] == 0U)
        {
            u32PMFaultTicks[u8DockNo] = u32CurrentTick + pdMS_TO_TICKS(PM_FAULT_DEBOUNCE_MS);
        }
        if (u32CurrentTick >= u32PMFaultTicks[u8DockNo])
        {
            SYS_CONSOLE_PRINT("G%d: PM fault after %lu ms\r\n", (int)u8DockNo, PM_FAULT_DEBOUNCE_MS);
            u32PMFaultTicks[u8DockNo] = 0U;
            return true;
        }
    }
    else
    {
        u32PMFaultTicks[u8DockNo] = 0U;
    }
    return false;
}

/**
 * @brief  Evaluate PM fault code bitmap from session DB.
 *         Returns true if any critical (bits 0-15) PM fault is active.
 */
static bool bGetPMFaultStatus(uint8_t u8DockNo)
{
    uint32_t u32FaultCode    = SESSION_GetPMFaultCode(u8DockNo);
    uint32_t u32PMFaultBitmap = 0U;

    if (u32FaultCode & (1U << 0))  { SET_BIT(u32PMFaultBitmap, PM_FAULT_INPUT_UNDERVOLT);
        SYS_CONSOLE_PRINT("G%d: PM Input Undervoltage\r\n", (int)u8DockNo); }
    if (u32FaultCode & (1U << 1))  { SET_BIT(u32PMFaultBitmap, PM_FAULT_PHASE_LOSS);
        SYS_CONSOLE_PRINT("G%d: PM Phase Loss\r\n", (int)u8DockNo); }
    if (u32FaultCode & (1U << 2))  { SET_BIT(u32PMFaultBitmap, PM_FAULT_INPUT_OVERVOLT);
        SYS_CONSOLE_PRINT("G%d: PM Input Overvoltage\r\n", (int)u8DockNo); }
    if (u32FaultCode & (1U << 3))  { SET_BIT(u32PMFaultBitmap, PM_FAULT_OUTPUT_OVERVOLT);
        SYS_CONSOLE_PRINT("G%d: PM Output Overvoltage\r\n", (int)u8DockNo); }
    if (u32FaultCode & (1U << 4))  { SET_BIT(u32PMFaultBitmap, PM_FAULT_OUTPUT_OVERCURRENT);
        SYS_CONSOLE_PRINT("G%d: PM Output Overcurrent\r\n", (int)u8DockNo); }
    if (u32FaultCode & (1U << 12)) { SET_BIT(u32PMFaultBitmap, PM_WARN_OUTPUT_UNDERVOLT);
        SYS_CONSOLE_PRINT("G%d: PM Warn Output Undervolt\r\n", (int)u8DockNo); }
    if (u32FaultCode & (1U << 13)) { SET_BIT(u32PMFaultBitmap, PM_WARN_OUTPUT_OVERVOLT);
        SYS_CONSOLE_PRINT("G%d: PM Warn Output Overvolt\r\n", (int)u8DockNo); }
    if (u32FaultCode & (1U << 14)) { SET_BIT(u32PMFaultBitmap, PM_WARN_POWER_LIMIT);
        SYS_CONSOLE_PRINT("G%d: PM Warn Power Limit\r\n", (int)u8DockNo); }

    SESSION_SetPMFaultBitmap(u8DockNo, u32PMFaultBitmap);
    return ((u32PMFaultBitmap & 0x0000FFFFUL) != 0U);
}

/**
 * @brief  BMS CAN communication timeout monitor.
 *         Sets BMSRxStatus false if no frame received within BMS_COMMS_TIMEOUT_MS.
 *         Returns true (fault) only during active charging.
 */
static bool bCheckBMSStatus(uint8_t u8DockNo)
{
    bool bRet = false;
    uint32_t u32CurrentTick    = xTaskGetTickCount();
    uint32_t u32LastBMSRxTick  = SESSION_GetBMSLastRxTime(u8DockNo);
    CH_State_e eState          = SESSION_GetChargingState(u8DockNo);
    uint32_t u32ElapsedMs      = (u32CurrentTick - u32LastBMSRxTick);

    if (u32ElapsedMs >= BMS_COMMS_TIMEOUT_MS)
    {
        if (SESSION_GetBMSRxStatus(u8DockNo) == true)
        {
            SESSION_SetBMSRxStatus(u8DockNo, false);
            SYS_CONSOLE_PRINT("G%d: BMS comms timeout (%lu ms)\r\n", (int)u8DockNo, u32ElapsedMs);
        }
        if (eState == CH_STATE_CHARGING) { bRet = true; }
    }
    else
    {
        if (u32LastBMSRxTick == 0U)
        {
            SESSION_SetBMSRxStatus(u8DockNo, false);
        }
        else if (SESSION_GetBMSRxStatus(u8DockNo) == false)
        {
            SESSION_SetBMSRxStatus(u8DockNo, true);
            SYS_CONSOLE_PRINT("G%d: BMS comms restored (%lu ms)\r\n", (int)u8DockNo, u32ElapsedMs);
        }
    }
    return bRet;
}

/**
 * @brief  PM CAN communication timeout monitor.
 *         Sets PMRxStatus false if no frame received within PM_COMMS_TIMEOUT_MS.
 *         Returns true (fault) only during active charging.
 */
static bool bCheckPMStatus(uint8_t u8DockNo)
{
    bool bRet = false;
    uint32_t u32CurrentTick   = xTaskGetTickCount();
    uint32_t u32LastPMRxTick  = SESSION_GetPMLastRxTime(u8DockNo);
    CH_State_e eState         = SESSION_GetChargingState(u8DockNo);
    uint32_t u32ElapsedMs     = (u32CurrentTick - u32LastPMRxTick);

    // SYS_CONSOLE_PRINT("@1 Dock %u : Last Rx Tick %d\r\n",u8DockNo, SESSION_GetPMLastRxTime(u8DockNo));
    if (u32ElapsedMs >= PM_COMMS_TIMEOUT_MS)
    {
        if (SESSION_GetPMRxStatus(u8DockNo) == true)
        {
            SESSION_SetPMRxStatus(u8DockNo, false);
            SYS_CONSOLE_PRINT("G%d: PM comms timeout (%lu ms)\r\n", (int)u8DockNo, u32ElapsedMs);
        }
        if (eState == CH_STATE_CHARGING) { bRet = true; }
    }
    else
    {
        if (u32LastPMRxTick == 0U)
        {
            SESSION_SetPMRxStatus(u8DockNo, false);
        }
        else if (SESSION_GetPMRxStatus(u8DockNo) == false)
        {
            SESSION_SetPMRxStatus(u8DockNo, true);
            SYS_CONSOLE_PRINT("G%d: PM comms restored (%lu ms)\r\n", (int)u8DockNo, u32ElapsedMs);
        }
    }
    return bRet;
}

/**
 * @brief  Zero-current fault — sustained low current during charging.
 *         PM output current below ZERO_CURRENT_THRESHOLD_A for
 *         ZERO_CURRENT_FAULT_THRESHOLD_MS triggers fault.
 */
static bool bCheckZeroCurrentFault(uint8_t u8DockNo)
{
    static uint32_t u32ZeroCurrentTicks[MAX_DOCKS] = {0};
    uint32_t u32CurrentTick = xTaskGetTickCount();
    float fCurrent = SESSION_GetPmOutputCurrent(u8DockNo);
    CH_State_e eState = SESSION_GetChargingState(u8DockNo);

    if ((eState == CH_STATE_CHARGING) && (fCurrent < ZERO_CURRENT_THRESHOLD_A))
    {
        if (u32ZeroCurrentTicks[u8DockNo] == 0U)
        {
            u32ZeroCurrentTicks[u8DockNo] = u32CurrentTick + pdMS_TO_TICKS(ZERO_CURRENT_FAULT_THRESHOLD_MS);
        }
        if (u32CurrentTick > u32ZeroCurrentTicks[u8DockNo])
        {
            SYS_CONSOLE_PRINT("G%d: Zero current fault (I=%.2fA)\r\n",
                              (int)u8DockNo, (double)fCurrent);
            u32ZeroCurrentTicks[u8DockNo] = 0U;
            return true;
        }
    }
    else
    {
        u32ZeroCurrentTicks[u8DockNo] = 0U;
    }
    return false;
}

/**
 * @brief  Pre-charge timeout — session stuck in pre-charge flow too long.
 *         Fault triggers if not in CHARGING state and AUTH is active
 *         for longer than PRECHARGE_FAIL_THRESHOLD_MS.
 */
static bool bCheckPrechargeFailure(uint8_t u8DockNo)
{
    static uint32_t u32PrechargeFailureTicks[MAX_DOCKS] = {0};
    uint32_t u32CurrentTick = xTaskGetTickCount();

    /* Only active when auth command is present but not yet charging */
    if ((SESSION_GetAuthenticationCommand(u8DockNo) == 0U) ||
        (SESSION_GetChargingState(u8DockNo) == CH_STATE_CHARGING))
    {
        u32PrechargeFailureTicks[u8DockNo] = 0U;
        return false;
    }

    if (u32PrechargeFailureTicks[u8DockNo] == 0U)
    {
        u32PrechargeFailureTicks[u8DockNo] = u32CurrentTick + pdMS_TO_TICKS(PRECHARGE_FAIL_THRESHOLD_MS);
        return false;
    }

    if (xTaskGetTickCount() >= u32PrechargeFailureTicks[u8DockNo])
    {
        SYS_CONSOLE_PRINT("G%d: Pre-charge failure timeout\r\n", (int)u8DockNo);
        u32PrechargeFailureTicks[u8DockNo] = 0U;
        return true;
    }
    return false;
}


/* ============================================================
 * SECTION 7: COMMON ENERGY & TIME CALCULATION
 * ============================================================ */

/**
 * @brief  Calculate and accumulate energy delivered during charging session.
 *
 *         Runs every ENERGY_CALC_INTERVAL_MS during CH_STATE_CHARGING.
 *         Energy (kWh) = (V × I × dt) / (3600 × 1000)
 *         Resets energy counter at CH_STATE_PRECHARGE (session start).
 *         Logs final energy at CH_STATE_SESSION_COMPLETE.
 *
 * @param  u8DockNo  Dock index
 */
static void vEnergyTimeCalculation(uint8_t u8DockNo)
{
    static uint32_t u32PreviousTick[MAX_DOCKS] = {0};
    const float fMinValidVoltage = 5.0f;
    const float fMinValidCurrent = 1.0f;
    CH_State_e eLiveStage = SESSION_GetChargingState(u8DockNo);

    if (eLiveStage == CH_STATE_CHARGING)
    {
        uint32_t u32CurrentTick = xTaskGetTickCount();
        if ((u32PreviousTick[u8DockNo] + pdMS_TO_TICKS(ENERGY_CALC_INTERVAL_MS)) <= u32CurrentTick)
        {
            u32PreviousTick[u8DockNo] = u32CurrentTick;

            float fVoltage   = SESSION_GetPmOutputVoltage(u8DockNo);
            float fCurrent   = SESSION_GetPmOutputCurrent(u8DockNo);
            float fPowerWatt = fVoltage * fCurrent;
            SESSION_SetOutputPower(u8DockNo, fPowerWatt);

            if ((fVoltage > fMinValidVoltage) && (fCurrent > fMinValidCurrent))
            {
                float fDeltaKwh  = (fPowerWatt / 3600.0f) / FACTOR_1000_F;
                float fNewEnergy = (float)SESSION_GetEnergyDelivered(u8DockNo) + fDeltaKwh;
                SESSION_SetEnergyDelivered(u8DockNo, fNewEnergy);
            }
            else
            {
                SYS_CONSOLE_PRINT("[Gun %d] Invalid V/I (V=%.1f, I=%.1f)\r\n",
                                  (int)u8DockNo, (double)fVoltage, (double)fCurrent);
            }
        }
    }
    else if (eLiveStage == CH_STATE_PRECHARGE)
    {
        SESSION_SetEnergyDelivered(u8DockNo, 0.0f);
        u32PreviousTick[u8DockNo] = 0U;
        SYS_CONSOLE_PRINT("[Gun %d] Session started — energy counter reset\r\n", (int)u8DockNo);
    }
    else if (eLiveStage == CH_STATE_SESSION_COMPLETE)
    {
        float fFinalEnergy = (float)SESSION_GetEnergyDelivered(u8DockNo);
        SYS_CONSOLE_PRINT("[Gun %d] Session ended. Energy = %.3f kWh\r\n",
                          (int)u8DockNo, (double)fFinalEnergy);
        u32PreviousTick[u8DockNo] = 0U;
    }
}


/* ============================================================
 * SECTION 8: COMMON STATE HANDLER & TASK
 * ============================================================ */

/**
 * @brief  Per-dock charging process handler called each task cycle.
 *
 *         Execution order:
 *           1. Print system telemetry (rate-limited)
 *           2. Update system state (LED, session lifecycle)
 *           3. Update vehicle/PM demand info in session DB
 *           4. Update energy accumulation
 *           5. Evaluate stop conditions — force SHUTDOWN or ERROR if triggered
 */
static void vChargingProcessHandler(uint8_t u8DockNo)
{
    vPrintSystemData(u8DockNo);
    updateSystemState(u8DockNo);
    vUpdateVehiclePMInfo(u8DockNo);
    vEnergyTimeCalculation(u8DockNo);

    CH_State_e eState = SESSION_GetChargingState(u8DockNo);

    if (bCheckStopCondition(u8DockNo) == true)
    {
        if ((eState == CH_STATE_CHARGING) || (eState == CH_STATE_PRECHARGE))
        {
            SYS_CONSOLE_PRINT("[GUN %d] Fault Stop. FaultCode: 0x%08lX\r\n",
                              (int)u8DockNo,
                              SESSION_GetSystemFaultBitmap(u8DockNo));
            SESSION_SetChargingState(u8DockNo, CH_STATE_SHUTDOWN);
            SESSION_SetAuthenticationCommand(u8DockNo, 0U);
        }
        else
        {
            SESSION_SetChargingState(u8DockNo, CH_STATE_ERROR);
        }
    }
}

/**
 * @brief  Update LED and session lifecycle flags on state change.
 *
 *         Detects state transitions and:
 *           - Sets LED color/blink pattern per state
 *           - Resets BMS data on INIT
 *           - Tracks session-active flag
 *           - Triggers session cleanup (SESSION_ResetSession) when
 *             session exits active states
 */
static void updateSystemState(uint8_t u8DockNo)
{
    static uint8_t   bSessionActive[MAX_DOCKS]  = {0};
    static uint8_t   u8PrvLiveState[MAX_DOCKS]  = {CH_STATE_INIT};
    CH_State_e eState = SESSION_GetChargingState(u8DockNo);

    if (eState != u8PrvLiveState[u8DockNo])
    {
        u8PrvLiveState[u8DockNo] = (uint8_t)eState;

        switch (eState)
        {
        case CH_STATE_INIT:
            vSetLedState(u8DockNo, LED_BLUE, LED_STATE_BLINK);
            // SESSION_ResetBMSData(u8DockNo);
            break;

        case CH_STATE_AUTH_SUCCESS:
        case CH_STATE_PARAM_VALIDATE:
        case CH_STATE_CONNECTION_CONFIRMED:
        case CH_STATE_INITIALIZE:
            vSetLedState(u8DockNo, LED_BLUE, LED_STATE_STEADY);
            bSessionActive[u8DockNo] = 1U;
            break;

        case CH_STATE_PRECHARGE:
            vSetLedState(u8DockNo, LED_BLUE, LED_STATE_BLINK);
            bSessionActive[u8DockNo] = 1U;
            break;

        case CH_STATE_CHARGING:
            vSetLedState(u8DockNo, LED_GREEN, LED_STATE_STEADY);
            bSessionActive[u8DockNo] = 1U;
            break;

        case CH_STATE_SHUTDOWN:
        case CH_STATE_SESSION_COMPLETE:
            vSetLedState(u8DockNo, LED_GREEN, LED_STATE_BLINK);
            break;

        case CH_STATE_ERROR:
            vSetLedState(u8DockNo, LED_RED, LED_STATE_STEADY);
            break;

        default:
            SYS_CONSOLE_PRINT("G%d: Unknown state %d\r\n", (int)u8DockNo, (int)eState);
            vSetLedState(u8DockNo, LED_RED, LED_STATE_BLINK);
            break;
        }

        /* Session cleanup: reset session once it exits active states */
        bool bIsActiveState = ((eState == CH_STATE_AUTH_SUCCESS)         ||
                               (eState == CH_STATE_PARAM_VALIDATE)       ||
                               (eState == CH_STATE_CONNECTION_CONFIRMED) ||
                               (eState == CH_STATE_INITIALIZE)           ||
                               (eState == CH_STATE_PRECHARGE)            ||
                               (eState == CH_STATE_CHARGING));

        if ((!bIsActiveState) && (bSessionActive[u8DockNo] != 0U))
        {
            SYS_CONSOLE_PRINT("G%d: Session cleanup\r\n", (int)u8DockNo);
            bSessionActive[u8DockNo] = 0U;
            SESSION_ResetSession(u8DockNo);
        }
    }
}

/**
 * @brief  CHARGING_TASK — Main FreeRTOS task loop.
 *         Iterates over all docks each cycle, calling the process handler
 *         followed by the state machine dispatcher.
 */
static void CHARGING_TASK(void *pvParameters)
{
    (void)pvParameters;
    SYS_CONSOLE_PRINT("CHARGING_TASK started\r\n");

    for (;;)
    {
        for (uint8_t u8DockNo = DOCK_1; u8DockNo <= SESSION_GetMaxDocks(); u8DockNo++)
        {
            vDummyDataUpdate(u8DockNo); // Simulate BMS/PM data for testing
            vChargingProcessHandler(u8DockNo);
            Charging_StateMachine(u8DockNo);
        }
        vTaskDelay(pdMS_TO_TICKS(CHARGING_TASK_DELAY_MS));
    }
}

/* ============================================================
 * SECTION 9: LED CONTROL
 * ============================================================ */

/**
 * @brief  Set LED color and blink/steady state for a dock.
 *
 * @param  u8DockNo  Dock index
 * @param  ledColor  LED_RED, LED_GREEN, or LED_BLUE
 * @param  ledState  LED_STATE_STEADY or LED_STATE_BLINK
 */
void vSetLedState(uint8_t u8DockNo, uint8_t ledColor, uint8_t ledState)
{
    /* TODO: Connect to hardware LED driver */
    (void)u8DockNo;
    switch (ledColor)
    {
    case LED_RED:   /* TODO: set red LED */   break;
    case LED_GREEN: /* TODO: set green LED */ break;
    case LED_BLUE:  /* TODO: set blue LED */  break;
    default:                                  break;
    }
    (void)ledState;
}


/* ============================================================
 * SECTION 10: DEBUG / TELEMETRY PRINT FUNCTIONS
 * ============================================================ */

/**
 * @brief  Rate-limited system telemetry printer.
 *         Prints every PRINT_INTERVAL_MS or on state change.
 */
static void vPrintSystemData(uint8_t u8DockNo)
{
    static uint32_t   u32NextPrintTick[MAX_DOCKS]   = {0};
    static CH_State_e prevChargingState[MAX_DOCKS]  = {CH_STATE_INIT};

    uint32_t u32CurrentTick  = xTaskGetTickCount();
    CH_State_e eState        = SESSION_GetChargingState(u8DockNo);

    if ((u32CurrentTick > u32NextPrintTick[u8DockNo]) ||
        (eState != prevChargingState[u8DockNo]))
    {
        prevChargingState[u8DockNo] = eState;
        u32NextPrintTick[u8DockNo]  = u32CurrentTick + pdMS_TO_TICKS(PRINT_INTERVAL_MS);

        vPrintStateAndDeviceInfo(u8DockNo);
        if (eState == CH_STATE_CHARGING)
        {
            vPrintChargingInfo(u8DockNo);
        }
    }
}

/** @brief Print state, auth, BMS/PM connectivity, and fault bitmaps. */
static void vPrintStateAndDeviceInfo(uint8_t u8DockNo)
{
    SYS_CONSOLE_PRINT(
        "\r\n================ CHARGING INFO - Dock %d [%s] ================\r\n"
        "Auth Cmd       : %lu\r\n"
        "BMS Status     : %s\r\n"
        "PM Status      : %s\r\n"
        "System Fault   : 0x%08lX\r\n"
        "BMS Fault      : 0x%08lX\r\n"
        "PM Fault       : 0x%08lX\r\n"
        "=======================================================\r\n",
        (int)u8DockNo,
        CH_GetStateString(SESSION_GetChargingState(u8DockNo)),
        SESSION_GetAuthenticationCommand(u8DockNo),
        SESSION_GetBMSRxStatus(u8DockNo) ? "Connected" : "Disconnected",
        SESSION_GetPMRxStatus(u8DockNo)  ? "Connected" : "Disconnected",
        SESSION_GetSystemFaultBitmap(u8DockNo),
        SESSION_GetBMSFaultBitmap(u8DockNo),
        SESSION_GetPMFaultBitmap(u8DockNo)
    );
}

/** @brief Print live charging metrics (SOC, V, I, energy). */
static void vPrintChargingInfo(uint8_t u8DockNo)
{
    SYS_CONSOLE_PRINT(
        "=================== CHARGING METRICS - Dock %d ===================\r\n"
        "Demand  : V=%4.0f V   I=%4.0f A\r\n"
        "Output  : V=%4.0f V   I=%4.0f A\r\n"
        "SOC     : %3d%% (Initial: %3d%%)\r\n"
        "Energy  : %6.3f kWh\r\n"
        "===================================================================\r\n",
        (int)u8DockNo,
        (double)SESSION_GetBMSDemandVoltage(u8DockNo),
        (double)SESSION_GetBMSDemandCurrent(u8DockNo),
        (double)SESSION_GetPmOutputVoltage(u8DockNo),
        (double)SESSION_GetPmOutputCurrent(u8DockNo),
        (int)SESSION_GetCurrentSoc(u8DockNo),
        (int)SESSION_GetInitialSoc(u8DockNo),
        (double)SESSION_GetEnergyDelivered(u8DockNo)
    );
}

/** @brief Convert CH_State_e to human-readable string for debug output. */
static const char *CH_GetStateString(CH_State_e state)
{
    switch (state)
    {
    case CH_STATE_INIT:                 return "INIT";
    case CH_STATE_AUTH_SUCCESS:         return "AUTH_SUCCESS";
    case CH_STATE_PARAM_VALIDATE:       return "PARAM_VALIDATE";
    case CH_STATE_CONNECTION_CONFIRMED: return "CONNECTION_CONFIRMED";
    case CH_STATE_INITIALIZE:           return "INITIALIZE";
    case CH_STATE_PRECHARGE:            return "PRECHARGE";
    case CH_STATE_CHARGING:             return "CHARGING";
    case CH_STATE_SHUTDOWN:             return "SHUTDOWN";
    case CH_STATE_SESSION_COMPLETE:     return "SESSION_COMPLETE";
    case CH_STATE_ERROR:                return "ERROR";
    default:                            return "UNKNOWN";
    }
}


/* ============================================================
 * SECTION 11: PUBLIC API
 * ============================================================ */

/**
 * @brief  Initialize and start the CHARGING_TASK FreeRTOS task.
 *
 * @return true  - Task created successfully
 * @return false - Task creation failed
 */
bool ChargingTask_Init(void)
{
    bool bStatus = false;

    BaseType_t xTaskStatus = xTaskCreate(CHARGING_TASK,
                                         CHARGING_TASK_NAME,
                                         CHARGING_TASK_STACK_SIZE_WORDS,
                                         NULL,
                                         CHARGING_TASK_PRIORITY,
                                         &xCHARGING_TASK);
    if (xTaskStatus == pdPASS)
    {
        SYS_CONSOLE_PRINT("CHARGING_TASK created [Protocol: %s]\r\n",
#if (CHARGING_PROTOCOL == PROTOCOL_17017_25)
                          "LEVDC ISO 17017-25"
#else
                          "TVS Proprietary"
#endif
        );
        vChargingCommunicationInit();
        bStatus = true;
    }
    else
    {
        SYS_CONSOLE_PRINT("CHARGING_TASK creation FAILED\r\n");
    }
    return bStatus;
}

void vDummyDataUpdate(uint8_t u8DockNo)
{
    if (SESSION_GetCompartmentId() == 1)
    {
        return;
    }

    float fVoltage = 52.0f + u8DockNo; // dock-wise increment
    float fCurrent = 40.0f + u8DockNo;
    uint16_t u16FaultInfo = 0; // No faults in dummy data
    static uint8_t u8Counter = 0;
    if (bGPIO_Operation(DI_BP_STATUS, u8DockNo, GPIO_READ) == true)
    {
        TVS_MsgFrameInfo_t sFrame = {0};
        (void)bGetSetTVSBMSData(u8DockNo, &sFrame, GET_PARA);
        /* ==========================================================
         * 0x100 - BMS STATUS (RX → Charger)
         * ========================================================== */
        sFrame.TVS_Rx100_BMSStatus.u16BMSCurrent =
            (uint16_t)(fCurrent / 0.03125f); // ~1280

        sFrame.TVS_Rx100_BMSStatus.u16BMSVoltage =
            (uint16_t)(fVoltage / 0.015625f); // ~3328

        sFrame.TVS_Rx100_BMSStatus.u8Counter = u8Counter++;
        sFrame.TVS_Rx100_BMSStatus.u8SOC = 80 + u8DockNo;
        sFrame.TVS_Rx100_BMSStatus.u8ErrorState = 0;
        sFrame.TVS_Rx100_BMSStatus.u8Temperature = (25 + 30); // offset -30

        /* ==========================================================
         * 0x101 - BMS PROFILE (RX → Charger)  ⭐ IMPORTANT
         * ========================================================== */

        sFrame.TVS_Rx101_BMSProfile.u16MaxChargeVoltage =
            (uint16_t)(fVoltage * 1000.0f);

        sFrame.TVS_Rx101_BMSProfile.u16MaxChargeCurrent =
            (uint16_t)(fCurrent * 1000.0f);

        sFrame.TVS_Rx101_BMSProfile.u16CutOffChargeCurrent =
            (uint16_t)(5.0f * 1000.0f);

        sFrame.TVS_Rx101_BMSProfile.u16PreChargeCurrent =
            (uint16_t)(2.0f * 1000.0f);

        (void)bGetSetTVSBMSData(u8DockNo, &sFrame, SET_PARA);
        SESSION_SetBMSLastRxTime(u8DockNo, xTaskGetTickCount());
    }

    /* ==========================================================
     * PM Data Update (simulate PM response based on BMS profile) ⭐ IMPORTANT
     * ========================================================== */
    if (bGPIO_Operation(DO_AC_RELAY_ON, u8DockNo, GPIO_READ) == true)
    {
        
        SESSION_SetPmOutputVoltage(u8DockNo, fVoltage);
        SESSION_SetPmOutputCurrent(u8DockNo, fCurrent);
        SESSION_SetPMFaultCode(u8DockNo, u16FaultInfo);
        SESSION_SetPMLastRxTime(u8DockNo, xTaskGetTickCount());
    }

}
