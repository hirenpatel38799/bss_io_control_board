/******************************************************************************
 * File Name   : ChargingHandler.h
 * Description : Portable EV DC Charging State Machine Handler
 *
 * This module implements a dual-protocol charging state machine supporting:
 *   - PROTOCOL_17017_25  : LEVDC ISO 17017-25 standard CAN protocol
 *   - PROTOCOL_TVS_PROP  : TVS proprietary CAN protocol
 *
 * Protocol selection is done at compile time via the CHARGING_PROTOCOL macro.
 * All hardware and protocol interactions are performed through callback
 * interfaces provided by the application layer.
 *
 * Author      : Sarang Parmar
 * Date        : 16-Mar-2026
 * Version     : 2.0
 ******************************************************************************/

#ifndef CHARGING_HANDLER_H
#define CHARGING_HANDLER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "sessionDBHandler.h"

/*==============================================================================
 * PROTOCOL SELECTION
 * Uncomment exactly ONE of the following to select the active protocol.
 * This gates all protocol-specific structures, macros, and state functions.
 *============================================================================*/
/* #define CHARGING_PROTOCOL   PROTOCOL_17017_25 */
#define CHARGING_PROTOCOL      PROTOCOL_TVS_PROP

/* Protocol identifier tokens — do NOT modify these */
#define PROTOCOL_17017_25      (1U)
#define PROTOCOL_TVS_PROP      (2U)

/* Compile-time guard: ensure exactly one protocol is chosen */
#if !defined(CHARGING_PROTOCOL)
    #error "CHARGING_PROTOCOL must be defined as PROTOCOL_17017_25 or PROTOCOL_TVS_PROP"
#endif
#if (CHARGING_PROTOCOL != PROTOCOL_17017_25) && (CHARGING_PROTOCOL != PROTOCOL_TVS_PROP)
    #error "CHARGING_PROTOCOL must be PROTOCOL_17017_25 or PROTOCOL_TVS_PROP"
#endif

/*==============================================================================
 * COMMON GET/SET OPERATION CODES
 *============================================================================*/
#define SET_PARA    (0U)
#define GET_PARA    (1U)

/*==============================================================================
 * SYSTEM FAULT BITMAP — protocol-independent
 *============================================================================*/
#define SYSTEM_FAULT_NONE                   (0U)
#define SYSTEM_FAULT_ESTOP_TRIGGERED        (1U << 0)
#define SYSTEM_FAULT_BMS_ERROR              (1U << 1)
#define SYSTEM_FAULT_PM_ERROR               (1U << 2)
#define SYSTEM_FAULT_BMS_COMMUNICATION_FAILURE (1U << 3)
#define SYSTEM_FAULT_PM_COMMUNICATION_FAILURE  (1U << 4)
#define SYSTEM_FAULT_PM_ZERO_CURRENT        (1U << 5)
#define SYSTEM_FAULT_PRECHARGE_FAILURE      (1U << 6)

/*==============================================================================
 * BMS FAULT BITMAP — protocol-independent
 *============================================================================*/
#define BMS_FAULT_ENERGY_TRANSFER_ERROR     (0U)
#define BMS_FAULT_BATTERY_OVERVOLT          (1U)
#define BMS_FAULT_BATTERY_UNDERVOLT         (2U)
#define BMS_FAULT_BATTERY_CURRENT_DEVIATION (3U)
#define BMS_FAULT_BATTERY_VOLTAGE_DEVIATION (4U)
#define BMS_FAULT_CHARGING_STOP_CONTROL     (5U)

#define BMS_WARN_HIGH_TEMP                  (16U)

/*==============================================================================
 * PM FAULT BITMAP — protocol-independent
 *============================================================================*/
#define PM_FAULT_INPUT_UNDERVOLT            (0U)
#define PM_FAULT_PHASE_LOSS                 (1U)
#define PM_FAULT_INPUT_OVERVOLT             (2U)
#define PM_FAULT_OUTPUT_OVERVOLT            (3U)
#define PM_FAULT_OUTPUT_OVERCURRENT         (4U)
#define PM_WARN_OUTPUT_UNDERVOLT            (12U)
#define PM_WARN_OUTPUT_OVERVOLT             (13U)
#define PM_WARN_POWER_LIMIT                 (14U)

/*==============================================================================
 * STOP REASON CODES — protocol-independent
 *============================================================================*/
#define STOP_REASON_MCU_REQUEST             (1U)
#define STOP_REASON_FAULT                   (2U)

/*==============================================================================
 * BIT MANIPULATION HELPERS
 *============================================================================*/
#define SET_BIT(reg, bit)   ((reg) |=  (1U << (bit)))
#define CLR_BIT(reg, bit)   ((reg) &= ~(1U << (bit)))
#define GET_BIT(reg, bit)   (((reg) >> (bit)) & 1U)

/*==============================================================================
 * COMMON SCALING FACTORS
 *============================================================================*/
// #define FACTOR_10       (10.0f)
// #define FACTOR_0_1      (0.1f)
// #define FACTOR_1000_F   (1000.0f)
// #define FACTOR_1000     (1000U)
// #define FACTOR_100      (100U)


/* ============================================================
 * SECTION A: LEVDC ISO 17017-25 PROTOCOL
 * Active when: CHARGING_PROTOCOL == PROTOCOL_17017_25
 * ============================================================ */
#if (CHARGING_PROTOCOL == PROTOCOL_17017_25)

/*--------------------------------------------------------------
 * A1. LEVDC CAN Message IDs
 *------------------------------------------------------------*/
/* EV (BMS) → EVSE (Charger) — Receive IDs */
#define LEVDC_CAN_ID_EV_REQUEST            0x500
#define LEVDC_CAN_ID_EV_CHARGING_INFO      0x501
#define LEVDC_CAN_ID_EV_CONTROL_OPTION     0x502
#define LEVDC_CAN_ID_EV_RESERVED_580       0x580
#define LEVDC_CAN_ID_EV_RESERVED_581       0x581
#define LEVDC_CAN_ID_EV_RESERVED_582       0x582
#define LEVDC_CAN_ID_EV_RESERVED_583       0x583

/* EVSE (Charger) → EV (BMS) — Transmit IDs */
#define LEVDC_CAN_ID_EVSE_STATUS           0x508
#define LEVDC_CAN_ID_EVSE_OUTPUT_INFO      0x509
#define LEVDC_CAN_ID_EVSE_CAPABILITY       0x510
#define LEVDC_CAN_ID_EVSE_CHARGER_ID       0x584

/*--------------------------------------------------------------
 * A2. LEVDC Protocol Limits
 *------------------------------------------------------------*/
#define LEVDC_POWER_RESOLUTION    50U       /* Power resolution in watts           */
#define LEVDC_MAX_VOLTAGE         120U      /* Max charger output voltage (V)      */
#define LEVDC_MAX_CURRENT         100U      /* Max charger output current (A)      */
#define LEVDC_RATED_DC_OP_POWER   ((LEVDC_MAX_VOLTAGE * LEVDC_MAX_CURRENT) / LEVDC_POWER_RESOLUTION)
#define LEVDC_SHUTOFF_DELAY_MS    pdMS_TO_TICKS(2000U)

/*--------------------------------------------------------------
 * A3. LEVDC TX CAN Message Structures (EVSE → EV)
 *------------------------------------------------------------*/

/**
 * @brief 0x508 — EVSE Status Frame (8 bytes)
 *        Reports EVSE state, errors, output capability to EV.
 */
typedef struct __attribute__((packed))
{
    /* Byte 0 */
    uint8_t u8ChargingSysError      : 1;  /**< 0 = No error,    1 = Error          */
    uint8_t u8EvseMalFunctionError  : 1;  /**< 0 = No error,    1 = Malfunction    */
    uint8_t u8EVCompatible          : 1;  /**< 0 = Compatible,  1 = Incompatible   */
    uint8_t u8Res                   : 5;  /**< Reserved                            */
    /* Byte 1 */
    uint8_t u8EvseStopCtrl          : 1;  /**< 0 = No error,    1 = Stop           */
    uint8_t u8EvseStatus            : 1;  /**< 0 = Standby,     1 = Charging       */
    uint8_t u8ConLatchStatus        : 1;  /**< 0 = Unlatched,   1 = Latched        */
    uint8_t u8EVSEReadyForCharge    : 1;  /**< 0 = Not Ready,   1 = Ready          */
    uint8_t u8WaitStatebfrCharg     : 1;  /**< Wait state flag                     */
    uint8_t u8Res1                  : 3;  /**< Reserved                            */
    /* Bytes 2-3 */
    uint16_t u16RatedOutputVol;           /**< Rated output voltage  (x10, V)      */
    /* Bytes 4-5 */
    uint16_t u16AvailOutputCur;           /**< Available output current (x10, A)   */
    /* Bytes 6-7 */
    uint16_t u16ConfDCvolLimit;           /**< Configured DC voltage limit (x10, V)*/
} LEVDC_Tx508_t;

/**
 * @brief 0x509 — EVSE Output Info Frame (8 bytes)
 *        Reports live output power, voltage, current, and remaining time.
 */
typedef struct __attribute__((packed))
{
    uint8_t  u8ControlProtocolNum;        /**< Byte 0 - Protocol number            */
    uint8_t  u8AvailDCOutputPower;        /**< Byte 1 - Available power (50W res)  */
    uint16_t u16EVSEoutputVoltage;        /**< Bytes 2-3 - Output voltage (x10, V) */
    uint16_t u16EVSEoutputCurrent;        /**< Bytes 4-5 - Output current (x10, A) */
    uint16_t u16RemainChargeTime;         /**< Bytes 6-7 - Remaining time (min)    */
} LEVDC_Tx509_t;

/**
 * @brief 0x510 — EVSE Capability Frame (8 bytes)
 *        Advertises voltage control capability to EV.
 */
typedef struct __attribute__((packed))
{
    uint8_t u8Res                   : 1;  /**< Reserved                            */
    uint8_t u8EVSEVolatageControlOpt: 1;  /**< 0 = No control, 1 = Control enabled */
    uint8_t u8Res1                  : 6;  /**< Reserved                            */
    uint8_t u8Res2[7];                    /**< Bytes 1-7: Reserved                 */
} LEVDC_Tx510_t;

/**
 * @brief 0x584 — EVSE Charger ID Frame (8 bytes)
 *        Charger identification string transmitted to EV.
 */
typedef struct __attribute__((packed))
{
    uint8_t u8ChargerID1;                 /**< Byte 0 - Charger ID octet 1         */
    uint8_t u8ChargerID2;                 /**< Byte 1 - Charger ID octet 2         */
    uint8_t u8ChargerID3;                 /**< Byte 2 - Charger ID octet 3         */
    uint8_t u8ChargerID4;                 /**< Byte 3 - Charger ID octet 4         */
    uint8_t u8ChargerID5;                 /**< Byte 4 - Charger ID octet 5         */
    uint8_t u8ChargerID6;                 /**< Byte 5 - Charger ID octet 6         */
    uint8_t u8ChargerID7;                 /**< Byte 6 - Charger ID octet 7         */
    uint8_t u8ChargerID8;                 /**< Byte 7 - Charger ID octet 8         */
} LEVDC_Tx584_t;

/*--------------------------------------------------------------
 * A4. LEVDC RX CAN Message Structures (EV → EVSE)
 *------------------------------------------------------------*/

/**
 * @brief 0x500 — EV Request Frame (8 bytes)
 *        EV reports its status, faults, and power demands to EVSE.
 */
typedef struct __attribute__((packed))
{
    /* Byte 0 — EV fault flags */
    uint8_t u8EnergyTransferError   : 1;  /**< Energy transfer error               */
    uint8_t u8BatteryOverVol        : 1;  /**< Battery over-voltage                */
    uint8_t u8BatteryUnderVol       : 1;  /**< Battery under-voltage               */
    uint8_t u8BatterCurrentDeviError: 1;  /**< Battery current deviation error     */
    uint8_t u8HighBatteryTemp       : 1;  /**< High battery temperature warning    */
    uint8_t u8BatterVoltageDeviError: 1;  /**< Battery voltage deviation error     */
    uint8_t u8Res                   : 2;  /**< Reserved                            */
    /* Byte 1 — EV control flags */
    uint8_t u8EvChargingEnable      : 1;  /**< EV requests charging enable         */
    uint8_t u8EvConStatus           : 1;  /**< EV connector status                 */
    uint8_t u8EvChargingPosition    : 1;  /**< EV charging position confirmed      */
    uint8_t u8EvChargingStopControl : 1;  /**< EV requests charging stop           */
    uint8_t u8WaitReqToEngTransfer  : 1;  /**< Wait request before energy transfer */
    uint8_t u8DigitalCommToggle     : 1;  /**< Digital communication alive toggle  */
    uint8_t u8Res1                  : 2;  /**< Reserved                            */
    /* Bytes 2-3 */
    uint16_t u16ReqDcCurrent;             /**< Requested DC current (x10, A)       */
    /* Bytes 4-5 */
    uint16_t u16DcOutputVolTarget;        /**< Target output voltage (x10, V)      */
    /* Bytes 6-7 */
    uint16_t u16DcOutputVoltLimit;        /**< Max output voltage limit (x10, V)   */
} LEVDC_Rx500_t;

/**
 * @brief 0x501 — EV Charging Info Frame (8 bytes)
 *        EV reports protocol, charging rate, and time estimates.
 */
typedef struct __attribute__((packed))
{
    uint8_t  u8ControlProtocolNo;         /**< Byte 0 - Protocol number            */
    uint8_t  u8ChargingRate;              /**< Byte 1 - SOC / charging rate (%)    */
    uint16_t u16MaxChargingTime;          /**< Bytes 2-3 - Max charge time (min)   */
    uint16_t u16EstimatedChargingTime;    /**< Bytes 4-5 - Estimated time (min)    */
    uint16_t u16Res;                      /**< Bytes 6-7 - Reserved                */
} LEVDC_Rx501_t;

/**
 * @brief 0x502 — EV Control Option Frame (8 bytes)
 *        EV reports voltage control option preference.
 */
typedef struct __attribute__((packed))
{
    uint8_t u8VoltageControlOption  : 1;  /**< 0 = No control, 1 = Enabled         */
    uint8_t u8Res                   : 7;  /**< Reserved                            */
    uint8_t u8Res1[7];                    /**< Bytes 1-7: Reserved                 */
} LEVDC_Rx502_t;

/**
 * @brief 0x580–0x583 — EV Reserved Frames (8 bytes each)
 *        Reserved for future development.
 */
typedef struct __attribute__((packed))
{
    uint8_t u8FutureDev[8];               /**< Bytes 0-7: Reserved                 */
} LEVDC_Rx580_t, LEVDC_Rx581_t, LEVDC_Rx582_t, LEVDC_Rx583_t;

/*--------------------------------------------------------------
 * A5. LEVDC Aggregated Message Frame
 *------------------------------------------------------------*/
/**
 * @brief Aggregated live CAN data store for one dock (LEVDC protocol).
 *        Holds the latest TX and RX frames for all LEVDC CAN message IDs.
 */
typedef struct __attribute__((packed))
{
    LEVDC_Tx508_t  LevdcTX_508ID_Info;   /**< 0x508 - EVSE Status (TX)            */
    LEVDC_Tx509_t  LevdcTX_509ID_Info;   /**< 0x509 - EVSE Output Info (TX)       */
    LEVDC_Tx510_t  LevdcTX_510ID_Info;   /**< 0x510 - EVSE Capability (TX)        */
    LEVDC_Tx584_t  LevdcTX_584ID_Info;   /**< 0x584 - EVSE Charger ID (TX)        */
    LEVDC_Rx500_t  LevdcRX_500ID_Info;   /**< 0x500 - EV Request (RX)             */
    LEVDC_Rx501_t  LevdcRX_501ID_Info;   /**< 0x501 - EV Charging Info (RX)       */
    LEVDC_Rx502_t  LevdcRX_502ID_Info;   /**< 0x502 - EV Control Option (RX)      */
    LEVDC_Rx580_t  LevdcRX_580ID_Info;   /**< 0x580 - EV Reserved (RX)            */
    LEVDC_Rx581_t  LevdcRX_581ID_Info;   /**< 0x581 - EV Reserved (RX)            */
    LEVDC_Rx582_t  LevdcRX_582ID_Info;   /**< 0x582 - EV Reserved (RX)            */
    LEVDC_Rx583_t  LevdcRX_583ID_Info;   /**< 0x583 - EV Reserved (RX)            */
} ChargingMsgFrameInfo_t;

extern ChargingMsgFrameInfo_t Charging_LiveInfo[MAX_DOCKS];

/*--------------------------------------------------------------
 * A6. LEVDC Protocol Enumerations
 *------------------------------------------------------------*/
typedef enum { EVSE_NOERROR = 0U,  EVSE_ERROR = 1U }           ErrorState_e;
typedef enum { NO_VOLTAGE_CONTROL = 0, VOLTAGE_CONTROL_ENABLED = 1 } VoltageControlOption_e;
typedef enum { EVSE_STANDBY = 0, EVSE_CHARGING = 1 }           ChargingState_e;
typedef enum { EV_COMPATIBLE = 0U, EV_INCOMPATIBLE = 1U }      EvIncompatibility;
typedef enum { EVSE_NOT_READY = 0U, EVSE_READY = 1U }          EvSupplyEquipmentState_e;
typedef enum { GUN_UNLATCHED = 0U, GUN_LATCHED = 1U }          EvseGunLatch;
typedef enum { EVSE_NOT_READY_FOR_CHARGE = 0U, EVSE_READY_FOR_CHARGE = 1U } EvseReadyForCharge_e;

/*--------------------------------------------------------------
 * A7. LEVDC GET/SET Function Prototype
 *------------------------------------------------------------*/
/**
 * @brief  Thread-safe read/write of LEVDC live CAN frame for a dock.
 * @param  u8DockNo   Dock index (0 to MAX_DOCKS-1)
 * @param  psData     Pointer to ChargingMsgFrameInfo_t buffer
 * @param  u8Operation SET_PARA (write) or GET_PARA (read)
 * @return true on success, false on invalid arguments
 */
bool bGetSetLevdcBMSData(uint8_t u8DockNo,
                         ChargingMsgFrameInfo_t *psData,
                         uint8_t u8Operation);

#endif /* CHARGING_PROTOCOL == PROTOCOL_17017_25 */


/* ============================================================
 * SECTION B: TVS PROPRIETARY PROTOCOL
 * Active when: CHARGING_PROTOCOL == PROTOCOL_TVS_PROP
 * ============================================================ */
#if (CHARGING_PROTOCOL == PROTOCOL_TVS_PROP)

/*--------------------------------------------------------------
 * B1. TVS CAN Message IDs
 *------------------------------------------------------------*/
/* Charger (EVSE) → BMS — Transmit IDs */
#define TVS_CAN_ID_INFO                (0x90U)   /**< Charger status info (TX, 100ms)  */
#define TVS_CAN_ID_CHARGE_PROFILE      (0x91U)   /**< Charge profile: V & I (TX,100ms) */
#define TVS_CAN_ID_FM_VERSION_INFO     (0x92U)   /**< FW version info (TX, 1000ms)     */

/* BMS → Charger (EVSE) — Receive IDs */
#define TVS_CAN_ID_STATUS              (0x100U)  /**< BMS live status (RX, 100ms)      */
#define TVS_CAN_ID_PROFILE             (0x101U)  /**< BMS charge profile (RX, 100ms)   */

/*--------------------------------------------------------------
 * B2. TVS Signal Resolution & Offset Factors
 *    (per DBC specification)
 *------------------------------------------------------------*/
#define TVS_FACTOR_CURRENT_0x91        (0.015625f) /**< 0x91 CHARGER_Current  res (A)  */
#define TVS_FACTOR_VOLTAGE_0x91        (0.015625f) /**< 0x91 CHARGER_Voltage  res (V)  */
#define TVS_FACTOR_BMS_CURRENT         (0.03125f)  /**< 0x100 BMS_Current     res (A)  */
#define TVS_FACTOR_BMS_VOLTAGE         (0.015625f) /**< 0x100 BMS_Voltage     res (V)  */
#define TVS_FACTOR_BMS_PROFILE         (0.001f)    /**< 0x101 Profile signals res      */

#define TVS_OFFSET_AC_INPUT            (50)        /**< CHARGER_ACInput  offset  (V)   */
#define TVS_OFFSET_TEMPERATURE         (50)        /**< CHARGER_Temp     offset  (°C)  */
#define TVS_OFFSET_BMS_TEMPERATURE     (30)        /**< BMS_Temperature  offset  (°C)  */

/*--------------------------------------------------------------
 * B3. TVS Protocol Limits
 *------------------------------------------------------------*/
#define TVS_MAX_VOLTAGE                (128.0f)    /**< Max BMS voltage limit (V)       */
#define TVS_MAX_CURRENT                (65.535f)   /**< Max BMS profile current (A)     */
#define TVS_MIN_AC_INPUT               (50U)       /**< Min AC input voltage (V)        */
#define TVS_MAX_AC_INPUT               (305U)      /**< Max AC input voltage (V)        */
#define TVS_SHUTOFF_DELAY_MS           pdMS_TO_TICKS(500U)
#define TVS_TEMP_DERATING_THRESHOLD    (55)        /**< Charger temp at which derating triggers (°C) */

/*--------------------------------------------------------------
 * B4. TVS Signal Encode (Physical → Raw) Macros
 *------------------------------------------------------------*/
#define TVS_ENCODE_CHARGER_CURRENT(phys)  ((uint16_t)((phys) / TVS_FACTOR_CURRENT_0x91))
#define TVS_ENCODE_CHARGER_VOLTAGE(phys)  ((uint16_t)((phys) / TVS_FACTOR_VOLTAGE_0x91))
#define TVS_ENCODE_AC_INPUT(phys)         ((uint8_t)((phys)  - TVS_OFFSET_AC_INPUT))
#define TVS_ENCODE_TEMPERATURE(phys)      ((uint8_t)((phys)  + TVS_OFFSET_TEMPERATURE))
#define TVS_ENCODE_BMS_TEMPERATURE(phys)  ((uint8_t)((phys)  + TVS_OFFSET_BMS_TEMPERATURE))

/*--------------------------------------------------------------
 * B5. TVS Signal Decode (Raw → Physical) Macros
 *------------------------------------------------------------*/
#define TVS_DECODE_CHARGER_CURRENT(raw)       ((float)(raw) * TVS_FACTOR_CURRENT_0x91)
#define TVS_DECODE_CHARGER_VOLTAGE(raw)       ((float)(raw) * TVS_FACTOR_VOLTAGE_0x91)
#define TVS_DECODE_BMS_CURRENT(raw)           ((float)(raw) * TVS_FACTOR_BMS_CURRENT)
#define TVS_DECODE_BMS_VOLTAGE(raw)           ((float)(raw) * TVS_FACTOR_BMS_VOLTAGE)
#define TVS_DECODE_BMS_PROFILE_CURRENT(raw)   ((float)(raw) * TVS_FACTOR_BMS_PROFILE)
#define TVS_DECODE_BMS_PROFILE_VOLTAGE(raw)   ((float)(raw) * TVS_FACTOR_BMS_PROFILE)
#define TVS_DECODE_AC_INPUT(raw)              ((uint16_t)(raw) + TVS_OFFSET_AC_INPUT)
#define TVS_DECODE_TEMPERATURE(raw)           ((int16_t)(raw)  - TVS_OFFSET_TEMPERATURE)
#define TVS_DECODE_BMS_TEMPERATURE(raw)       ((int16_t)(raw)  - TVS_OFFSET_BMS_TEMPERATURE)

/*--------------------------------------------------------------
 * B6. TVS Firmware Version — update per release
 *------------------------------------------------------------*/
#define FW_VERSION_MAJOR       (1U)
#define FW_VERSION_MINOR       (0U)
#define FW_VERSION_ITERATION   (0U)
#define FW_CHARGER_TYPE        (TVS_CHARGER_TYPE_3KW_DELTA_OFFBOARD)
#define FW_RELEASE_DATE_DD     (16U)
#define FW_RELEASE_DATE_MM     (3U)
#define FW_RELEASE_DATE_Y1Y2   (20U)
#define FW_RELEASE_DATE_Y3Y4   (26U)

/*--------------------------------------------------------------
 * B7. TVS TX CAN Message Structures (Charger → BMS)
 *------------------------------------------------------------*/

/**
 * @brief 0x90 — Charger Info Frame (8 bytes, TX, 100ms)
 *        Charger reports its type, AC input, temperature, counter,
 *        and status flags (error, fan, output, derating) to BMS.
 *
 * Byte 4 bit layout:
 *   [3:0] = CHARGER_ErrorState (4-bit error code)
 *   [4]   = CHARGER_Fan
 *   [5]   = CHARGER_Output
 *   [6]   = Charger_deratting
 *   [7]   = Reserved
 */
typedef struct __attribute__((packed))
{
    uint8_t  u8ChargerType;               /**< Byte 0 - CHARGER_Type  (res=1, off=0)   */
    uint8_t  u8ACInput;                   /**< Byte 1 - CHARGER_ACInput(res=1, off=50) */
    uint8_t  u8Temperature;               /**< Byte 2 - CHARGER_Temperature(off=-50°C) */
    uint8_t  u8Counter;                   /**< Byte 3 - CHARGER_Counter (rolling)      */
    uint8_t  u8ErrorState   : 4;          /**< Byte 4[3:0] - CHARGER_ErrorState        */
    uint8_t  u8Fan          : 1;          /**< Byte 4[4]   - CHARGER_Fan               */
    uint8_t  u8Output       : 1;          /**< Byte 4[5]   - CHARGER_Output            */
    uint8_t  u8Derating     : 1;          /**< Byte 4[6]   - Charger_deratting         */
    uint8_t  u8Res          : 1;          /**< Byte 4[7]   - Reserved                  */
    uint8_t  u8Res1[3];                   /**< Bytes 5-7   - Reserved                  */
} TVS_Tx90_Info_t;

/**
 * @brief 0x91 — Charger Charge Profile Frame (8 bytes, TX, 100ms)
 *        Charger reports actual output current and terminal voltage to BMS.
 *        Resolution: 0.015625 (1/64) for both signals.
 */
typedef struct __attribute__((packed))
{
    uint16_t u16ChargingVoltage;          /**< Bytes 0-1 - CHARGER_TerminalVoltage (res=0.015625, V) */
    uint16_t u16ChargingCurrent;          /**< Bytes 2-3 - CHARGER_Current (res=0.015625, A)  */
    uint8_t  u8Res[4];                    /**< Bytes 4-7 - Reserved                           */
} TVS_Tx91_ChargeProfile_t;

/**
 * @brief 0x92 — Charger FM Version Info Frame (8 bytes, TX, 1000ms)
 *        Charger reports firmware version and release date to BMS.
 */
typedef struct __attribute__((packed))
{
    uint8_t  u8FWVersionMajor;            /**< Byte 0 - FW_version_major                      */
    uint8_t  u8FWVersionMinor;            /**< Byte 1 - FW_version_minor                      */
    uint8_t  u8FWVersionIteration;        /**< Byte 2 - FW_version_iteration                  */
    uint8_t  u8FWChargerType;             /**< Byte 3 - FW_CHARGER_type                       */
    uint8_t  u8FWReleaseDateDD;           /**< Byte 4 - FW_Release_Date_DD                    */
    uint8_t  u8FWReleaseDateMM;           /**< Byte 5 - FW_Release_Date_MM                    */
    uint8_t  u8FWReleaseDateY1Y2;         /**< Byte 6 - FW_Release_Date_Y1Y2                  */
    uint8_t  u8FWReleaseDateY3Y4;         /**< Byte 7 - FW_Release_Date_Y3Y4                  */
} TVS_Tx92_FMVersionInfo_t;

/*--------------------------------------------------------------
 * B8. TVS RX CAN Message Structures (BMS → Charger)
 *------------------------------------------------------------*/

/**
 * @brief 0x100 — BMS Status Frame (8 bytes, RX, 100ms)
 *        BMS reports live battery current, voltage, SOC,
 *        error state, and temperature to charger.
 */
typedef struct __attribute__((packed))
{
    uint16_t u16BMSCurrent;               /**< Bytes 0-1 - BMS_Current  (res=0.03125, A)      */
    uint16_t u16BMSVoltage;               /**< Bytes 2-3 - BMS_Voltage  (res=0.015625, V)[13:0]*/
    uint8_t  u8Counter;                   /**< Byte 4    - BMS_Counter  (rolling)              */
    uint8_t  u8SOC;                       /**< Byte 5    - BMS_SOC      (0–100%)               */
    uint8_t  u8ErrorState;                /**< Byte 6    - BMS_ErrorState (0=OK, 1=Stop)       */
    uint8_t  u8Temperature;               /**< Byte 7    - BMS_Temperature (res=1, off=-30°C)  */
} TVS_Rx100_Status_t;

/**
 * @brief 0x101 — BMS Profile Frame (8 bytes, RX, 100ms)
 *        BMS sends charging set-points (max current, voltage,
 *        cut-off current, and pre-charge current) to charger.
 *        Resolution: 0.001 for all signals.
 */
typedef struct __attribute__((packed))
{
    uint16_t u16MaxChargeCurrent;         /**< Bytes 0-1 - BMS_MaxChargeCurrent   (res=0.001, A) */
    uint16_t u16MaxChargeVoltage;         /**< Bytes 2-3 - BMS_MaxChargeVoltage   (res=0.001, V) */
    uint16_t u16CutOffChargeCurrent;      /**< Bytes 4-5 - BMS_CutOffChargeCurrent(res=0.001, A) */
    uint16_t u16PreChargeCurrent;         /**< Bytes 6-7 - BMS_PreChargeCurrent   (res=0.001, A) */
} TVS_Rx101_Profile_t;

/*--------------------------------------------------------------
 * B9. TVS Aggregated Message Frame
 *------------------------------------------------------------*/
/**
 * @brief Aggregated live CAN data store for one dock (TVS protocol).
 *        Holds the latest TX and RX frames for all TVS CAN message IDs.
 */
typedef struct __attribute__((packed))
{
    TVS_Tx90_Info_t          TVS_Tx90_ChargerInfo;   /**< 0x90  - Charger Info (TX)        */
    TVS_Tx91_ChargeProfile_t TVS_Tx91_ChargeProfile; /**< 0x91  - Charge Profile (TX)      */
    TVS_Tx92_FMVersionInfo_t TVS_Tx92_FMVersionInfo; /**< 0x92  - FW Version Info (TX)     */
    TVS_Rx100_Status_t       TVS_Rx100_BMSStatus;    /**< 0x100 - BMS Status (RX)          */
    TVS_Rx101_Profile_t      TVS_Rx101_BMSProfile;   /**< 0x101 - BMS Profile (RX)         */
} TVS_MsgFrameInfo_t;

extern TVS_MsgFrameInfo_t TVS_LiveInfo[MAX_DOCKS];

/*--------------------------------------------------------------
 * B10. TVS Protocol Enumerations
 *------------------------------------------------------------*/

/** @brief CHARGER_Type values (0x90, Byte 0) */
typedef enum
{
    TVS_CHARGER_TYPE_TVS650W_FRIWO      = 1,
    TVS_CHARGER_TYPE_TVS950W_FRIWO      = 2,
    TVS_CHARGER_TYPE_TVS500W_FRIWO      = 3,
    TVS_CHARGER_TYPE_TVS650W_NAPINO     = 4,
    TVS_CHARGER_TYPE_TVS650W_ANEVOLVE   = 5,
    TVS_CHARGER_TYPE_TVS550W_FRIWO      = 8,
    TVS_CHARGER_TYPE_TVS580W_FRIWO      = 9,
    TVS_CHARGER_TYPE_TVS1350W           = 10,
    TVS_CHARGER_TYPE_TVS1500W_DELTA     = 11,
    TVS_CHARGER_TYPE_3KW_DELTA_OFFBOARD = 22
} TVS_ChargerType_t;

/** @brief CHARGER_ErrorState values (0x90, Byte 4 bits [3:0]) */
typedef enum
{
    TVS_CHARGER_ERR_NONE              = 0,   /**< No error                          */
    TVS_CHARGER_ERR_BATT_SHORT        = 1,   /**< Battery short circuit             */
    TVS_CHARGER_ERR_BATT_REVERSE      = 2,   /**< Battery reverse polarity          */
    TVS_CHARGER_ERR_BATT_VOL_RANGE    = 3,   /**< Battery voltage out of range      */
    TVS_CHARGER_ERR_LOST_CAN          = 4,   /**< Lost CAN bus communication        */
    TVS_CHARGER_ERR_BATT_REPORT       = 5,   /**< Battery reported error            */
    TVS_CHARGER_ERR_BATT_DISCONNECTED = 6,   /**< Battery disconnected              */
    TVS_CHARGER_ERR_OVER_TEMP         = 7,   /**< Charger over temperature          */
    TVS_CHARGER_ERR_OVER_VOLTAGE      = 8,   /**< Charger over voltage              */
    TVS_CHARGER_ERR_OVER_CURRENT      = 9,   /**< Charger over current              */
    TVS_CHARGER_ERR_AC_INPUT_RANGE    = 10,  /**< AC input out of range             */
    TVS_CHARGER_ERR_FAN               = 11,  /**< Fan error                         */
    TVS_CHARGER_ERR_INTERNAL          = 12,  /**< Internal error                    */
    TVS_CHARGER_ERR_TERMINAL_SHORT    = 13,  /**< Terminal shorted                  */
    TVS_CHARGER_ERR_OVER_CHARGE_TIME  = 14   /**< Over-charging time                */
} TVS_ChargerError_t;

/** @brief CHARGER_Fan values (0x90, Byte 4 bit 4) */
typedef enum { TVS_FAN_OFF = 0,  TVS_FAN_ON = 1  } TVS_FanState_t;

/** @brief CHARGER_Output values (0x90, Byte 4 bit 5) */
typedef enum { TVS_OUTPUT_OFF = 0, TVS_OUTPUT_ON = 1 } TVS_OutputState_t;

/** @brief Charger_deratting values (0x90, Byte 4 bit 6) */
typedef enum { TVS_DERATING_NONE = 0, TVS_DERATING_ACTIVE = 1 } TVS_DeratingState_t;

/** @brief BMS_ErrorState values (0x100, Byte 6) */
typedef enum { TVS_BMS_ERR_NONE = 0, TVS_BMS_ERR_STOP_CHARGING = 1 } TVS_BmsError_t;

/*--------------------------------------------------------------
 * B11. TVS GET/SET Function Prototype
 *------------------------------------------------------------*/
/**
 * @brief  Thread-safe read/write of TVS live CAN frame for a dock.
 * @param  u8DockNo    Dock index (0 to MAX_DOCKS-1)
 * @param  psData      Pointer to TVS_MsgFrameInfo_t buffer
 * @param  u8Operation SET_PARA (write) or GET_PARA (read)
 * @return true on success, false on invalid arguments
 */
bool bGetSetTVSBMSData(uint8_t u8DockNo,
                       TVS_MsgFrameInfo_t *psData,
                       uint8_t u8Operation);

#endif /* CHARGING_PROTOCOL == PROTOCOL_TVS_PROP */


/* ============================================================
 * SECTION C: COMMON LED / STATE ENUMERATIONS
 * (shared by both protocols)
 * ============================================================ */
// typedef enum
// {
//     LED_RED   = 1U,
//     LED_GREEN = 2U,
//     LED_BLUE  = 3U,
//     LED_NAME_MAX
// } ledname_e;

// typedef enum
// {
//     LED_STATE_STEADY = 1U,
//     LED_STATE_BLINK
// } ledStatus_e;

/* ============================================================
 * SECTION D: PUBLIC API
 * ============================================================ */
/**
 * @brief  Create and start the CHARGING_TASK FreeRTOS task.
 * @return true  - Task created successfully
 * @return false - Task creation failed
 */
bool ChargingTask_Init(void);

#ifdef __cplusplus
}
#endif

#endif /* CHARGING_HANDLER_H */