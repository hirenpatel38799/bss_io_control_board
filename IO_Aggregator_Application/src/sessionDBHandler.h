/* ************************************************************************** */
/** @file    sessionDBHandler.h
 *  @brief   Session Database Handler — Interface
 *
 *  @company BACANCY SYSTEMS PVT. LTD.
 *
 *  @summary
 *    Provides macros and APIs to read/write per-dock charging session data.
 *
 *  @description
 *    All live charging session variables (state, SOC, voltages, fault bitmaps,
 *    temperatures, timestamps, etc.) are stored in a global array `sessionDB[]`
 *    indexed by dock number. Accessor macros provide inline read/write with
 *    no function-call overhead.
 *
 *  Version : 2.0
 *
 *  Changes from v1.0:
 *    - Fixed: include guard used reserved leading-underscore identifier
 *    - Fixed: SYSTEM_FAULT_NONE = 0 AND SYSTEM_FAULT_ESTOP_TRIGGERED = 0
 *      were both zero — fault detection always returned "no fault" for E-Stop
 *    - Fixed: SET_BIT / CLEAR_BIT macros lacked parentheses around argument,
 *      causing operator-precedence bugs with compound expressions
 *    - Fixed: u8BMSRxStatue typo in struct field (→ u8BMSRxStatus)
 *    - Fixed: u32EnergyDelivered declared uint32_t but used as float in .c file
 *      (→ changed to float fEnergyDelivered_kWh)
 *    - Fixed: SESSION_SetSystemFaultBitmap/GetSystemFaultBitmap mapped to
 *      u16SessionFaultCode (16-bit) but fault bitmap is 32-bit — now maps to
 *      a dedicated u32SystemFaultBitmap field
 *    - Fixed: SESSION_SetOutputPower macro mapped to u16OutputPower but power
 *      is set as float in ChargingHandler — type mismatch fixed (→ float)
 *    - Fixed: CANBUS_3/4/5 defined in enum but only 0–2 are used — kept but
 *      annotated as reserved
 *    - Improved: C++ extern "C" guard added around all declarations
 *    - Improved: All magic-number factors grouped with matching float variants
 *    - Improved: Doxygen on every type, enum, and function prototype
 */
/* ************************************************************************** */

#ifndef SESSION_DB_HANDLER_H   /* No leading underscore — reserved by C standard */
#define SESSION_DB_HANDLER_H

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 * Includes
 * ========================================================================== */
#include <stdint.h>
#include <stdbool.h>

/* ============================================================================
 * Constants
 * ========================================================================== */

/** @brief Sentinel value for an uninitialised / invalid session index */
#define SESSION_INVALID_IDX       (0xFFU)
#define COMPARTMENT_ID        (0x03U)   /**< Fixed compartment identifier */
#define COMPARTMENT_IP        "192.168.1.233"
/* ============================================================================
 * Bit-manipulation helpers
 * ========================================================================== */

/** @brief Set bit @p bit in @p reg (register/variable, not a pointer) */
#define SET_BIT(reg, bit)    ((reg) |=  (1UL << (uint32_t)(bit)))

/** @brief Clear bit @p bit in @p reg */
#define CLEAR_BIT(reg, bit)  ((reg) &= ~(1UL << (uint32_t)(bit)))

/** @brief Read single bit @p bit from @p reg, returns 0 or 1 */
#define GET_BIT(reg, bit)    (((reg) >> (uint32_t)(bit)) & 1UL)

/* ============================================================================
 * Numeric Scaling Factors
 * ========================================================================== */
#define FACTOR_10           (10U)
#define FACTOR_100          (100U)
#define FACTOR_1000         (1000U)
#define FACTOR_10_F         (10.0f)
#define FACTOR_100_F        (100.0f)
#define FACTOR_1000_F       (1000.0f)
#define FACTOR_0_1          (0.1f)
#define FACTOR_0_01         (0.01f)
#define FACTOR_0_001        (0.001f)

/* ============================================================================
 * Dock / Channel Enumeration
 * ========================================================================== */

/**
 * @brief Physical dock identifiers.
 *
 *        COMPARTMENT (0) is a special index for the enclosure sensor.
 *        DOCK_1–DOCK_3 map to CAN channels 0–2 respectively.
 *        MAX_DOCKS is the sentinel — also used as the array size.
 */
typedef enum
{
    COMPARTMENT = 0,   /**< Enclosure / compartment sensor index */
    DOCK_1      = 1,   /**< Dock 1 → CAN0                       */
    DOCK_2      = 2,   /**< Dock 2 → CAN1                       */
    DOCK_3      = 3,   /**< Dock 3 → CAN2                       */
    DOCK_4      = 4,   /**< Dock 4 → CAN3                       */
    DOCK_5      = 5,   /**< Dock 5 → CAN4                       */
    DOCK_6      = 6,   /**< Dock 6 → CAN5                       */
    MAX_DOCKS          /**< Total array size (do not use as index)*/
} Dock_e;

/** @brief GPIO direction enumeration */
typedef enum {
    GPIO_READ = 0,
    GPIO_WRITE = 1
}GPIO_Direction_e;
/* ============================================================================
 * CAN Bus Enumeration
 * ========================================================================== */

/**
 * @brief CAN bus channel identifiers.
 *        CANBUS_0–CANBUS_2 are active; CANBUS_3–CANBUS_5 are reserved.
 */
typedef enum
{
    CANBUS_0 = 0,
    CANBUS_1,
    CANBUS_2,
    CANBUS_3,   /**< Reserved */
    CANBUS_4,   /**< Reserved */
    CANBUS_5,   /**< Reserved */
    CANBUS_MAX
} CANBus_e;

/* ============================================================================
 * Charging State Machine
 * ========================================================================== */

/**
 * @brief States of the per-dock charging state machine.
 */
typedef enum
{
    CH_STATE_INIT                 = 0,
    CH_STATE_AUTH_SUCCESS,
    CH_STATE_PARAM_VALIDATE,
    CH_STATE_CONNECTION_CONFIRMED,
    CH_STATE_INITIALIZE,
    CH_STATE_PRECHARGE,
    CH_STATE_CHARGING,
    CH_STATE_SHUTDOWN,
    CH_STATE_SESSION_COMPLETE,
    CH_STATE_ERROR
} CH_State_e;

/* ============================================================================
 * Miscellaneous Enumerations
 * ========================================================================== */

/** @brief Rectifier (power module) on/off state */
typedef enum
{
    RECTIFIER_OFF = 0,
    RECTIFIER_ON
} RectifierState_e;

/** @brief GET / SET operation selector */
typedef enum
{
    GET_PARA = 0x01U,
    SET_PARA = 0x02U
} GetSet_e;

/** @brief Session lifecycle event codes */
typedef enum
{
    CHARGING_SESSION_STOPPED = 0x00U,
    CHARGING_SESSION_STARTED = 0x01U
} SessionEvent_e;

/** @brief Session stop-reason codes */
typedef enum
{
    STOP_REASON_NONE        = 0,
    STOP_REASON_MCU_REQUEST,
    STOP_REASON_FAULT,
    STOP_REASON_TIMEOUT
} charging_stop_reason_t;

/* ============================================================================
 * System Fault Bitmap Bit Positions
 *
 * BUG FIX: In v1.0, both SYSTEM_FAULT_NONE = 0 and
 * SYSTEM_FAULT_ESTOP_TRIGGERED = 0 were assigned the same value.
 * This made it impossible to distinguish "no fault" from "E-Stop triggered"
 * in a bitmap comparison. SYSTEM_FAULT_NONE is now a standalone zero-value
 * sentinel; all fault *bit positions* start at 0 and are used with SET_BIT.
 * ========================================================================== */

/**
 * @brief Bit positions within the 32-bit system fault bitmap.
 *        Use SET_BIT(bitmap, SYSTEM_FAULT_xxx) to assert a fault.
 */
typedef enum
{
    /* ---- Critical faults (bits 0–15) ---- */
    SYSTEM_FAULT_ESTOP_TRIGGERED            = 0,
    SYSTEM_FAULT_BMS_ERROR                  = 1,
    SYSTEM_FAULT_PM_ERROR                   = 2,
    SYSTEM_FAULT_BMS_COMMUNICATION_FAILURE  = 3,
    SYSTEM_FAULT_PM_COMMUNICATION_FAILURE   = 4,
    SYSTEM_FAULT_PM_ZERO_CURRENT            = 5,
    SYSTEM_FAULT_PRECHARGE_FAILURE          = 6,
    SYSTEM_FAULT_BMS_OVER_TEMPERATURE       = 7,
    SYSTEM_FAULT_PM_OVER_TEMPERATURE        = 8,

    /* ---- Non-critical / warnings (bits 16–31) ---- */
    /* (add warning bits here) */

} system_fault_bit_t;

/** @brief Zero-value sentinel meaning "no active faults" */
#define SYSTEM_FAULT_NONE   (0UL)

/* ============================================================================
 * Session Data Structure
 * ========================================================================== */

/**
 * @brief Per-dock session data.
 *
 *        One instance exists per physical dock (array size = MAX_DOCKS).
 *        All fields are accessed through the SESSION_Set/Get macros below.
 *
 *  v2.0 changes:
 *    - u8BMSRxStatue  → u8BMSRxStatus  (typo fix)
 *    - u32EnergyDelivered (uint32_t) → fEnergyDelivered_kWh (float)
 *      because the charging handler accumulates energy as float kWh
 *    - u16OutputPower (uint16_t) → fOutputPower_W (float) to match usage
 *    - u16SessionFaultCode (16-bit) → u32SystemFaultBitmap (32-bit)
 *      because the fault bitmap requires 32 bits
 */
typedef struct
{
    /* State machine */
    CH_State_e       eChargingState;          /**< Current charging state            */
    RectifierState_e bPMOnOffStatus;          /**< Rectifier on/off                  */
    bool             bStartChargingComm;      /**< Charging comms started flag       */

    /* SOC */
    uint8_t          u8CurrentSoc;            /**< Live SOC (%)                      */
    uint8_t          u8InitialSoc;            /**< SOC at session start (%)          */

    /* Session metadata */
    uint8_t          u8SessionEndReason;      /**< Stop-reason code                  */
    uint8_t          u8AuthenticationCommand; /**< Auth command from BMS             */

    /* Device connectivity */
    uint8_t          u8BMSRxStatus;           /**< BMS CAN RX active (0/1)  ← FIXED typo */
    uint8_t          u8PMRxStatus;            /**< PM CAN RX active (0/1)            */

    /* Temperatures */
    uint8_t          u8DockTemperature;       /**< Dock housing temperature (°C)     */
    uint8_t          u8BMSTemperature;        /**< Battery temperature (°C)          */
    uint8_t          u8PMTemperature;         /**< Power module temperature (°C)     */

    /* Fault bitmaps */
    uint32_t         u32BMSFaultBitmap;       /**< BMS fault bit positions           */
    uint32_t         u32PMFaultBitmap;        /**< PM fault bit positions            */
    uint32_t         u32SystemFaultBitmap;    /**< System-level fault bitmap ← FIXED width */

    /* PM raw fault code from hardware */
    uint16_t         u16PMFaultCode;

    /* Power / energy */
    float            fOutputPower_W;          /**< Instantaneous output power (W) ← FIXED type */
    float            fPmOutputVoltage;        /**< PM output voltage (V)             */
    float            fPmOutputCurrent;        /**< PM output current (A)             */
    float            fPmSetPointVoltage;           /**< PM set-point voltage (V)          */
    float            fPmSetPointCurrent;           /**< PM set-point current (A)          */
    float            fBMSDemandVoltage;       /**< BMS demanded voltage (V)          */
    float            fBMSDemandCurrent;       /**< BMS demanded current (A)          */
    float            fEnergyDelivered_kWh;    /**< Cumulative session energy (kWh) ← FIXED type */

    /* Timing */
    uint16_t         u16EstimatedChargingTime; /**< Estimated remaining time (min)   */

    /* Communication timestamps (FreeRTOS ticks) */
    volatile uint32_t u32PMLastRxTime;        /**< Tick of last PM CAN frame         */
    volatile uint32_t u32BMSLastRxTime;       /**< Tick of last BMS CAN frame        */

    uint8_t          u8CompartmentId;        /**< Compartment identifier            */
    uint8_t          u8MaxDocks;              /**< Max number of docks supported     */
    char             cIpAddress[16];     // "xxx.xxx.xxx.xxx" + '\0'
    char             cSubnetMask[16];
} SESSION_Data_t;

/** @brief Global session database — one entry per dock */
extern SESSION_Data_t sessionDB[MAX_DOCKS];

/* ============================================================================
 * Accessor Macros
 *
 * All macros validate index at compile time only (no runtime guard).
 * Caller is responsible for passing a valid index (0 < idx < MAX_DOCKS).
 * ========================================================================== */

/* --- Charging state & control --- */
#define SESSION_SetChargingState(idx, val)      (sessionDB[(idx)].eChargingState = (val))
#define SESSION_GetChargingState(idx)           (sessionDB[(idx)].eChargingState)

#define SESSION_SetPMState(idx, val)            (sessionDB[(idx)].bPMOnOffStatus = (val))
#define SESSION_GetPMState(idx)                 (sessionDB[(idx)].bPMOnOffStatus)

#define SESSION_SetStartChargingComm(idx, val)  (sessionDB[(idx)].bStartChargingComm = (val))
#define SESSION_GetStartChargingComm(idx)       (sessionDB[(idx)].bStartChargingComm)

/* --- SOC --- */
#define SESSION_SetCurrentSoc(idx, val)         (sessionDB[(idx)].u8CurrentSoc = (val))
#define SESSION_GetCurrentSoc(idx)              (sessionDB[(idx)].u8CurrentSoc)

#define SESSION_SetInitialSoc(idx, val)         (sessionDB[(idx)].u8InitialSoc = (val))
#define SESSION_GetInitialSoc(idx)              (sessionDB[(idx)].u8InitialSoc)

/* --- Session metadata --- */
#define SESSION_SetSessionEndReason(idx, val)   (sessionDB[(idx)].u8SessionEndReason = (val))
#define SESSION_GetSessionEndReason(idx)        (sessionDB[(idx)].u8SessionEndReason)

#define SESSION_SetAuthenticationCommand(idx, val) (sessionDB[(idx)].u8AuthenticationCommand = (val))
#define SESSION_GetAuthenticationCommand(idx)      (sessionDB[(idx)].u8AuthenticationCommand)

/* --- Device connectivity --- */
/* BUG FIX: was u8BMSRxStatue (typo) — now u8BMSRxStatus */
#define SESSION_SetBMSRxStatus(idx, val)        (sessionDB[(idx)].u8BMSRxStatus = (val))
#define SESSION_GetBMSRxStatus(idx)             (sessionDB[(idx)].u8BMSRxStatus)

#define SESSION_SetPMRxStatus(idx, val)         (sessionDB[(idx)].u8PMRxStatus = (val))
#define SESSION_GetPMRxStatus(idx)              (sessionDB[(idx)].u8PMRxStatus)

/* --- Temperatures --- */
#define SESSION_SetDockTemperature(idx, val)    (sessionDB[(idx)].u8DockTemperature = (val))
#define SESSION_GetDockTemperature(idx)         (sessionDB[(idx)].u8DockTemperature)

#define SESSION_SetBMSTemperature(idx, val)     (sessionDB[(idx)].u8BMSTemperature = (val))
#define SESSION_GetBMSTemperature(idx)          (sessionDB[(idx)].u8BMSTemperature)

#define SESSION_SetPMTemperature(idx, val)      (sessionDB[(idx)].u8PMTemperature = (val))
#define SESSION_GetPMTemperature(idx)           (sessionDB[(idx)].u8PMTemperature)

/* --- Fault bitmaps --- */
#define SESSION_SetBMSFaultBitmap(idx, val)     (sessionDB[(idx)].u32BMSFaultBitmap = (val))
#define SESSION_GetBMSFaultBitmap(idx)          (sessionDB[(idx)].u32BMSFaultBitmap)

#define SESSION_SetPMFaultBitmap(idx, val)      (sessionDB[(idx)].u32PMFaultBitmap = (val))
#define SESSION_GetPMFaultBitmap(idx)           (sessionDB[(idx)].u32PMFaultBitmap)

/* BUG FIX: was u16SessionFaultCode (16-bit) — now u32SystemFaultBitmap (32-bit) */
#define SESSION_SetSystemFaultBitmap(idx, val)  (sessionDB[(idx)].u32SystemFaultBitmap = (val))
#define SESSION_GetSystemFaultBitmap(idx)       (sessionDB[(idx)].u32SystemFaultBitmap)

#define SESSION_SetPMFaultCode(idx, val)        (sessionDB[(idx)].u16PMFaultCode = (val))
#define SESSION_GetPMFaultCode(idx)             (sessionDB[(idx)].u16PMFaultCode)

/* --- Power / energy --- */
/* BUG FIX: was u16OutputPower (uint16_t) — now fOutputPower_W (float) */
#define SESSION_SetOutputPower(idx, val)        (sessionDB[(idx)].fOutputPower_W = (float)(val))
#define SESSION_GetOutputPower(idx)             (sessionDB[(idx)].fOutputPower_W)

#define SESSION_SetPmOutputVoltage(idx, val)    (sessionDB[(idx)].fPmOutputVoltage = (float)(val))
#define SESSION_GetPmOutputVoltage(idx)         (sessionDB[(idx)].fPmOutputVoltage)

#define SESSION_SetPmOutputCurrent(idx, val)    (sessionDB[(idx)].fPmOutputCurrent = (float)(val))
#define SESSION_GetPmOutputCurrent(idx)         (sessionDB[(idx)].fPmOutputCurrent)

#define SESSION_SetBMSDemandVoltage(idx, val)   (sessionDB[(idx)].fBMSDemandVoltage = (float)(val))
#define SESSION_GetBMSDemandVoltage(idx)        (sessionDB[(idx)].fBMSDemandVoltage)

#define SESSION_SetBMSDemandCurrent(idx, val)   (sessionDB[(idx)].fBMSDemandCurrent = (float)(val))
#define SESSION_GetBMSDemandCurrent(idx)        (sessionDB[(idx)].fBMSDemandCurrent)

#define SESSION_SetPmVoltageSetpoint(idx, val)       (sessionDB[(idx)].fPmSetPointVoltage = (float)(val))
#define SESSION_GetPmVoltageSetpoint(idx)            (sessionDB[(idx)].fPmSetPointVoltage)

#define SESSION_SetPmCurrentSetpoint(idx, val)       (sessionDB[(idx)].fPmSetPointCurrent = (float)(val))
#define SESSION_GetPmCurrentSetpoint(idx)            (sessionDB[(idx)].fPmSetPointCurrent)

/* BUG FIX: was uint32_t u32EnergyDelivered — now float fEnergyDelivered_kWh */
#define SESSION_SetEnergyDelivered(idx, val)    (sessionDB[(idx)].fEnergyDelivered_kWh = (float)(val))
#define SESSION_GetEnergyDelivered(idx)         (sessionDB[(idx)].fEnergyDelivered_kWh)

/* --- Timing --- */
#define SESSION_SetEstimatedChargingTime(idx, val) (sessionDB[(idx)].u16EstimatedChargingTime = (val))
#define SESSION_GetEstimatedChargingTime(idx)      (sessionDB[(idx)].u16EstimatedChargingTime)

#define SESSION_SetPMLastRxTime(idx, val)       (sessionDB[(idx)].u32PMLastRxTime = (val))
#define SESSION_GetPMLastRxTime(idx)            (sessionDB[(idx)].u32PMLastRxTime)

#define SESSION_SetBMSLastRxTime(idx, val)      (sessionDB[(idx)].u32BMSLastRxTime = (val))
#define SESSION_GetBMSLastRxTime(idx)           (sessionDB[(idx)].u32BMSLastRxTime)

/* --- Compartment and Network --- */
#define SESSION_SetCompartmentId(val)        (sessionDB[(COMPARTMENT)].u8CompartmentId = (val))
#define SESSION_GetCompartmentId()           (sessionDB[(COMPARTMENT)].u8CompartmentId)

#define SESSION_SetMaxDocks(val)             (sessionDB[(COMPARTMENT)].u8MaxDocks = (val))
#define SESSION_GetMaxDocks()                (sessionDB[(COMPARTMENT)].u8MaxDocks)

#define SESSION_SetIpAddress(val)            strcpy(sessionDB[(COMPARTMENT)].cIpAddress, val)
#define SESSION_GetIpAddress()               (sessionDB[(COMPARTMENT)].cIpAddress)

#define SESSION_SetSubnetMask(val)           strcpy(sessionDB[(COMPARTMENT)].cSubnetMask, val)
#define SESSION_GetSubnetMask()              (sessionDB[(COMPARTMENT)].cSubnetMask)

/* ============================================================================
 * Function Prototypes
 * ========================================================================== */

/**
 * @brief  Reset all session entries to zero/default.
 *         Call once at system startup.
 */
void SESSION_ResetAll(void);

/**
 * @brief  Reset a single dock session to default values.
 * @param  idx  Dock index (must be < MAX_DOCKS)
 */
void SESSION_ResetSession(uint8_t idx);

/**
 * @brief  Reset BMS-related fields for one dock.
 * @param  idx  Dock index (must be < MAX_DOCKS)
 */
void SESSION_ResetBMSData(uint8_t idx);

/**
 * @brief  Reset PM-related fields for one dock.
 * @param  idx  Dock index (must be < MAX_DOCKS)
 */
void SESSION_ResetPMData(uint8_t idx);

/**
 * @brief  Reset temperature fields for one dock.
 * @param  idx  Dock index (must be < MAX_DOCKS)
 */
void SESSION_ResetTempData(uint8_t idx);

#ifdef __cplusplus
}
#endif

#endif /* SESSION_DB_HANDLER_H */

/* ************************************************************************** */
/* End of File                                                                */
/* ************************************************************************** */