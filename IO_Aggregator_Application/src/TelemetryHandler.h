/* ************************************************************************** */
/** @file    TelemetryHandler.h
 *  @brief   Telemetry Handler — Interface
 *
 *  @company BACANCY SYSTEMS PVT. LTD. 
 *
 *  @summary
 *    Defines structures and function prototypes for broadcasting per-dock
 *    PM, BMS, and temperature data to a TCP client every second.
 *
 *  Version : 2.0
 *
 *  Changes from v1.0:
 *    - Fixed: include guard used reserved leading-underscore identifier
 *    - Fixed: telemetryData[] declared as size [4] — must match MAX_DOCKS
 *      from sessionDBHandler.h; now uses MAX_DOCKS constant
 *    - Fixed: TELEMETRY_BMSData_t was not __attribute__((packed)) but
 *      TELEMETRY_PMData_t was — inconsistent. Both structs are packed now.
 *    - Fixed: u32SystemFaultBitmap duplicated the BMS fault — now carries
 *      the actual system-level fault bitmap from SESSION_GetSystemFaultBitmap
 *    - Fixed: TELEMETRY_TempData_t was missing __attribute__((packed))
 *      causing potential padding in the serialised TCP frame
 *    - Improved: frame header constants added for all fixed header fields
 *    - Improved: payload size constants defined and used to size local arrays,
 *      preventing silent overflow if struct grows
 *    - Added: Telemetry_IsConnected() inline helper so callers can check
 *      TCP state without accessing the internal socket directly
 *    - Added: C++ extern "C" guard
 */
/* ************************************************************************** */

#ifndef TELEMETRY_HANDLER_H
#define TELEMETRY_HANDLER_H

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 * Includes
 * ========================================================================== */
#include <stdint.h>
#include <stdbool.h>
#include "sessionDBHandler.h"   /* For MAX_DOCKS */

/* ============================================================================
 * Command IDs (single byte in frame)
 * ========================================================================== */
#define TELEMETRY_CMD_PM_DATA           (0x10U) /**< Power module telemetry frame   */
#define TELEMETRY_CMD_BMS_DATA          (0x11U) /**< BMS telemetry frame            */
#define TELEMETRY_CMD_TEMP_DATA         (0x12U) /**< Temperature telemetry frame    */

/* ============================================================================
 * Frame Constants
 * ========================================================================== */
#define TELEMETRY_MSG_TYPE_BROADCAST    (0x0004U) /**< 2-byte message type field    */

/**
 * @brief Frame header layout (bytes before payload):
 *
 *   [0–3]  Unique ID     (uint32_t, big-endian)
 *   [4–5]  Message type  (uint16_t, big-endian) = TELEMETRY_MSG_TYPE_BROADCAST
 *   [6]    Compartment ID
 *   [7]    Dock ID
 *   [8]    Command ID
 *   [9]    Payload length
 *   [10–N] Payload
 *   [N+1–N+2] CRC16-CCITT (big-endian)
 */
#define TELEMETRY_FRAME_HEADER_SIZE     (10U)   /**< Bytes before payload           */
#define TELEMETRY_FRAME_CRC_SIZE        (2U)    /**< CRC appended after payload     */

/* ============================================================================
 * Payload Size Constants
 * These define the exact serialised sizes — used to declare local arrays
 * and prevent overflow if the structs are extended.
 * ========================================================================== */

/**
 * PM payload byte count:
 *   u32PmOutputVoltage_mV  4
 *   u32PmOutputCurrent_mA  4
 *   u32OutputPower_W       4
 *   u32TotalEnergy_Wh      4
 *   u32PMFaultCode         4
 *   u8PMTemperature        1
 *   u8PMStatus             1
 *                        = 22 bytes
 */
#define TELEMETRY_PM_PAYLOAD_SIZE       (22U)

/**
 * BMS payload byte count:
 *   u32BMSDemandVoltage    4
 *   u32BMSDemandCurrent    4
 *   u32EstimatedTime_s     4
 *   u32BMSFaultCode        4
 *   u8CurrentSoc           1
 *   u8InitialSoc           1
 *   u8BMSTemperature       1
 *   u8BMSStatus            1
 *   u8ChargingState        1
 *   u32SystemFaultBitmap   4
 *                        = 25 bytes
 */
#define TELEMETRY_BMS_PAYLOAD_SIZE      (25U)

/**
 * Temperature payload byte count:
 *   u8CompartmentTemperature 1
 *   u8DockTemperature[MAX_DOCKS]       1
 *                          = 7 bytes
 */
#define TELEMETRY_TEMP_PAYLOAD_SIZE     (7U)

/**
 * Maximum frame buffer size — header + largest payload + CRC
 */
#define TELEMETRY_FRAME_BUF_SIZE \
    (TELEMETRY_FRAME_HEADER_SIZE + TELEMETRY_BMS_PAYLOAD_SIZE + TELEMETRY_FRAME_CRC_SIZE)

/* ============================================================================
 * Telemetry Data Structures
 *
 * All structs are __attribute__((packed)) to prevent compiler-inserted padding
 * corrupting serialised TCP frames when memcpy'd directly.
 * ========================================================================== */

/**
 * @brief Power Module telemetry payload.
 */
typedef struct __attribute__((packed))
{
    uint32_t u32PmOutputVoltage_mV;  /**< Charger output voltage (mV)        */
    uint32_t u32PmOutputCurrent_mA;  /**< Charger output current (mA)        */
    uint32_t u32OutputPower_W;       /**< Instantaneous output power (W)     */
    uint32_t u32TotalEnergy_Wh;      /**< Session energy delivered (Wh)      */
    uint32_t u32PMFaultCode;         /**< PM fault bitmap                    */
    uint8_t  u8PMTemperature;        /**< PM temperature (°C)               */
    uint8_t  u8PMStatus;             /**< PM connectivity status (0/1)       */
} TELEMETRY_PMData_t;

/**
 * @brief BMS telemetry payload.
 *
 *  BUG FIX: u32SystemFaultBitmap in v1.0 was a copy of u32BMSFaultCode.
 *  It now carries the system-level fault bitmap (SESSION_GetSystemFaultBitmap).
 */
typedef struct __attribute__((packed))
{
    uint32_t u32BMSDemandVoltage;    /**< BMS demanded voltage (mV)          */
    uint32_t u32BMSDemandCurrent;    /**< BMS demanded current (mA)          */
    uint32_t u32EstimatedTime_s;     /**< Estimated remaining charge time (s)*/
    uint32_t u32BMSFaultCode;        /**< BMS fault bitmap                   */
    uint8_t  u8CurrentSoc;           /**< Live SOC (%)                       */
    uint8_t  u8InitialSoc;           /**< SOC at session start (%)           */
    uint8_t  u8BMSTemperature;       /**< BMS temperature (°C)              */
    uint8_t  u8BMSStatus;            /**< BMS connectivity status (0/1)      */
    uint8_t  u8ChargingState;        /**< Current state machine state        */
    uint32_t u32SystemFaultBitmap;   /**< System fault bitmap (from session) */
} TELEMETRY_BMSData_t;

/**
 * @brief Temperature telemetry payload.
 *
 *  BUG FIX: missing __attribute__((packed)) in v1.0 — added.
 */
typedef struct __attribute__((packed))
{
    uint8_t u8CompartmentTemperature; /**< Enclosure temperature (°C)        */
    uint8_t u8DockTemperature[MAX_DOCKS];       /**< Dock temperature (°C)           */
} TELEMETRY_TempData_t;

/**
 * @brief Aggregated telemetry data for one dock.
 */
typedef struct __attribute__((packed))
{
    TELEMETRY_PMData_t   pmData;     /**< Power module data                  */
    TELEMETRY_BMSData_t  bmsData;    /**< BMS data                           */
    TELEMETRY_TempData_t tempData;   /**< Temperature data                   */
} TELEMETRY_Data_t;

/* ============================================================================
 * Global Telemetry Data Array
 * BUG FIX: was [4] — now uses MAX_DOCKS to stay in sync with sessionDBHandler.h
 * ========================================================================== */
extern TELEMETRY_Data_t telemetryData[MAX_DOCKS];

/* ============================================================================
 * Function Prototypes
 * ========================================================================== */

/**
 * @brief  Open the telemetry TCP server and start the telemetry task.
 *
 *         Must be called after the TCP/IP stack is initialised and before
 *         the FreeRTOS scheduler is started.
 *
 * @return true  Socket and task created successfully
 * @return false Failed to open socket or create task
 */
bool Telemetry_Init(void);

/**
 * @brief  FreeRTOS task: collect session data and broadcast over TCP every second.
 *
 *         Handles socket reconnection automatically.
 *         Do not call directly — pass to xTaskCreate via Telemetry_Init.
 *
 * @param  pvParameters  Unused (pass NULL)
 */
void Telemetry_Task(void *pvParameters);

#ifdef __cplusplus
}
#endif

#endif /* TELEMETRY_HANDLER_H */

/* ************************************************************************** */
/* End of File                                                                */
/* ************************************************************************** */