/*******************************************************************************
 * File Name   : IOHandler.h
 * Company     : Bacancy - SunMobility
 * Summary     : IO Handler — header file
 *
 * Description :
 *   Provides function prototypes, type definitions, and configuration macros
 *   for the IO Aggregator board. Covers:
 *     - I2C IO expander (TCA9539) control
 *     - ADC temperature / analog input reading
 *     - Digital input / output management
 *     - Flash-backed configuration storage
 *     - TCP command frame parsing and dispatch
 *
 * Version     : 2.0
 *
 * Changes from v1.0:
 *   - Fixed: include guard used reserved leading-underscore identifier
 *   - Fixed: duplicate `uint16_t serialnum` definition (declared twice)
 *   - Fixed: duplicate `uint32_t digitalOutputs` / `uint16_t relayOutputs`
 *     definitions — declared in header AND defined without initialisation in .c
 *   - Fixed: `ADC_TO_VOLT_CONVERTER` macro was missing parentheses around `x`
 *     causing operator-precedence bugs
 *   - Fixed: conflicting COEFF_A/B/C and REF_RESISTOR defined in both header
 *     and .c file — removed from header (they belong only in .c as statics)
 *   - Fixed: `extern uint32_t *ramStart` declared in header but also defined
 *     with pointer cast in .c — cleaned up
 *   - Fixed: `TCP_RequestFrame_t.payload` is `uint8_t*` pointing into a
 *     receive buffer — noted as non-owning pointer to prevent misuse
 *   - Improved: all magic numbers replaced with named constants
 *   - Improved: `vIOHandler` return type is now `bool` to signal task creation failure
 *   - Improved: C++ extern "C" guard added
 *******************************************************************************/

#ifndef IO_HANDLER_H   /* No leading underscore — reserved by C standard */
#define IO_HANDLER_H

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 * Includes
 * ========================================================================== */
#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>
#include "sessionDBHandler.h"

/* ============================================================================
 * FreeRTOS Task Configuration
 * ========================================================================== */
#define IO_SERVER_HANDLER_STACK_DEPTH   (1024U)             /**< Task stack (words)     */
#define IO_SERVER_HANDLER_TASK_PRIORITY (tskIDLE_PRIORITY + 2U) /**< Task priority      */

/* ============================================================================
 * Board / Firmware Version Selection
 * Exactly one of these must be 1; the other must be 0.
 * ========================================================================== */
#define HEV_IO_Aggregator           (1)  /**< HEV aggregator build             */
#define Two_Wheeler_IO_Aggregator   (0)  /**< Two-wheeler aggregator build     */

#if (HEV_IO_Aggregator == 1) && (Two_Wheeler_IO_Aggregator == 1)
    #error "Only one aggregator type may be selected at a time"
#endif

#if HEV_IO_Aggregator
    #define SYS_VERSION_STR     "0.0.1"
    #define SYS_FW_VERSION      (0x000001U)
    #define SYS_HW_VERSION      (0x000102U)
#endif

#if Two_Wheeler_IO_Aggregator
    #define SYS_VERSION_STR     "0.1.3"
    #define SYS_FW_VERSION      (0x000103U)
    #define SYS_HW_VERSION      (0x000102U)
#endif

/* ============================================================================
 * Server Port Configuration
 * ========================================================================== */
#define IO_SERVER_PORT_HEV          (8888U)   /**< TCP port for IO server      */
#define IO_SERVER_PORT_WHEELER      (8888U)   /**< TCP port (two-wheeler)      */
#define RECONNECT_DELAY_MS          (5000U)   /**< Socket reconnect interval   */

/* ============================================================================
 * I2C IO Expander (TCA9539) Configuration
 * ========================================================================== */
#define TCA9539_I2C_ADDRESS         (0x75U)   /**< I2C address of IO expander 1 */

/** TCA9539 register map */
#define INPUT_PORT_REG              (0x00U)   /**< Input port 0 (P00–P07)      */
#define INPUT_PORT_REG_2            (0x01U)   /**< Input port 1 (P10–P17)      */
#define TCA9539_OUTPUT_PORT_0       (0x02U)   /**< Output port 0               */
#define TCA9539_OUTPUT_PORT_1       (0x03U)   /**< Output port 1               */
#define TCA9539_CONFIG_PORT0        (0x06U)   /**< Configuration port 0        */
#define TCA9539_CONFIG_PORT1        (0x07U)   /**< Configuration port 1        */

/* ============================================================================
 * ADC Configuration
 * ========================================================================== */
#define ADC_VREF                    (3.3f)    /**< ADC reference voltage (V)   */
#define ADC_MAX_VALUE               (4095U)   /**< 12-bit ADC full-scale count */

/**
 * @brief Convert raw ADC count to voltage.
 * BUG FIX: original macro was missing parentheses around x:
 *   `x * (ADC_VREF / ADC_MAX_VALUE)` → x*a which looks fine alone but
 *   `ADC_TO_VOLT_CONVERTER(a+b)` expands to `a+b * (...)` = wrong.
 *   Fixed: `(x) * (ADC_VREF / (float)ADC_MAX_VALUE)`
 */
#define ADC_TO_VOLT_CONVERTER(x)    ((x) * (ADC_VREF / (float)ADC_MAX_VALUE))

/** Sensor type selection — set exactly one to 1 */
#define PT100_Sensor                (1)
#define NTC_Sensor                  (0)

/** ADC channel counts */
#define NUM_TEMPERATURE_ANALOG_PINS (16U)     /**< Temperature ADC channels    */
#define NUM_ANALOG_PINS             (4U)      /**< Non-temperature ADC channels*/
#define ALL_ANALOG_PINS             (20U)     /**< Total ADC channels          */

/* ============================================================================
 * TCP Frame Limits
 * ========================================================================== */
#define TCP_MIN_FRAME_SIZE          (12U)     /**< Minimum valid frame (bytes) */
#define MESSAGE_BUFFER_SIZE         (2048U)   /**< Analog message buffer size  */

/* ============================================================================
 * Bootloader Trigger
 * ========================================================================== */
#define BTL_TRIGGER_PATTERN         (0x5048434DUL) /**< Magic RAM pattern      */
#define BTL_TRIGGER_RAM_START       (0x20020000U)  /**< RAM address for pattern*/

/* ============================================================================
 * Pin Count Definitions
 * ========================================================================== */
#define NUM_DIGITAL_INPUTS          (60U)   /**< Total direct + expander DI   */
#define NUM_DIGITAL_OUTPUTS         (24U)   /**< Total DO channels            */
#define RELAY_COUNT                 (6U)    /**< Total relay / eFuse outputs  */

/* ============================================================================
 * Flash Configuration
 * ========================================================================== */
#define FLASH_MAGIC                 (0xDEADBEEFUL) /**< Flash validity token  */
#define FLASH_ADDRESS               (0x0C0F0000UL) /**< Flash config start addr*/

/* =========================
 * FLASH CONFIG MAC
 * ========================= */
#define MAC_FLASH_ADDRESS      (0x0C0F1000UL)

#define MAC_FLASH_MAGIC        (0x4D414346UL)   /* 'MACF' */

#define FCW_TIMEOUT_CYCLES     (1000000UL)

/* ============================================================================
 * Extern Global Variables
 * (minimised — only expose what other modules genuinely need)
 * ========================================================================== */
extern uint32_t *ramStart;          /**< Pointer to bootloader trigger RAM    */
extern char     messageAnalog[MESSAGE_BUFFER_SIZE];
extern uint32_t total_length;

/* I2C shared buffers */
extern uint8_t  i2cTxBuf[2];
extern uint8_t  i2cRxBuf[2];
extern bool     i2cTransferDone;

/* ADC result buffers — volatile because written by ISR/hardware */
extern volatile uint32_t adc_data[NUM_ANALOG_PINS];

/* Output status mirrors */
extern uint32_t doStatus;      /**< Digital output state bitmap               */
extern uint16_t relayStatus;   /**< Relay output state bitmap                 */
extern uint16_t serialnum;     /**< Device serial number                      */

/* ============================================================================
 * Flash Configuration Structure
 * ========================================================================== */

/**
 * @brief Persistent configuration stored in flash.
 *
 *        The `magic` field must equal FLASH_MAGIC for data to be considered
 *        valid. All other fields are zeroed and re-defaulted if magic fails.
 */
typedef struct
{
    uint8_t u8CompartmentId;
    uint8_t u8MaxDocks;
    char cIpAddress[16];     // "xxx.xxx.xxx.xxx" + '\0'
    char cSubnetMask[16];
} IOC_Config_t;
typedef struct __attribute__((packed))
{
    uint32_t magic;                  /**< Validity token = FLASH_MAGIC        */
    uint32_t digitalOutputs;        /**< Saved DO bitmap                     */
    uint16_t relayOutputs;          /**< Saved relay bitmap                  */
    uint16_t serialNumber;          /**< Device serial number                */
    /* CAN port / baud configuration */
    uint16_t canPorts[6];
    uint32_t canBaudRates[6];
    /* RS-485 port / UART configuration */
    uint16_t rs485Ports[2];
    struct
    {
        uint32_t baudRate;
        uint8_t  dataBits;
        char     parity;
        uint8_t  stopBits;
    } rs485Config[2];
    /* IOC configuration */
    IOC_Config_t iocCfg;   // Added
} flash_data_t;

extern flash_data_t writeData;  /**< In-RAM mirror of flash config           */

/* =========================
 * MAC STORAGE STRUCTURE
 * ========================= */

typedef struct
{
    uint32_t magic;
    uint8_t  mac_last_byte;
    uint8_t  reserved[3];
} mac_flash_data_t;
/* =========================
 * GLOBAL MAC STORAGE
 * ========================= */

// extern static mac_flash_data_t gMacData;
/* ============================================================================
 * TCP Request Frame
 * ========================================================================== */

/**
 * @brief Parsed TCP request frame header.
 *
 * @note `payload` is a non-owning pointer into the caller's receive buffer.
 *       It is valid only until the buffer is overwritten.
 */
typedef struct __attribute__((packed))
{
    uint32_t  uniqueId;        /**< Frame sequence number                    */
    uint16_t  msgType;         /**< Frame type (0x0001 = request)            */
    uint8_t   compartmentId;   /**< Compartment identifier                   */
    uint8_t   dockId;          /**< Dock identifier                          */
    uint8_t   commandId;       /**< Command (CMD_xxx)                        */
    uint8_t   payloadLen;      /**< Payload length in bytes                  */
    uint8_t  *payload;         /**< Non-owning pointer into RX buffer        */
} TCP_RequestFrame_t;

/* ============================================================================
 * Command IDs
 * ========================================================================== */

/**
 * @brief TCP command identifiers dispatched by ParseAndProcessInbPacket.
 */
typedef enum
{
    CMD_GPIO_OPERATION    = 0x01U,
    CMD_ANALOG_READ       = 0x02U,
    CMD_CHARGING_COMMAND  = 0x03U,
    CMD_EFUSE_COMMAND     = 0x04U,
    CMD_BOOT_MODE_COMMAND = 0x05U,
    CMD_SOFT_RESET_COMMAND= 0x06U,
    CMD_COMMISSIONING_COMMAND = 0x07U,
} CommandID_t;

/* ============================================================================
 * GPIO Pin Assignment Table
 * ========================================================================== */

/**
 * @brief Board GPIO configuration.
 *
 *        Pin numbers map to the port numbers used by GPIO_Read / GPIO_Write.
 *        Arrays are indexed [DOCK_1–1] through [DOCK_3–1] (zero-based).
 *
 *        BUG FIX: array sizes use MAX_DOCKS so they are always in sync with
 *        the session DB dock count.
 */
typedef struct __attribute__((packed))
{
    /* Digital outputs — per dock */
    uint8_t AC_Relay_Pin[MAX_DOCKS];
    uint8_t DC_Relay_Pin[MAX_DOCKS];
    uint8_t Solenoid_PinHi[MAX_DOCKS];
    uint8_t Solenoid_PinLo[MAX_DOCKS];
    uint8_t R_LED_Pin[MAX_DOCKS];
    uint8_t G_LED_Pin[MAX_DOCKS];
    uint8_t B_LED_Pin[MAX_DOCKS];
    uint8_t Dock_Fan_Pin[MAX_DOCKS];
    /* Digital outputs — compartment-level */
    uint8_t Compartment_Fan_Pin;
    /* Digital inputs — per dock */
    uint8_t DoorLock_Pin[MAX_DOCKS];
    uint8_t SolenoidLock_Pin[MAX_DOCKS];
    uint8_t IgnitionSence_Pin[MAX_DOCKS];
    /* Digital inputs — global */
    uint8_t EStop_Pin;
} Board_gpio_st;

/* ============================================================================
 * GPIO Operation Enumeration
 * ========================================================================== */

/**
 * @brief GPIO operations passed to bGPIO_Operation().
 */
typedef enum
{
    DO_AC_RELAY_ON              = 0,
    DO_AC_RELAY_OFF,
    DO_DC_RELAY_ON,
    DO_DC_RELAY_OFF,
    DO_SOLENOID_HIGH,
    DO_SOLENOID_LOW,
    DO_R_LED_HIGH,
    DO_R_LED_LOW,
    DO_G_LED_HIGH,
    DO_G_LED_LOW,
    DO_B_LED_HIGH,
    DO_B_LED_LOW,
    DO_DOCK_FAN_HIGH,
    DO_DOCK_FAN_LOW,
    DO_COMPARTMENT_FAN_HIGH,
    DO_COMPARTMENT_FAN_LOW,
    DI_E_STOP_STATUS,
    DI_DOOR_LOCK_STATUS,
    DI_SOLENOID_LOCK_STATUS,
    DI_BP_STATUS,
    GPIO_OPERATION_MAX           /**< Sentinel — not a valid operation        */
} GPIOOperation_e;

/* ============================================================================
 * Function Prototypes — Public API
 * ========================================================================== */

/**
 * @brief  Create the IO handler FreeRTOS task.
 * @return true  on successful task creation
 * @return false on failure (xTaskCreate returned != pdPASS)
 */
bool vIOHandler(void);

/**
 * @brief  Read all ADC channels, apply moving average, and update session DB temperatures.
 */
void ReadAllAnalogInputPins(void);

/**
 * @brief  Configure TCA9539 IO expander 1 port directions.
 * @return true on success
 */
bool IOExpander1_Configure_Ports(void);

/**
 * @brief  Read and process TCA9539 IO expander 1 inputs.
 */
void IOExpander1_Data(void);

/**
 * @brief  Evaluate IO expander input byte and update digital input buffer.
 * @param  expanderData  Pointer to 2-byte expander data (port 0, port 1)
 */
void checkIOExpanderInput(uint8_t *expanderData);

/**
 * @brief  Read one register from TCA9539 via I2C.
 * @param  reg  Register address
 * @return Register value (0 on I2C failure)
 */
uint8_t TCA9539_ReadRegister(uint8_t reg);

/**
 * @brief  Perform a named GPIO operation for the specified dock.
 *
 *         Maps the high-level operation (DO_AC_RELAY_ON etc.) to the correct
 *         physical pin for the given dock and calls GPIO_Read or GPIO_Write.
 *
 * @param  eGPIOType  Operation to perform
 * @param  u8DockNo   Dock index — used as array index into Gpio_conf arrays.
 *                    Must satisfy: 0 < u8DockNo < MAX_DOCKS for per-dock ops.
 *                    Compartment-level ops (DO_COMPARTMENT_FAN_*) ignore this.
 * @param  eGPIODirection  GPIO direction (GPIO_READ or GPIO_WRITE)
 * @return true   for write ops (always), or pin state for read ops
 * @return false  on invalid operation or pin error
 */
bool bGPIO_Operation(GPIOOperation_e eGPIOType, uint8_t u8DockNo, GPIO_Direction_e eGPIODirection);

/**
 * @brief  Save digital and relay output states to flash.
 * @param  u32Digital   32-bit digital output bitmap
 * @param  u16Relay     16-bit relay output bitmap
 * @param  u16SerialNum Device serial number
 */
void saveOutputsToFlash(uint32_t u32Digital, uint16_t u16Relay, uint16_t u16SerialNum);

/**
 * @brief  Update moving average for one ADC channel.
 * @param  channel   Channel index (0 to ALL_ANALOG_PINS-1)
 * @param  newValue  New raw ADC sample
 * @return Averaged 16-bit ADC value
 */
uint16_t MovingAverage_Update(uint8_t channel, uint16_t newValue);

/**
 * @brief  Write I2C data to TCA9539.
 * @param  address   7-bit I2C address
 * @param  wrData    Pointer to data buffer
 * @param  wrLength  Number of bytes to write
 * @return true on success
 */
bool SERCOM7_I2C_Write(uint16_t address, uint8_t *wrData, uint32_t wrLength);

/**
 * @brief  Read I2C data from TCA9539.
 * @param  address   7-bit I2C address
 * @param  rdData    Pointer to receive buffer
 * @param  rdLength  Number of bytes to read
 * @return true on success
 */
bool SERCOM7_I2C_Read(uint16_t address, uint8_t *rdData, uint32_t rdLength);

/* =========================
 * API PROTOTYPES
 * ========================= */

/**
 * @brief Initialize MAC system (read flash or load default)
 *
 * @return true  MAC read successfully from flash
 * @return false MAC was invalid, default was used and stored
 */
bool MAC_Init(void);

/**
 * @brief Read MAC from flash into internal buffer
 *
 * @return true  valid MAC read
 * @return false invalid flash, default used
 */
bool MAC_ReadFromFlash(void);

/**
 * @brief Write MAC last byte to flash
 *
 * @param lastByte last byte of MAC address
 * @return true write success
 * @return false write failed
 */
bool MAC_WriteToFlash(uint8_t lastByte);

/**
 * @brief Get current MAC pointer
 */
const uint8_t* MAC_Get(void);

/**
 * @brief Convert MAC to string format
 */
void MAC_ToString(const uint8_t *mac, char *str);

/**
 * @brief Print MAC on console
 */
void MAC_Print(const uint8_t *mac);

#ifdef __cplusplus
}
#endif

#endif /* IO_HANDLER_H */

/*******************************************************************************
 * End of File
 *******************************************************************************/