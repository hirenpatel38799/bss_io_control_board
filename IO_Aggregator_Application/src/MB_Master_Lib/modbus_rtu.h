/*******************************************************************************
 * @file    modbus_rtu.h
 * @brief   Modbus RTU Frame Builder & Parser - Header
 * @details Handles raw byte-level Modbus RTU framing:
 *            - Frame construction (request serialisation)
 *            - Frame parsing and validation (response deserialisation)
 *            - RX byte-accumulation state machine
 *            - Inter-frame gap detection (T3.5 silence)
 *
 *          This layer sits between the UART port HAL and the Modbus Master
 *          state machine.  It knows about bytes and frames but does NOT
 *          schedule requests or implement retries — that is the master's job.
 *
 * @company BACANCY SYSTEMS PVT. LTD.
 *******************************************************************************/

#ifndef MODBUS_RTU_H
#define MODBUS_RTU_H

/* =========================================================================
 * Included Files
 * ========================================================================= */
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "uart_port.h"

/* =========================================================================
 * Configuration Constants
 * ========================================================================= */

/** Maximum Modbus RTU frame size in bytes (256 PDU + 1 addr + 2 CRC) */
#define MODBUS_RTU_MAX_FRAME_SIZE       (256U)

/** Minimum valid Modbus RTU frame: SlaveID(1) + FC(1) + Data(0) + CRC(2) */
#define MODBUS_RTU_MIN_FRAME_SIZE       (4U)

/** Maximum number of data bytes in a single response (FC03/FC04 limit) */
#define MODBUS_RTU_MAX_DATA_BYTES       (250U)

/* =========================================================================
 * Type Definitions
 * ========================================================================= */

/**
 * @brief  Modbus function codes supported by this stack.
 */
typedef enum
{
    MB_FC_READ_COILS              = 0x01U,
    MB_FC_READ_DISCRETE_INPUTS    = 0x02U,
    MB_FC_READ_HOLDING_REGISTERS  = 0x03U,
    MB_FC_READ_INPUT_REGISTERS    = 0x04U,
    MB_FC_WRITE_SINGLE_COIL       = 0x05U,
    MB_FC_WRITE_SINGLE_REGISTER   = 0x06U,
    MB_FC_WRITE_MULTIPLE_COILS    = 0x0FU,
    MB_FC_WRITE_MULTIPLE_REGISTERS = 0x10U,
    MB_FC_EXCEPTION_OFFSET        = 0x80U   /**< OR'd with FC in exception responses */
} ModbusFunctionCode_t;

/**
 * @brief  Modbus exception codes (per specification Table 7).
 */
typedef enum
{
    MB_EX_NONE                    = 0x00U,
    MB_EX_ILLEGAL_FUNCTION        = 0x01U,
    MB_EX_ILLEGAL_DATA_ADDRESS    = 0x02U,
    MB_EX_ILLEGAL_DATA_VALUE      = 0x03U,
    MB_EX_SERVER_DEVICE_FAILURE   = 0x04U,
    MB_EX_ACKNOWLEDGE             = 0x05U,
    MB_EX_SERVER_DEVICE_BUSY      = 0x06U,
    MB_EX_MEMORY_PARITY_ERROR     = 0x08U,
    MB_EX_GATEWAY_PATH_UNAVAIL    = 0x0AU,
    MB_EX_GATEWAY_TARGET_NO_RESP  = 0x0BU
} ModbusException_t;

/**
 * @brief  RTU frame receiver state machine states.
 */
typedef enum
{
    MODBUS_RTU_RX_IDLE = 0U,    /**< Waiting for first byte after silence */
    MODBUS_RTU_RX_RECEIVING,    /**< Accumulating bytes                   */
    MODBUS_RTU_RX_FRAME_READY   /**< Complete frame awaiting processing   */
} ModbusRtu_RxState_t;

/**
 * @brief  Parsed and validated Modbus RTU response frame.
 */
typedef struct
{
    uint8_t           u8SlaveAddr;                         /**< Responding slave address       */
    ModbusFunctionCode_t eFuncCode;                        /**< Function code                  */
    bool              bIsException;                        /**< true → exception response       */
    ModbusException_t eExceptionCode;                      /**< Exception code (if exception)   */
    uint8_t           au8Data[MODBUS_RTU_MAX_DATA_BYTES];  /**< Response data bytes             */
    uint8_t           u8DataLen;                           /**< Number of valid data bytes      */
} ModbusRtu_Response_t;

/**
 * @brief  Built Modbus RTU request frame (ready to transmit).
 */
typedef struct
{
    uint8_t  au8Frame[MODBUS_RTU_MAX_FRAME_SIZE];   /**< Complete frame bytes (incl. CRC) */
    uint16_t u16Length;                             /**< Total frame length               */
} ModbusRtu_Request_t;

/**
 * @brief  Per-channel RX accumulator context.
 *         One instance per UartPort_Channel_t.
 */
typedef struct
{
    uint8_t             au8Buffer[MODBUS_RTU_MAX_FRAME_SIZE]; /**< Raw accumulated bytes */
    uint16_t            u16Count;                              /**< Bytes received so far  */
    ModbusRtu_RxState_t eState;                               /**< Receiver FSM state     */
    bool                bOverflow;                            /**< Buffer overflow flag   */
} ModbusRtu_RxCtx_t;

/* =========================================================================
 * Public API — Frame Builder
 * ========================================================================= */

/**
 * @brief  Build a Read Holding Registers (FC03) request frame.
 *
 * @param[out] pxReq         Destination frame structure.
 * @param[in]  u8SlaveAddr   Target slave address (1–247).
 * @param[in]  u16RegAddr    Starting register address (0-based).
 * @param[in]  u16RegCount   Number of registers to read (1–125).
 *
 * @return  true on success; false if any parameter is out of range.
 */
bool ModbusRtu_BuildReadHoldingRegisters(ModbusRtu_Request_t *pxReq,
                                          uint8_t              u8SlaveAddr,
                                          uint16_t             u16RegAddr,
                                          uint16_t             u16RegCount);

/**
 * @brief  Build a Write Single Register (FC06) request frame.
 *
 * @param[out] pxReq         Destination frame structure.
 * @param[in]  u8SlaveAddr   Target slave address (1–247).
 * @param[in]  u16RegAddr    Target register address (0-based).
 * @param[in]  u16Value      16-bit register value to write.
 *
 * @return  true on success; false if any parameter is out of range.
 */
bool ModbusRtu_BuildWriteSingleRegister(ModbusRtu_Request_t *pxReq,
                                         uint8_t              u8SlaveAddr,
                                         uint16_t             u16RegAddr,
                                         uint16_t             u16Value);

/**
 * @brief  Build a Write Multiple Registers (FC16) request frame.
 *
 * @param[out] pxReq         Destination frame structure.
 * @param[in]  u8SlaveAddr   Target slave address (1–247).
 * @param[in]  u16RegAddr    Starting register address (0-based).
 * @param[in]  pu16Values    Array of register values (big-endian in frame).
 * @param[in]  u8RegCount    Number of registers (1–123).
 *
 * @return  true on success; false if any parameter is out of range or
 *          resulting frame would exceed MODBUS_RTU_MAX_FRAME_SIZE.
 */
bool ModbusRtu_BuildWriteMultipleRegisters(ModbusRtu_Request_t *pxReq,
                                            uint8_t              u8SlaveAddr,
                                            uint16_t             u16RegAddr,
                                            const uint16_t      *pu16Values,
                                            uint8_t              u8RegCount);

/* =========================================================================
 * Public API — RX Parser
 * ========================================================================= */

/**
 * @brief  Reset the RX context for a channel (call on init or after error).
 *
 * @param[in,out]  pxCtx   RX context to reset.
 */
void ModbusRtu_RxCtx_Reset(ModbusRtu_RxCtx_t *pxCtx);

/**
 * @brief  Feed one received byte into the RX accumulator.
 *
 * @details Called from ISR context via the UartPort byte-received callback.
 *          Performs overflow detection only; full validation happens in
 *          ModbusRtu_ParseResponse() once the frame is complete.
 *
 * @param[in,out]  pxCtx   Per-channel RX context.
 * @param[in]      u8Byte  Received byte to accumulate.
 */
void ModbusRtu_RxFeedByte(ModbusRtu_RxCtx_t *pxCtx, uint8_t u8Byte);

/**
 * @brief  Signal end-of-frame (T3.5 silence) to the RX context.
 *
 * @details Called from timer-ISR context.  Transitions the FSM to
 *          MODBUS_RTU_RX_FRAME_READY so the master task can process it.
 *
 * @param[in,out]  pxCtx  Per-channel RX context.
 */
void ModbusRtu_RxFrameEnd(ModbusRtu_RxCtx_t *pxCtx);

/**
 * @brief  Parse and validate a completed frame from the RX context.
 *
 * @details Performs: CRC validation, slave address check, function code
 *          validation, exception detection, and data extraction.
 *          Should be called from task context (not ISR) after the FSM
 *          reaches MODBUS_RTU_RX_FRAME_READY.
 *
 * @param[in]  pxCtx         Completed RX context.
 * @param[in]  u8ExpectedAddr Slave address expected for this transaction.
 * @param[in]  eExpectedFC   Function code expected for this transaction.
 * @param[out] pxResp        Populated on success.
 *
 * @return  true  Frame is valid and pxResp is populated.
 *          false Frame failed validation (CRC, address, FC, length, overflow).
 */
bool ModbusRtu_ParseResponse(const ModbusRtu_RxCtx_t *pxCtx,
                              uint8_t                  u8ExpectedAddr,
                              ModbusFunctionCode_t     eExpectedFC,
                              ModbusRtu_Response_t    *pxResp);

/**
 * @brief  Determine the expected response length in bytes for a given request.
 *
 * @details Used to validate that the received byte count matches the protocol
 *          specification before parsing.
 *
 * @param[in]  eFuncCode   Function code of the sent request.
 * @param[in]  u8RegCount  Number of registers / coils in the request.
 *
 * @return  Expected total response frame length (including CRC).
 *          Returns 0 for unsupported or unknown function codes.
 */
uint16_t ModbusRtu_GetExpectedResponseLen(ModbusFunctionCode_t eFuncCode,
                                           uint8_t              u8RegCount);

#endif /* MODBUS_RTU_H */

/*******************************************************************************
 * End of File
 *******************************************************************************/
