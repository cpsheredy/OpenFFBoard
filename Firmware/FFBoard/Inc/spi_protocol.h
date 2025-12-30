/**
 * @file spi_protocol.h
 * @brief Shared SPI protocol definitions for FFBoard wireless bridge
 * 
 * This file defines the packet structures and protocol used for communication
 * between the STM32 F407 (FFBoard) and nRF52832 (wireless bridge).
 * 
 * Based on the FFBoard Wireless Bridge Architecture specification.
 */

#ifndef SPI_PROTOCOL_H_
#define SPI_PROTOCOL_H_

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************
 * Configuration
 ******************************************************************************/

#define SPI_BRIDGE_MAX_PAYLOAD      244     // Maximum payload size
#define SPI_BRIDGE_HEADER_SIZE      4       // header + length + sequence(2)
#define SPI_BRIDGE_CRC_SIZE         2       // CRC-16
#define SPI_BRIDGE_MAX_PACKET_SIZE  (SPI_BRIDGE_HEADER_SIZE + SPI_BRIDGE_MAX_PAYLOAD + SPI_BRIDGE_CRC_SIZE)

#define SPI_BRIDGE_TX_QUEUE_SIZE    32
#define SPI_BRIDGE_RX_QUEUE_SIZE    32

/*******************************************************************************
 * Packet Types (bits 7-5 of header)
 ******************************************************************************/

typedef enum {
    PKT_TYPE_HID_INPUT      = 0x00,     // 000 = HID Input Report (STM32 → PC)
    PKT_TYPE_HID_OUTPUT     = 0x01,     // 001 = HID Output Report (PC → STM32)
    PKT_TYPE_HID_FEAT_REQ   = 0x02,     // 010 = HID Feature Request (PC → STM32)
    PKT_TYPE_HID_FEAT_RESP  = 0x03,     // 011 = HID Feature Response (STM32 → PC)
    PKT_TYPE_CDC_TX         = 0x04,     // 100 = CDC TX (STM32 → PC)
    PKT_TYPE_CDC_RX         = 0x05,     // 101 = CDC RX (PC → STM32)
    PKT_TYPE_CONTROL        = 0x06,     // 110 = Control/Status
    PKT_TYPE_RESERVED       = 0x07      // 111 = Reserved
} spi_packet_type_t;

/*******************************************************************************
 * Priority Levels (bits 4-3 of header)
 ******************************************************************************/

typedef enum {
    PKT_PRIORITY_LOW        = 0x00,     // CDC bulk data
    PKT_PRIORITY_NORMAL     = 0x01,     // HID input reports
    PKT_PRIORITY_HIGH       = 0x02,     // HID output/FFB
    PKT_PRIORITY_CRITICAL   = 0x03      // Control messages
} spi_priority_t;

/*******************************************************************************
 * Header Flags (bits 2-0)
 ******************************************************************************/

#define PKT_FLAG_ACK_REQUIRED   0x04    // Bit 2: ACK required
#define PKT_FLAG_FRAGMENTED     0x02    // Bit 1: More packets follow
#define PKT_FLAG_RESERVED       0x01    // Bit 0: Reserved

/*******************************************************************************
 * Control Commands
 ******************************************************************************/

typedef enum {
    CTRL_PING               = 0x00,
    CTRL_PONG               = 0x01,
    CTRL_RESET              = 0x02,
    CTRL_SUSPEND            = 0x03,
    CTRL_RESUME             = 0x04,
    CTRL_ACK                = 0x05,
    CTRL_NACK               = 0x06,
    CTRL_BUFFER_STATUS      = 0x07,
    CTRL_VERSION            = 0x08,     // Protocol version exchange
    CTRL_READY              = 0x09      // Device ready notification
} control_cmd_t;

/*******************************************************************************
 * SPI Transaction Status
 ******************************************************************************/

typedef enum {
    SPI_STATUS_OK           = 0x00,     // Transaction successful
    SPI_STATUS_BUSY         = 0xFF,     // Slave busy, retry later
    SPI_STATUS_ERROR        = 0xFE,     // Error occurred
    SPI_STATUS_CRC_ERROR    = 0xFD,     // CRC mismatch
    SPI_STATUS_OVERFLOW     = 0xFC,     // Buffer overflow
    SPI_STATUS_INVALID      = 0xFB      // Invalid packet
} spi_status_t;

/*******************************************************************************
 * Packet Structures
 ******************************************************************************/

/**
 * @brief Build header byte from components
 */
static inline uint8_t spi_build_header(spi_packet_type_t type, spi_priority_t priority, uint8_t flags) {
    return ((type & 0x07) << 5) | ((priority & 0x03) << 3) | (flags & 0x07);
}

/**
 * @brief Extract packet type from header
 */
static inline spi_packet_type_t spi_get_packet_type(uint8_t header) {
    return (spi_packet_type_t)((header >> 5) & 0x07);
}

/**
 * @brief Extract priority from header
 */
static inline spi_priority_t spi_get_priority(uint8_t header) {
    return (spi_priority_t)((header >> 3) & 0x03);
}

/**
 * @brief Check if ACK is required
 */
static inline bool spi_ack_required(uint8_t header) {
    return (header & PKT_FLAG_ACK_REQUIRED) != 0;
}

/**
 * @brief Check if packet is fragmented
 */
static inline bool spi_is_fragmented(uint8_t header) {
    return (header & PKT_FLAG_FRAGMENTED) != 0;
}

/**
 * @brief Generic SPI packet structure
 */
typedef struct __attribute__((packed)) {
    uint8_t  header;                    // Packet type, priority, flags
    uint8_t  length;                    // Payload length (0-244)
    uint16_t sequence;                  // Sequence number
    uint8_t  payload[SPI_BRIDGE_MAX_PAYLOAD + SPI_BRIDGE_CRC_SIZE]; // Payload + CRC
} spi_packet_t;

/**
 * @brief HID Report packet (for gamepad state)
 * Header: 0x08 (type=000, priority=01, ack=0, frag=0)
 */
typedef struct __attribute__((packed)) {
    uint8_t  report_id;                 // 0x01
    uint64_t buttons;                   // Button mask
    int16_t  axes[8];                   // X, Y, Z, RX, RY, RZ, Dial, Slider
} spi_hid_input_payload_t;

#define SPI_HID_INPUT_PAYLOAD_SIZE  (1 + 8 + 16)  // 25 bytes

/**
 * @brief HID Command packet (configuration/query)
 * Based on existing HID_CMD_Data_t structure
 */
typedef struct __attribute__((packed)) {
    uint8_t  report_id;                 // 0xA1
    uint8_t  type;                      // Command type
    uint16_t clsid;                     // Class ID
    uint8_t  instance;                  // Class instance
    uint32_t cmd;                       // Command identifier
    uint64_t data;                      // Data value
    uint64_t addr;                      // Address/secondary value
} spi_hid_cmd_payload_t;

#define SPI_HID_CMD_PAYLOAD_SIZE    25  // 25 bytes

/**
 * @brief Control packet payload
 */
typedef struct __attribute__((packed)) {
    uint8_t  cmd;                       // Control command
    uint8_t  params[8];                 // Command parameters
} spi_control_payload_t;

/*******************************************************************************
 * CRC-16 Calculation (CCITT)
 ******************************************************************************/

/**
 * @brief Calculate CRC-16 CCITT
 * @param data Pointer to data buffer
 * @param length Number of bytes
 * @return CRC-16 value
 */
uint16_t spi_calculate_crc16(const uint8_t* data, uint16_t length);

/**
 * @brief Validate packet CRC
 * @param packet Pointer to packet
 * @return true if CRC is valid
 */
bool spi_validate_packet(const spi_packet_t* packet);

/**
 * @brief Append CRC to packet
 * @param packet Pointer to packet (CRC will be appended after payload)
 */
void spi_append_crc(spi_packet_t* packet);

/*******************************************************************************
 * Protocol Version
 ******************************************************************************/

#define SPI_PROTOCOL_VERSION_MAJOR  1
#define SPI_PROTOCOL_VERSION_MINOR  0

#ifdef __cplusplus
}
#endif

#endif /* SPI_PROTOCOL_H_ */
