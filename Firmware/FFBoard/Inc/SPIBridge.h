/**
 * @file SPIBridge.h
 * @brief SPI Bridge for FFBoard wireless communication
 * 
 * This class implements the SPI master communication with the nRF52832
 * wireless bridge module. It handles packet framing, CRC validation,
 * and provides interfaces for HID and CDC data transfer.
 * 
 * Hardware Configuration:
 *   STM32 F407          nRF52832
 *   -----------         ----------
 *   PA5 (SCK)    -----> P0.25 (SCK)
 *   PA6 (MISO)   <----- P0.24 (MISO)
 *   PA7 (MOSI)   -----> P0.23 (MOSI)
 *   PA4 (CS)     -----> P0.22 (CS)
 *   PA3 (IRQ)    <----- P0.21 (READY/IRQ)
 */

#ifndef SPI_BRIDGE_H_
#define SPI_BRIDGE_H_

#include "cppmain.h"
#include "SPI.h"
#include "GPIOPin.h"
#include "thread.hpp"
#include "semaphore.hpp"
#include "queue.hpp"
#include "ffb_defs.h"
#include "ExtiHandler.h"
#include "HidCommandInterface.h"

extern "C" {
#include "spi_protocol.h"
}

#include <functional>
#include <memory>

/*******************************************************************************
 * Configuration
 ******************************************************************************/

// SPI Bridge configuration
#define SPI_BRIDGE_SPEED            4000000     // 4 MHz SPI clock
#define SPI_BRIDGE_TIMEOUT_MS       100         // Transaction timeout
#define SPI_BRIDGE_POLL_INTERVAL_MS 1           // Polling interval when no IRQ
#define SPI_BRIDGE_MAX_RETRIES      3           // Max retransmission attempts

// Thread configuration
#define SPI_BRIDGE_STACK_SIZE       512         // Thread stack size in words
#define SPI_BRIDGE_PRIORITY         (configMAX_PRIORITIES - 2)

/*******************************************************************************
 * SPIBridge Class
 ******************************************************************************/

class SPIBridge : public cpp_freertos::Thread, public SPIDevice, public ExtiHandler {
public:
    /**
     * @brief Construct SPI Bridge
     * @param port Reference to SPI port
     * @param csPin Chip select pin
     * @param irqPin IRQ input pin from nRF52832
     */
    SPIBridge(SPIPort& port, OutputPin csPin, InputPin irqPin);
    
    virtual ~SPIBridge();
    
    /**
     * @brief Initialize the bridge
     * @return true if initialization successful
     */
    bool init();
    
    /***************************************************************************
     * HID Interface
     **************************************************************************/
    
    /**
     * @brief Send HID input report (gamepad state)
     * @param report Pointer to HID report structure
     * @return true if queued successfully
     */
    bool sendHIDReport(const reportHID_t* report);
    
    /**
     * @brief Send HID command response
     * @param cmd Pointer to HID command data
     * @return true if queued successfully
     */
    bool sendHIDCommand(const HID_CMD_Data_t* cmd);
    
    /**
     * @brief Register callback for received HID output reports (FFB)
     * @param callback Function to call when FFB data received
     */
    void registerHIDOutputCallback(std::function<void(const uint8_t*, uint16_t)> callback);
    
    /**
     * @brief Register callback for received HID commands
     * @param callback Function to call when HID command received
     */
    void registerHIDCommandCallback(std::function<void(const HID_CMD_Data_t*)> callback);
    
    /***************************************************************************
     * CDC Interface
     **************************************************************************/
    
    /**
     * @brief Send CDC data
     * @param data Pointer to data buffer
     * @param length Number of bytes to send
     * @return true if queued successfully
     */
    bool sendCDCData(const char* data, uint16_t length);
    
    /**
     * @brief Register callback for received CDC data
     * @param callback Function to call when CDC data received
     */
    void registerCDCCallback(std::function<void(const char*, uint16_t)> callback);
    
    /***************************************************************************
     * Control
     **************************************************************************/
    
    /**
     * @brief Check if wireless bridge is connected
     * @return true if connected and responding
     */
    bool isConnected() const { return connected; }
    
    /**
     * @brief Get connection statistics
     */
    struct Stats {
        uint32_t txPackets;
        uint32_t rxPackets;
        uint32_t txErrors;
        uint32_t rxErrors;
        uint32_t crcErrors;
        uint32_t timeouts;
        uint32_t lastPingMs;
    };
    
    Stats getStats() const { return stats; }
    
    /**
     * @brief Send ping to check connection
     * @return true if pong received within timeout
     */
    bool ping();
    
    /**
     * @brief Reset the bridge
     */
    void reset();
    
protected:
    // Thread main loop
    void Run() override;
    
    // EXTI handler for IRQ pin
    void exti(uint16_t GPIO_Pin) override;
    
private:
    /***************************************************************************
     * Internal Methods
     **************************************************************************/
    
    /**
     * @brief Send a packet to nRF52832
     * @param packet Packet to send
     * @return SPI status
     */
    spi_status_t sendPacket(spi_packet_t* packet);
    
    /**
     * @brief Receive a packet from nRF52832
     * @param packet Buffer for received packet
     * @return SPI status
     */
    spi_status_t receivePacket(spi_packet_t* packet);
    
    /**
     * @brief Process received packet
     * @param packet Received packet
     */
    void processPacket(const spi_packet_t* packet);
    
    /**
     * @brief Build and queue a packet
     * @param type Packet type
     * @param priority Packet priority
     * @param payload Payload data
     * @param length Payload length
     * @param ackRequired Whether ACK is required
     * @return true if queued successfully
     */
    bool queuePacket(spi_packet_type_t type, spi_priority_t priority,
                     const uint8_t* payload, uint8_t length, bool ackRequired);
    
    /**
     * @brief Send control message
     * @param cmd Control command
     * @param params Optional parameters
     * @param paramLen Parameter length
     * @return true if sent successfully
     */
    bool sendControlMessage(control_cmd_t cmd, const uint8_t* params = nullptr, 
                            uint8_t paramLen = 0);
    
    /**
     * @brief Handle control message
     * @param payload Control message payload
     * @param length Payload length
     */
    void handleControlMessage(const uint8_t* payload, uint8_t length);
    
    /***************************************************************************
     * Member Variables
     **************************************************************************/
    
    // Hardware
    InputPin irqPin;
    
    // Queues
    cpp_freertos::Queue txQueue;
    cpp_freertos::Queue rxQueue;
    
    // Callbacks
    std::function<void(const uint8_t*, uint16_t)> hidOutputCallback;
    std::function<void(const HID_CMD_Data_t*)> hidCommandCallback;
    std::function<void(const char*, uint16_t)> cdcCallback;
    
    // State
    volatile bool connected;
    volatile bool irqPending;
    uint16_t sequenceNumber;
    
    // Synchronization
    cpp_freertos::BinarySemaphore irqSemaphore;
    cpp_freertos::BinarySemaphore pongSemaphore;
    
    // Statistics
    Stats stats;
    
    // Buffers for SPI transactions
    uint8_t txBuffer[SPI_BRIDGE_MAX_PACKET_SIZE];
    uint8_t rxBuffer[SPI_BRIDGE_MAX_PACKET_SIZE];
};

#endif /* SPI_BRIDGE_H_ */
