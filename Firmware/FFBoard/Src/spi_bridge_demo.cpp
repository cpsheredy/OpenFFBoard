/**
 * @file spi_bridge_demo.cpp
 * @brief Demo/test code for STM32 SPI Bridge
 * 
 * This file demonstrates how to integrate the SPIBridge class with the
 * FFBoard firmware. It can be used as a starting point or for testing
 * the SPI communication layer.
 * 
 * To integrate with FFBoard:
 * 1. Include this code in your build
 * 2. Configure SPI1 in CubeMX (PA5=SCK, PA6=MISO, PA7=MOSI)
 * 3. Configure PA4 as GPIO output (CS)
 * 4. Configure PA3 as EXTI falling edge (IRQ)
 * 5. Create SPIBridge instance in cppmain.cpp
 * 6. Call bridge methods from FFBHIDMain
 */

#include "SPIBridge.h"
#include "cppmain.h"
#include "HidCommandInterface.h"
#include "spi_bridge_demo.h"

// External SPI handle (defined in main.c by CubeMX)
extern SPI_HandleTypeDef hspi2;

/*******************************************************************************
 * Global Variables
 ******************************************************************************/

// SPI Port wrapper
static std::vector<OutputPin> spi2CsPins = {
    OutputPin(*GPIOB, GPIO_PIN_12)   // PB12 (SPI2)
};
static SPIPort* spi2Port = nullptr;

// SPI Bridge instance
static SPIBridge* spiBridge = nullptr;

/*******************************************************************************
 * Callback Functions
 ******************************************************************************/

/**
 * @brief Callback when FFB output report received from PC via wireless
 */
static void onHIDOutputReceived(const uint8_t* data, uint16_t length) {
    // Forward to FFB handler
    // This replaces the direct USB HID output callback
    
    // Example: Parse FFB effect data
    if (length >= 2) {
        uint8_t reportId = data[0];
        (void)reportId;  // TODO: Process FFB report based on reportId
        // Process FFB report based on reportId
        // See HidFFB.cpp for report handling
    }
}

/**
 * @brief Callback when HID command received from PC via wireless
 */
static void onHIDCommandReceived(const HID_CMD_Data_t* cmd) {
    // Forward to command handler
    // This replaces the direct USB HID feature report callback
    
    // Example: Forward to HID_CommandInterface
    // HID_CommandInterface::globalInterface->hidCmdCallback(cmd);
}

/**
 * @brief Callback when CDC data received from PC via wireless
 */
static void onCDCDataReceived(const char* data, uint16_t length) {
    // Forward to CDC command parser
    // This replaces the direct USB CDC callback
    
    // Example: Process as string command
    // std::string cmdStr(data, length);
    // CommandHandler::processCommand(cmdStr);
}

/*******************************************************************************
 * Initialization
 ******************************************************************************/

/**
 * @brief Initialize SPI Bridge for wireless communication
 * 
 * Call this from cppmain() after HAL initialization but before
 * the main loop starts.
 * 
 * @return true if initialization successful
 */
bool initSPIBridge() {
    // Get APB clock for SPI2 (APB1 = 42MHz on F407)
    uint32_t apb1Clock = HAL_RCC_GetPCLK1Freq();
    
    // Create SPI port wrapper
    spi2Port = new SPIPort(hspi2, spi2CsPins, apb1Clock, true);
    
    // Create CS and IRQ pins
    OutputPin csPin(*GPIOB, GPIO_PIN_12); // PB12
    InputPin irqPin(*GPIOD, GPIO_PIN_9);  // PD9
    
    // Create bridge instance
    spiBridge = new SPIBridge(*spi2Port, csPin, irqPin);
    
    // Register callbacks
    spiBridge->registerHIDOutputCallback(onHIDOutputReceived);
    spiBridge->registerHIDCommandCallback(onHIDCommandReceived);
    spiBridge->registerCDCCallback(onCDCDataReceived);
    
    // Initialize bridge (starts thread, sends READY, pings nRF)
    if (!spiBridge->init()) {
        // Failed to connect to nRF
        return false;
    }
    
    return true;
}

/*******************************************************************************
 * Usage Examples
 ******************************************************************************/

/**
 * @brief Send HID report via wireless
 * 
 * Call this from FFBHIDMain::send_report() when in wireless mode
 */
void sendHIDReportWireless(const reportHID_t* report) {
    if (spiBridge != nullptr && spiBridge->isConnected()) {
        spiBridge->sendHIDReport(report);
    }
}

/**
 * @brief Send CDC string via wireless
 * 
 * Call this from CDCcomm::cdcSend() when in wireless mode
 */
void sendCDCWireless(const char* str, uint16_t len) {
    if (spiBridge != nullptr && spiBridge->isConnected()) {
        spiBridge->sendCDCData(str, len);
    }
}

/**
 * @brief Check if wireless bridge is connected
 */
bool isWirelessConnected() {
    return (spiBridge != nullptr && spiBridge->isConnected());
}

/**
 * @brief Get bridge statistics for debugging
 */
void printBridgeStats() {
    if (spiBridge == nullptr) {
        return;
    }
    
    SPIBridge::Stats stats = spiBridge->getStats();
    
    // Print stats via CDC or debug UART
    // printf("TX: %lu, RX: %lu, Errors: %lu, CRC: %lu\n",
    //        stats.txPackets, stats.rxPackets, 
    //        stats.txErrors + stats.rxErrors, stats.crcErrors);
}

/*******************************************************************************
 * Simple Test Function
 ******************************************************************************/

/**
 * @brief Simple SPI communication test
 * 
 * Tests basic ping/pong with nRF52832.
 * Call from a test mode or debug command.
 */
bool testSPIBridge() {
    if (spiBridge == nullptr) {
        return false;
    }
    
    // Test 1: Ping
    if (!spiBridge->ping()) {
        return false;
    }
    
    // Test 2: Send dummy HID report
    reportHID_t testReport;
    testReport.id = 1;
    testReport.buttons = 0x0000000000000001;  // Button 0 pressed
    testReport.X = 1000;
    testReport.Y = -1000;
    testReport.Z = 0;
    testReport.RX = 0;
    testReport.RY = 0;
    testReport.RZ = 0;
    testReport.Dial = 0;
    testReport.Slider = 0;
    
    if (!spiBridge->sendHIDReport(&testReport)) {
        return false;
    }
    
    // Test 3: Send CDC message
    const char* testMsg = "Hello from STM32!\n";
    if (!spiBridge->sendCDCData(testMsg, strlen(testMsg))) {
        return false;
    }
    
    return true;
}

/*******************************************************************************
 * CubeMX Configuration Notes
 ******************************************************************************/

/*
 * Required CubeMX Configuration for SPI Bridge:
 * 
 * SPI1 Configuration:
 * - Mode: Full-Duplex Master
 * - Hardware NSS: Disable (we use software CS)
 * - Prescaler: Calculate for ~4-8 MHz (APB2/16 or APB2/32)
 * - CPOL: Low
 * - CPHA: 1 Edge
 * - Data Size: 8 bits
 * - First Bit: MSB
 * - DMA: Optional (TX and RX streams)
 * 
 * GPIO Configuration, SPI1 (not in use):
 * - PA5: SPI1_SCK (Alternate Function)
 * - PA6: SPI1_MISO (Alternate Function)
 * - PA7: SPI1_MOSI (Alternate Function)
 * - PA4: GPIO_Output (CS pin, initially HIGH)
 * - PA3: GPIO_EXTI3 (IRQ from nRF, falling edge)

 * GPIO Configuration, SPI2:
 * - PB13: SPI2_SCK (Alternate Function)
 * - PB14: SPI2_MISO (Alternate Function)
 * - PB15: SPI2_MOSI (Alternate Function)
 * - PB12: GPIO_Output (CS pin, initially HIGH)
 * - PD9: GPIO_EXTI3 (IRQ from nRF, falling edge)

 * NVIC Configuration:
 * - EXTI3 interrupt enabled
 * - SPI1 interrupt enabled (if using IT mode)
 * - DMA streams enabled (if using DMA)
 * 
 * Clock Configuration:
 * - Ensure APB2 clock is known for SPI prescaler calculation
 */
