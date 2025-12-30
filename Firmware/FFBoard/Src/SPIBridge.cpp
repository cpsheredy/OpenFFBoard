/**
 * @file SPIBridge.cpp
 * @brief SPI Bridge implementation for FFBoard wireless communication
 */

#include "SPIBridge.h"
#include <cstring>

/*******************************************************************************
 * Constructor / Destructor
 ******************************************************************************/

SPIBridge::SPIBridge(SPIPort& port, OutputPin csPin, InputPin irqPin)
    : Thread("SPIBridge", SPI_BRIDGE_STACK_SIZE, SPI_BRIDGE_PRIORITY),
      SPIDevice(port, csPin),
      irqPin(irqPin),
      txQueue(SPI_BRIDGE_TX_QUEUE_SIZE, sizeof(spi_packet_t)),
      rxQueue(SPI_BRIDGE_RX_QUEUE_SIZE, sizeof(spi_packet_t)),
      irqSemaphore(false),
      pongSemaphore(false)
{
    connected = false;
    irqPending = false;
    sequenceNumber = 0;
    
    memset(&stats, 0, sizeof(stats));
    memset(txBuffer, 0, sizeof(txBuffer));
    memset(rxBuffer, 0, sizeof(rxBuffer));
    
    // Configure SPI for Mode 0, 4MHz
    SPIConfig config(csPin);
    config.peripheral.CLKPolarity = SPI_POLARITY_LOW;   // CPOL = 0
    config.peripheral.CLKPhase = SPI_PHASE_1EDGE;       // CPHA = 0
    
    // Calculate prescaler for target speed
    auto [prescaler, actualSpeed] = spiPort.getClosestPrescaler(SPI_BRIDGE_SPEED);
    config.peripheral.BaudRatePrescaler = prescaler;
    
    setSpiConfig(config);
}

SPIBridge::~SPIBridge() {
    // Thread cleanup handled by base class
}

/*******************************************************************************
 * Initialization
 ******************************************************************************/

bool SPIBridge::init() {
    // Register EXTI handler for IRQ pin
    // Note: This depends on your EXTI configuration in CubeMX
    // The IRQ pin should be configured as falling edge interrupt
    
    // Start the thread
    if (!Start()) {
        return false;
    }
    
    // Wait a bit for nRF to be ready
    Delay(pdMS_TO_TICKS(100));
    
    // Send ready notification
    sendControlMessage(CTRL_READY);
    
    //TROUBLESHOOT: Openloop
    //ping();  // Try ping but don't fail if no response
    //return true;  // Return true anyway for probing
    
    // Try to ping
    return ping();
}

/*******************************************************************************
 * HID Interface
 ******************************************************************************/

bool SPIBridge::sendHIDReport(const reportHID_t* report) {
    if (!connected || report == nullptr) {
        return false;
    }
    
    // Build HID input payload
    spi_hid_input_payload_t payload;
    payload.report_id = report->id;
    payload.buttons = report->buttons;
    payload.axes[0] = report->X;
    payload.axes[1] = report->Y;
    payload.axes[2] = report->Z;
    payload.axes[3] = report->RX;
    payload.axes[4] = report->RY;
    payload.axes[5] = report->RZ;
    payload.axes[6] = report->Dial;
    payload.axes[7] = report->Slider;
    
    return queuePacket(PKT_TYPE_HID_INPUT, PKT_PRIORITY_NORMAL,
                       (const uint8_t*)&payload, SPI_HID_INPUT_PAYLOAD_SIZE, false);
}

bool SPIBridge::sendHIDCommand(const HID_CMD_Data_t* cmd) {
    if (!connected || cmd == nullptr) {
        return false;
    }
    
    // Build HID command payload
    spi_hid_cmd_payload_t payload;
    payload.report_id = cmd->reportId;
    payload.type = static_cast<uint8_t>(cmd->type);
    payload.clsid = cmd->clsid;
    payload.instance = cmd->instance;
    payload.cmd = cmd->cmd;
    payload.data = cmd->data;
    payload.addr = cmd->addr;
    
    return queuePacket(PKT_TYPE_HID_FEAT_RESP, PKT_PRIORITY_NORMAL,
                       (const uint8_t*)&payload, SPI_HID_CMD_PAYLOAD_SIZE, true);
}

void SPIBridge::registerHIDOutputCallback(std::function<void(const uint8_t*, uint16_t)> callback) {
    hidOutputCallback = callback;
}

void SPIBridge::registerHIDCommandCallback(std::function<void(const HID_CMD_Data_t*)> callback) {
    hidCommandCallback = callback;
}

/*******************************************************************************
 * CDC Interface
 ******************************************************************************/

bool SPIBridge::sendCDCData(const char* data, uint16_t length) {
    if (!connected || data == nullptr || length == 0) {
        return false;
    }
    
    // Fragment large data if necessary
    uint16_t offset = 0;
    while (offset < length) {
        uint8_t chunkSize = (length - offset > SPI_BRIDGE_MAX_PAYLOAD) ? 
                            SPI_BRIDGE_MAX_PAYLOAD : (length - offset);
        
        bool moreData = (offset + chunkSize < length);
        uint8_t flags = moreData ? PKT_FLAG_FRAGMENTED : 0;
        
        spi_packet_t packet;
        packet.header = spi_build_header(PKT_TYPE_CDC_TX, PKT_PRIORITY_LOW, flags);
        packet.length = chunkSize;
        packet.sequence = sequenceNumber++;
        memcpy(packet.payload, data + offset, chunkSize);
        spi_append_crc(&packet);
        
        if (!txQueue.Enqueue(&packet, pdMS_TO_TICKS(10))) {
            return false;
        }
        
        offset += chunkSize;
    }
    
    return true;
}

void SPIBridge::registerCDCCallback(std::function<void(const char*, uint16_t)> callback) {
    cdcCallback = callback;
}

/*******************************************************************************
 * Control Methods
 ******************************************************************************/

bool SPIBridge::ping() {
    // Clear any pending pong
    pongSemaphore.Take(0);
    
    sendControlMessage(CTRL_PING);
    
    // Wait for pong with timeout
    if (pongSemaphore.Take(pdMS_TO_TICKS(SPI_BRIDGE_TIMEOUT_MS))) {
        connected = true;
        return true;
    }
    
    stats.timeouts++;
    connected = false;
    return false;
}

void SPIBridge::reset() {
    sendControlMessage(CTRL_RESET);
    connected = false;
    sequenceNumber = 0;
    
    // Clear queues
    txQueue.Flush();
    rxQueue.Flush();
    
    Delay(pdMS_TO_TICKS(100));
    ping();
}

/*******************************************************************************
 * Thread Main Loop
 ******************************************************************************/

void SPIBridge::Run() {
    spi_packet_t packet;
    
    while (true) {
        // Check for IRQ from nRF (data available)
        if (irqPending || irqSemaphore.Take(0)) {
            irqPending = false;
            
            // Receive data from nRF
            spi_status_t status = receivePacket(&packet);
            if (status == SPI_STATUS_OK) {
                stats.rxPackets++;
                processPacket(&packet);
            } else if (status != SPI_STATUS_BUSY) {
                stats.rxErrors++;
            }
        }
        
        // Send queued packets
        if (txQueue.Dequeue(&packet, 0)) {
            spi_status_t status = sendPacket(&packet);
            if (status == SPI_STATUS_OK) {
                stats.txPackets++;
            } else {
                stats.txErrors++;
                
                // Retry logic
                if (spi_ack_required(packet.header)) {
                    // Re-queue for retry (with limit)
                    // For simplicity, just log error here
                }
            }
        }
        
        // Periodic ping to check connection (every 1 second)
        static uint32_t lastPingTick = 0;
        uint32_t now = HAL_GetTick();
        if (connected && (now - lastPingTick > 1000)) {
            if (!ping()) {
                // Connection lost
                connected = false;
            }
            lastPingTick = now;
        }
        
        // Small delay to prevent busy loop
        Delay(pdMS_TO_TICKS(SPI_BRIDGE_POLL_INTERVAL_MS));
    }
}


/*******************************************************************************
 * EXTI Handler
 ******************************************************************************/

void SPIBridge::exti(uint16_t GPIO_Pin) {
    if (GPIO_Pin == irqPin.getPin()) {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        irqSemaphore.GiveFromISR(&xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

/*******************************************************************************
 * Internal Methods
 ******************************************************************************/

spi_status_t SPIBridge::sendPacket(spi_packet_t* packet) {
    if (packet == nullptr) {
        return SPI_STATUS_ERROR;
    }
    
    uint16_t txSize = SPI_BRIDGE_HEADER_SIZE + packet->length + SPI_BRIDGE_CRC_SIZE;
    
    // Copy to TX buffer
    memcpy(txBuffer, packet, txSize);
    
    // beginSpiTransfer handles semaphore AND CS
    beginSpiTransfer(&spiPort);
    
    // Transmit packet
    HAL_SPI_Transmit(spiPort.getPortHandle(), txBuffer, txSize, SPI_BRIDGE_TIMEOUT_MS);
    
    // Read status byte
    uint8_t status;
    HAL_SPI_Receive(spiPort.getPortHandle(), &status, 1, SPI_BRIDGE_TIMEOUT_MS);
    
    endSpiTransfer(&spiPort);
    
    return (spi_status_t)status;
}

spi_status_t SPIBridge::receivePacket(spi_packet_t* packet) {
    if (packet == nullptr) {
        return SPI_STATUS_ERROR;
    }
    
    // Clear RX buffer
    memset(rxBuffer, 0xFF, sizeof(rxBuffer));
    
    beginSpiTransfer(&spiPort);
    
    // Send dummy byte to clock out data, receive header first
    uint8_t dummyTx[4] = {0xFF, 0xFF, 0xFF, 0xFF};
    HAL_SPI_TransmitReceive(spiPort.getPortHandle(), dummyTx, rxBuffer, 4, SPI_BRIDGE_TIMEOUT_MS);
    
    // Parse header to get length
    spi_packet_t* rxPacket = (spi_packet_t*)rxBuffer;
    
    if (rxPacket->header == 0xFF || rxPacket->length > SPI_BRIDGE_MAX_PAYLOAD) {
        // No data or invalid packet
        endSpiTransfer(&spiPort);
        return SPI_STATUS_BUSY;
    }
    
    // Receive rest of packet (payload + CRC)
    if (rxPacket->length > 0) {
        uint8_t remaining = rxPacket->length + SPI_BRIDGE_CRC_SIZE;
        HAL_SPI_Receive(spiPort.getPortHandle(), 
                        rxBuffer + SPI_BRIDGE_HEADER_SIZE, 
                        remaining, 
                        SPI_BRIDGE_TIMEOUT_MS);
    }
    
    endSpiTransfer(&spiPort);
    
    // Validate CRC
    if (!spi_validate_packet(rxPacket)) {
        stats.crcErrors++;
        return SPI_STATUS_CRC_ERROR;
    }
    
    // Copy to output
    memcpy(packet, rxPacket, SPI_BRIDGE_HEADER_SIZE + rxPacket->length + SPI_BRIDGE_CRC_SIZE);
    
    return SPI_STATUS_OK;
}

void SPIBridge::processPacket(const spi_packet_t* packet) {
    spi_packet_type_t type = spi_get_packet_type(packet->header);
    
    switch (type) {
        case PKT_TYPE_HID_OUTPUT:
            // FFB command from PC
            if (hidOutputCallback) {
                hidOutputCallback(packet->payload, packet->length);
            }
            break;
            
        case PKT_TYPE_HID_FEAT_REQ:
            // HID feature request from PC
            if (hidCommandCallback && packet->length >= SPI_HID_CMD_PAYLOAD_SIZE) {
                // Convert payload to HID_CMD_Data_t
                const spi_hid_cmd_payload_t* cmdPayload = 
                    (const spi_hid_cmd_payload_t*)packet->payload;
                
                HID_CMD_Data_t cmd;
                cmd.reportId = cmdPayload->report_id;
                cmd.type = (HidCmdType)cmdPayload->type;
                cmd.clsid = cmdPayload->clsid;
                cmd.instance = cmdPayload->instance;
                cmd.cmd = cmdPayload->cmd;
                cmd.data = cmdPayload->data;
                cmd.addr = cmdPayload->addr;
                
                hidCommandCallback(&cmd);
            }
            break;
            
        case PKT_TYPE_CDC_RX:
            // CDC data from PC
            if (cdcCallback) {
                cdcCallback((const char*)packet->payload, packet->length);
            }
            break;
            
        case PKT_TYPE_CONTROL:
            handleControlMessage(packet->payload, packet->length);
            break;
            
        default:
            // Unknown packet type
            break;
    }
}

bool SPIBridge::queuePacket(spi_packet_type_t type, spi_priority_t priority,
                            const uint8_t* payload, uint8_t length, bool ackRequired) {
    if (payload == nullptr || length > SPI_BRIDGE_MAX_PAYLOAD) {
        return false;
    }
    
    spi_packet_t packet;
    uint8_t flags = ackRequired ? PKT_FLAG_ACK_REQUIRED : 0;
    
    packet.header = spi_build_header(type, priority, flags);
    packet.length = length;
    packet.sequence = sequenceNumber++;
    memcpy(packet.payload, payload, length);
    spi_append_crc(&packet);
    
    return txQueue.Enqueue(&packet, pdMS_TO_TICKS(10));
}

bool SPIBridge::sendControlMessage(control_cmd_t cmd, const uint8_t* params, uint8_t paramLen) {
    spi_control_payload_t payload;
    payload.cmd = cmd;
    
    if (params != nullptr && paramLen > 0 && paramLen <= sizeof(payload.params)) {
        memcpy(payload.params, params, paramLen);
    }
    
    uint8_t totalLen = 1 + paramLen;  // cmd + params
    
    return queuePacket(PKT_TYPE_CONTROL, PKT_PRIORITY_CRITICAL,
                       (const uint8_t*)&payload, totalLen, true);
}

void SPIBridge::handleControlMessage(const uint8_t* payload, uint8_t length) {
    if (payload == nullptr || length < 1) {
        return;
    }
    
    control_cmd_t cmd = (control_cmd_t)payload[0];
    
    switch (cmd) {
        case CTRL_PONG:
            stats.lastPingMs = HAL_GetTick();
            pongSemaphore.Give();
            break;
            
        case CTRL_PING:
            sendControlMessage(CTRL_PONG);
            break;
            
        case CTRL_ACK:
            // Handle ACK
            break;
            
        case CTRL_NACK:
            // Handle NACK - maybe retransmit
            break;
            
        case CTRL_READY:
            connected = true;
            break;
            
        case CTRL_BUFFER_STATUS:
            // nRF buffer status - can throttle if needed
            break;
            
        default:
            break;
    }
}
