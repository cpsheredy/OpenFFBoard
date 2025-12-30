#ifndef SPI_BRIDGE_DEMO_H_
#define SPI_BRIDGE_DEMO_H_

#include "ffb_defs.h"
#include <stdint.h>

bool initSPIBridge();
void sendHIDReportWireless(const reportHID_t* report);
void sendCDCWireless(const char* str, uint16_t len);
bool isWirelessConnected();
void printBridgeStats();
bool testSPIBridge();

#endif