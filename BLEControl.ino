#include <ArduinoBLE.h>

// extern globals
// These BLE objects are defined elsewhere and shared across files
extern BLEService uartService;                // Custom BLE UART-like service
extern BLEStringCharacteristic txChar;        // TX characteristic (device → phone)
extern BLEStringCharacteristic rxChar;        // RX characteristic (phone → device)

void ble_init() {
  // Start BLE hardware and stack
  if (!BLE.begin()) {
    Serial.println("BLE failed to start!");
    while (1);  // Halt if BLE cannot initialize
  }

  // Set advertised device name
  BLE.setLocalName("Nano33BLE-Terminal");

  // Advertise the UART-like service
  BLE.setAdvertisedService(uartService);

  // Add TX and RX characteristics to the service
  uartService.addCharacteristic(txChar);
  uartService.addCharacteristic(rxChar);

  // Register the service with the BLE stack
  BLE.addService(uartService);

  // Begin advertising so phones can connect
  BLE.advertise();
  Serial.println("BLE ready. Waiting for Serial Bluetooth Terminal app...");
}

void ble_send(const String &msg) {
  // Check if a BLE central device is connected
  BLEDevice central = BLE.central();
  if (central && central.connected()) {
    // Write outgoing message to the TX characteristic
    txChar.writeValue(msg);
  }
}
