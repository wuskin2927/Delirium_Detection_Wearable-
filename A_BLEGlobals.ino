#include <ArduinoBLE.h>                 // BLE communication library
#include <Arduino_BMI270_BMM150.h>      // IMU sensor library (accelerometer/gyro/magnetometer)


// BLE UART service + characteristics

BLEService uartService("6E400001-B5A3-F393-E0A9-E50E24DCCA9E");  
// Defines a custom BLE UART service using the Nordic UART UUID


BLEStringCharacteristic txChar(
  "6E400003-B5A3-F393-E0A9-E50E24DCCA9E",  // UUID for TX characteristic (Arduino → phone)
  BLENotify,                               // Enables notifications to send data to central device
  64                                       // Max string length
);


BLEStringCharacteristic rxChar(
  "6E400002-B5A3-F393-E0A9-E50E24DCCA9E",  // UUID for RX characteristic (phone → Arduino)
  BLEWrite,                                // Allows central device to write data to Arduino
  64                                       // Max string length
);
