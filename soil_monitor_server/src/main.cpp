#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <SPI.h>

#include "Adafruit_seesaw.h"

#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CAPACITANCE_CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define TEMPERATURE_CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a9"

Adafruit_seesaw ss;

BLEServer* pServer = NULL;
BLECharacteristic* pCapCharacteristic = NULL;
BLECharacteristic* pTempCharacteristic = NULL;

bool deviceConnected = false;
bool oldDeviceConnected = false;
uint32_t value = 0;

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
    }
};

void setup() {
  // Start Serial Monitor
  Serial.begin(115200);
  Serial.println("Starting BLE Soil Server!");

  // Initialize adafruit seesaw
  if (!ss.begin(0x36)) {
    Serial.println("ERROR! seesaw not found");
    while(1);
  } else {
    Serial.print("seesaw started! version: ");
    Serial.println(ss.getVersion(), HEX);
  }

  // Create the BLE Device
  BLEDevice::init("Soil Sensor 1");

  // Create the BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  
  // Create the BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Create Capacitance Characteristic
  pCapCharacteristic = pService->createCharacteristic(
                     CAPACITANCE_CHARACTERISTIC_UUID,
                     BLECharacteristic::PROPERTY_READ |
                     BLECharacteristic::PROPERTY_WRITE
                   );

  // Create Temperature Characteristic
  pTempCharacteristic = pService->createCharacteristic(
                     TEMPERATURE_CHARACTERISTIC_UUID,
                     BLECharacteristic::PROPERTY_READ |
                     BLECharacteristic::PROPERTY_WRITE
                   );

  // Start the service
  pService->start();

  //  Start Advertising
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);  // functions that help with iPhone connections issue
  pAdvertising->setMinPreferred(0x12);
  BLEDevice::startAdvertising();
  Serial.println("Characteristic defined! Now you can read it in your phone!");
}

void loop() {
  // Read Soil Capacitance and Temperature
  float tempC = ss.getTemp();
  float tempF = (tempC * 9/5) + 32;
  uint16_t capread = ss.touchRead(0);

  // Set characteristic data every 1 second
  if (deviceConnected) {
      pCapCharacteristic->setValue(capread);
      Serial.print("Capacitive: "); Serial.println(capread);
      pTempCharacteristic->setValue(tempF);
      Serial.print("Temperature: "); Serial.print(tempF); Serial.println("*F");
      delay(1000);
  }
  // disconnecting
  if (!deviceConnected && oldDeviceConnected) {
      delay(500); // give the bluetooth stack the chance to get things ready
      pServer->startAdvertising(); // restart advertising
      Serial.println("start advertising");
      oldDeviceConnected = deviceConnected;
  }
  // connecting
  if (deviceConnected && !oldDeviceConnected) {
      // do stuff here on connecting
      oldDeviceConnected = deviceConnected;
  }
  
  delay(100);
}