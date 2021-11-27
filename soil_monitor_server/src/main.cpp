#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <SPI.h>

#include "Adafruit_seesaw.h"

#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CAPACITANCE_CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define TEMPERATURE_CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a9"

#define DEVICE_NAME "Soil Sensor 1"

Adafruit_seesaw ss;

BLEServer* pServer = NULL;
BLECharacteristic* pCapCharacteristic = NULL;
BLECharacteristic* pTempCharacteristic = NULL;
BLEAdvertising *pAdvertising;

static bool boRestartAdvertising = false;;
bool deviceConnected = false;
bool oldDeviceConnected = false;
uint32_t value = 0;

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
    };

    void onDisconnect(BLEServer* pServer) {
      Serial.println("disconnect callback");
      deviceConnected = false;
      boRestartAdvertising = true;
    }
};

//manufacturer code (0x02E5 for Espressif)
int man_code = 0x02E5;

//function takes String and adds manufacturer code at the beginning 
void setManData(String c, int c_size, BLEAdvertisementData &adv, int m_code) {
  
  String s;
  char b2 = (char)(m_code >> 8);
  m_code <<= 8;
  char b1 = (char)(m_code >> 8);
  s.concat(b1);
  s.concat(b2);
  s.concat(c);
  adv.setManufacturerData(s.c_str());
  
}

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
  BLEDevice::init( DEVICE_NAME );

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
  // pAdvertising = BLEDevice::getAdvertising();
  pAdvertising = pServer->getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);  // functions that help with iPhone connections issue
  pAdvertising->setMinPreferred(0x12);

  //advert.setName("Soil sensor reading");


  // BLEDevice::startAdvertising();
  pAdvertising->start();
  Serial.println("Characteristic defined! Now you can read it in your phone!");
}

//int i;
void loop() {
  // Read Soil Capacitance and Temperature
  float tempC = ss.getTemp();
  float tempF = (tempC * 9/5) + 32;
  uint16_t capread = ss.touchRead(0);


  //i ++;
  //String a = String(i);
  BLEAdvertisementData advertisingData;
  String capString = String(capread);
  Serial.print("Capacitive: "); Serial.println(capread);

  setManData( capString, capString.length(), advertisingData, man_code);

  Serial.print("adv data is: " ); Serial.println(String(advertisingData.getPayload().data()));

  pAdvertising->stop();
  pAdvertising->setAdvertisementData(advertisingData);
  pAdvertising->start();

  delay(2000);
  // Set characteristic data every 1 second
  // if (deviceConnected) {
  //     pCapCharacteristic->setValue(capread);
  //     pTempCharacteristic->setValue(tempF);
  //     Serial.print("Temperature: "); Serial.print(tempF); Serial.println("*F");
  //     delay(1000);
  // }
  // // disconnecting
  // if ( boRestartAdvertising == true ) {
  //     delay(500); // give the bluetooth stack the chance to get things ready
  //     pServer->startAdvertising(); // restart advertising
  //     Serial.println("restart advertising");
  //     oldDeviceConnected = deviceConnected;
  //     boRestartAdvertising = false;
  // }
  // connecting
  
  //delay(100);
}