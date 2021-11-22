#include <Arduino.h>

/**
 * A BLE client example that is rich in capabilities.
 * There is a lot new capabilities implemented.
 * author unknown
 * updated by chegewaraAAA
 */

#include "BLEDevice.h"
//#include "BLEScan.h"

#include <HardwareSerial.h>

#define FRDM_SERIAL_TX_PIN ( 17 )
#define FRDM_SERIAL_RX_PIN ( 16 )
#define FRDM_SERIAL_MODE ( SERIAL_8N1 )
#define FRDM_SERIAL_BAUD ( 4800 )
#define MAX_NUMBER_OF_SENSORS ( 2 )

#define SENSOR_TIMEOUT_S ( 5 )

//#define PRINT_ALL_DEVICES

typedef enum eSensorValueType
{
  eSENSOR_VALUE_CAPACITANCE,
  eSENSOR_VALUE_TEMPERATURE,

  // new sensor value types above this line
  eSENSOR_VALUE_COUNT,
}eSENSOR_VALUE_TYPE;

typedef struct xSensorReadingPacket
{
  uint8_t u8SensorID;
  eSENSOR_VALUE_TYPE eValueType;
  union
  {
    uint16_t u16Capacitance;
    int8_t s8Temperature;
  };
}xSENSOR_READING_PACKET;


static HardwareSerial FRDMSerialComm(1); // use UART 2, 

// The remote service we wish to connect to.
static BLEUUID serviceUUID("4fafc201-1fb5-459e-8fcc-c5c9c331914b");
// The characteristic of the remote service we are interested in.
static BLEUUID capCharUUID("beb5483e-36e1-4688-b7f5-ea07361b26a8"); // capacitance
static BLEUUID tempCharUUID("beb5483e-36e1-4688-b7f5-ea07361b26a9"); // temperature

static boolean doConnect = false;
static boolean connected = false;
static boolean doScan = false;
static BLEClient* pClient;
static BLERemoteCharacteristic* pRemoteCapCharacteristic;
static BLERemoteCharacteristic* pRemoteTempCharacteristic;
static BLEAdvertisedDevice* myDevice;
static const std::string strDeviceNames[2] = {"Soil Sensor 1", "Soil Sensor 2"};
static uint8_t u8SensorIdx;
static BLERemoteService* pRemoteService;
static BLEScan* pBLEScan;

static void scanCompleteCB(BLEScanResults scanResults) ;

static void notifyCallback(
  BLERemoteCharacteristic* pBLERemoteCharacteristic,
  uint8_t* pData,
  size_t length,
  bool isNotify) {
    Serial.print("Notify callback for characteristic ");
    Serial.print(pBLERemoteCharacteristic->getUUID().toString().c_str());
    Serial.print(" of data length ");
    Serial.println(length);
    Serial.print("data: ");
    Serial.println((char*)pData);
}

class MyClientCallback : public BLEClientCallbacks {
  void onConnect(BLEClient* pclient) {
    connected = true;
    Serial.println(" - Connected to server");
  }

  void onDisconnect(BLEClient* pclient) {
    connected = false;
    Serial.println("onDisconnect");
    delay(1000);
    pBLEScan->start(SENSOR_TIMEOUT_S, &scanCompleteCB, false);
  }
};

bool connectToServer() {
    Serial.print("Forming a connection to ");
    Serial.println(myDevice->getAddress().toString().c_str());
    
    // pClient = BLEDevice::createClient();
    // Serial.println(" - Created client");

    // pClient->setClientCallbacks(new MyClientCallback());

    // Connect to the remote BLE Server.
    pClient->connect(myDevice);  // if you pass BLEAdvertisedDevice instead of address, it will be recognized type of peer device address (public or private)

    // Obtain a reference to the service we are after in the remote BLE server.
    //Serial.println(pClient->getServices());
    
    pRemoteService = pClient->getService(serviceUUID);

    if (pRemoteService == nullptr) {
      Serial.print("Failed to find our service UUID: ");
      Serial.println(serviceUUID.toString().c_str());
      pClient->disconnect();
      return false;
    }

    Serial.println(" - Found our service");

    // Obtain a reference to the characteristic in the service of the remote BLE server.
    pRemoteCapCharacteristic = pRemoteService->getCharacteristic(capCharUUID);

    if (pRemoteCapCharacteristic == nullptr) {
      Serial.print("Failed to find capacitance characteristic UUID: ");
      Serial.println(capCharUUID.toString().c_str());
      pClient->disconnect();
      return false;
    }
    
    // if(pRemoteCapCharacteristic->canNotify())
    // {
    //   pRemoteCapCharacteristic->registerForNotify(notifyCallback);
    // }

    pRemoteTempCharacteristic = pRemoteService->getCharacteristic(tempCharUUID);

    if (pRemoteTempCharacteristic == nullptr) {
      Serial.print("Failed to find temperature characteristic UUID: ");
      Serial.println(tempCharUUID.toString().c_str());
      pClient->disconnect();
      return false;
    }
    
    // if(pRemoteTempCharacteristic->canNotify())
    // {
    //   pRemoteTempCharacteristic->registerForNotify(notifyCallback);
    // }

    Serial.println(" - Found our characteristics");

    return true;
}
/**
 * Scan for BLE servers and find the first one that advertises the service we are looking for.
 */
class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
 /**
   * Called for each advertising BLE server.
   */

  void onResult(BLEAdvertisedDevice advertisedDevice) {
#ifdef PRINT_ALL_DEVICES
    Serial.print("BLE Advertised Device found: ");
    Serial.println(advertisedDevice.toString().c_str());
#endif

    // We have found a device, let's see if it's the device name we're looking for
    if( strDeviceNames[ u8SensorIdx % MAX_NUMBER_OF_SENSORS ] == advertisedDevice.getName() )
    {
    //if (advertisedDevice.haveServiceUUID() && advertisedDevice.isAdvertisingService(serviceUUID)) {

      pBLEScan->stop();
      free(myDevice);
      myDevice = new BLEAdvertisedDevice(advertisedDevice);
      doConnect = true;
      doScan = true;

    } // Found our server
  } // onResult
}; // MyAdvertisedDeviceCallbacks

void scanCompleteCB(BLEScanResults scanResults) 
{
    Serial.print("Timeout scanning for device ");
    Serial.println( ( u8SensorIdx % MAX_NUMBER_OF_SENSORS ), DEC );
    u8SensorIdx++;
    pBLEScan->start(SENSOR_TIMEOUT_S, &scanCompleteCB, false);
}

void setup() {
  Serial.begin( 115200 );
  FRDMSerialComm.begin( FRDM_SERIAL_BAUD, FRDM_SERIAL_MODE, FRDM_SERIAL_RX_PIN, FRDM_SERIAL_TX_PIN );

  Serial.println("Starting Arduino BLE Client application...");
  BLEDevice::init("");

  // Retrieve a Scanner and set the callback we want to use to be informed when we
  // have detected a new device.  Specify that we want active scanning and start the
  // scan to run for 5 seconds.
  pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setInterval(1349);
  pBLEScan->setWindow(449);
  pBLEScan->setActiveScan(true);
  pBLEScan->start(SENSOR_TIMEOUT_S, &scanCompleteCB, false);

  pClient = BLEDevice::createClient();
  pClient->setClientCallbacks(new MyClientCallback());

  u8SensorIdx = 1;
} // End of setup.


// This is the Arduino main loop function.
void loop() {
  uint8_t u8TxBuf[8];

  // If the flag "doConnect" is true then we have scanned for and found the desired
  // BLE Server with which we wish to connect.  Now we connect to it.  Once we are 
  // connected we set the connected flag to be true.
  if (doConnect == true) {
    if (connectToServer()) {
      Serial.println("We are now connected to the BLE Server.");
    } else {
      Serial.println("We have failed to connect to the server; there is nothin more we will do.");
    }
    doConnect = false;
  }

  // If we are connected to a peer BLE Server, update the characteristic each time we are reached
  // with the current time since boot.
  if (connected) {

      // Read the value of the characteristic.
    if(pRemoteCapCharacteristic->canRead()) {
      uint16_t u16CapValue = pRemoteCapCharacteristic->readUInt16();
      Serial.print("Sensor ");
      Serial.print( u8SensorIdx % MAX_NUMBER_OF_SENSORS ); 
      Serial.print(" capacitance = ");
      Serial.println( u16CapValue, DEC );

      u8TxBuf[0] = ( u8SensorIdx % MAX_NUMBER_OF_SENSORS );
      u8TxBuf[1] = eSENSOR_VALUE_CAPACITANCE;
      u8TxBuf[2] = ( ( u16CapValue & 0xFF00 ) >> 8 );
      u8TxBuf[3] = ( uint8_t( u16CapValue & 0x00FF ) );

      FRDMSerialComm.write( u8TxBuf, 8 );
    }

    delay(100);

    if(pRemoteTempCharacteristic->canRead()) {
      float fTempValue = pRemoteTempCharacteristic->readFloat();
      Serial.print("Sensor ");
      Serial.print( u8SensorIdx % MAX_NUMBER_OF_SENSORS ); 
      Serial.print(" temperature = ");
      Serial.println( fTempValue );
      
      u8TxBuf[0] = ( u8SensorIdx % MAX_NUMBER_OF_SENSORS );
      u8TxBuf[1] = eSENSOR_VALUE_TEMPERATURE;
      u8TxBuf[2] = ( (int8_t)fTempValue );

      FRDMSerialComm.write( u8TxBuf, 8 );
    }

    // got the data from this sensor, now disconnect and start scanning for other sensors
    u8SensorIdx++;
    pClient->disconnect();

  }//else if(doScan){
  //  BLEDevice::getScan()->start(0);   // this is just example to start scan after disconnect, most likely there is better way to do it in arduino
  //}
  Serial.println( "He's still alive");
  delay(1000); // Delay a second between loops.
} // End of loop