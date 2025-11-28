#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>

// BLE Service and Characteristic UUIDs (must match server)
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

// BLE objects
static BLEAddress *pServerAddress;
static boolean doConnect = false;
static boolean connected = false;
static BLERemoteCharacteristic* pRemoteCharacteristic;

// Client Callbacks
class MyClientCallback : public BLEClientCallbacks {
  void onConnect(BLEClient* pclient) {
    Serial.println("Connected to server");
  }

  void onDisconnect(BLEClient* pclient) {
    connected = false;
    Serial.println("Disconnected from server");
  }
};

// Notification Callback - Now parses 8 bytes into 4 Integers
static void notifyCallback(
  BLERemoteCharacteristic* pBLERemoteCharacteristic,
  uint8_t* pData,
  size_t length,
  bool isNotify) {
  
    if (length == 8) {
      // --- RAW HEX DEBUGGING (Optional, keeping it for reference) ---
      /*
      Serial.print("Raw Hex: ");
      for (int i = 0; i < 8; i++) {
        Serial.printf("%02X ", pData[i]);
      }
      Serial.println();
      */

      // --- DECODING LOGIC ---
      // Combine bytes to form 16-bit signed integers (Big Endian)
      // We use int16_t so negative coordinates (like -100) are handled correctly.
      int16_t val1_X      = (pData[2] << 8) | pData[3];
      int16_t val2_Y  = (pData[4] << 8) | pData[5];
      int16_t val3_Angle = (pData[6] << 8) | pData[7];

      // --- PRINT READABLE VALUES ---
      Serial.println("=== DATA PACKET RECEIVED ===");
      Serial.printf(" X Coordinate: %d\n", val1_X);
      Serial.printf(" Y Coordinate: %d\n", val2_Y);
      Serial.printf(" Angle (deg):  %d\n", val3_Angle);
      Serial.println("============================");
    
    } else {
      Serial.print("Unexpected data length: ");
      Serial.print(length);
      Serial.println(" bytes (expected 8)");
    }
}

// Advertised Device Callback
class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice advertisedDevice) {
    Serial.print("Found BLE Device: ");
    Serial.println(advertisedDevice.toString().c_str());

    // Check if device has the service we're looking for
    if (advertisedDevice.haveServiceUUID() &&
        advertisedDevice.getServiceUUID().equals(BLEUUID(SERVICE_UUID))) {
        
      advertisedDevice.getScan()->stop();
      pServerAddress = new BLEAddress(advertisedDevice.getAddress());
      doConnect = true;
      Serial.println("Target server found! Stopping scan...");
    }
  }
};

bool connectToServer() {
  Serial.print("Connecting to ");
  Serial.println(pServerAddress->toString().c_str());

  BLEClient* pClient = BLEDevice::createClient();
  Serial.println(" - Created client");

  pClient->setClientCallbacks(new MyClientCallback());

  // Connect to the remote BLE Server
  if (!pClient->connect(*pServerAddress)) {
    Serial.println(" - Connection failed");
    return false;
  }
  Serial.println(" - Connected to server");

  // Obtain a reference to the service
  BLERemoteService* pRemoteService = pClient->getService(SERVICE_UUID);
  if (pRemoteService == nullptr) {
    Serial.print("Failed to find service UUID: ");
    Serial.println(SERVICE_UUID);
    pClient->disconnect();
    return false;
  }
  Serial.println(" - Found service");

  // Obtain a reference to the characteristic
  pRemoteCharacteristic = pRemoteService->getCharacteristic(CHARACTERISTIC_UUID);
  if (pRemoteCharacteristic == nullptr) {
    Serial.print("Failed to find characteristic UUID: ");
    Serial.println(CHARACTERISTIC_UUID);
    pClient->disconnect();
    return false;
  }
  Serial.println(" - Found characteristic");

  // Register for notifications
  if(pRemoteCharacteristic->canNotify()) {
    pRemoteCharacteristic->registerForNotify(notifyCallback);
    Serial.println(" - Registered for notifications");
  }

  connected = true;
  return true;
}

void setup() {
  Serial.begin(115200);
  Serial.println("ESP32-C3 BLE Client Starting...");
  Serial.println("Waiting to decode coordinates...");

  BLEDevice::init("ESP32-C3-Client");

  // Start scanning
  BLEScan* pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setActiveScan(true);
  pBLEScan->setInterval(100);
  pBLEScan->setWindow(99);

  Serial.println("Scanning for BLE servers...");
}

void loop() {
  // If we're not connected and haven't found a server, scan
  if (!connected && !doConnect) {
    BLEScan* pBLEScan = BLEDevice::getScan();
    pBLEScan->start(5, false);
    Serial.println("Scanning for server...");
  }

  // If we found a server and need to connect
  if (doConnect && !connected) {
    if (connectToServer()) {
      Serial.println("Successfully connected to BLE server");
    } else {
      Serial.println("Failed to connect to server");
    }
    doConnect = false;
  }

  // If we're connected, just wait for notifications
  if (connected) {
    // Data is received via callback automatically
    delay(1000);
  } else {
    // If not connected, wait a bit before scanning again
    delay(2000);
  }
}