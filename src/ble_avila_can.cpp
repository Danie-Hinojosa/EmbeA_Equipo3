// THIS IS THE CODE FOR CONNECTING TO THE CAMERA AND SENDING DATA VIA CAN TO THE STM32

#include <SPI.h>
#include <mcp_can.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>

// --- HARDWARE CONFIGURATION (From your base code) ---
// CAN Module (MCP2515)
#define CAN0_CS      7   // Chip Select
#define CAN0_INT     2   // Interrupt Pin
#define SPI_SCK      4   // SCK
#define SPI_MISO     5   // MISO
#define SPI_MOSI     6   // MOSI

// --- BLE CONFIGURATION (From your BLE code) ---
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

// --- OBJECTS ---
MCP_CAN CAN0(CAN0_CS);
static BLEAddress *pServerAddress;
static boolean doConnect = false;
static boolean connected = false;
static BLERemoteCharacteristic* pRemoteCharacteristic;

// --- VARIABLES ---
// CAN ID to send coordinates on (Arbitrary choice, make sure STM32 listens to this)
#define CAN_COORD_ID  0x35 

// --- BLE CALLBACKS ---
class MyClientCallback : public BLEClientCallbacks {
  void onConnect(BLEClient* pclient) {
    Serial.println("[BLE] Connected to server");
  }

  void onDisconnect(BLEClient* pclient) {
    connected = false;
    Serial.println("[BLE] Disconnected from server");
  }
};

// --- CRITICAL: DATA BRIDGE ---
// This function runs automatically when BLE data arrives.
// It effectively bridges BLE -> CAN.
static void notifyCallback(
  BLERemoteCharacteristic* pBLERemoteCharacteristic,
  uint8_t* pData,
  size_t length,
  bool isNotify) {
  
  // 1. Validate Data Length (Expect 8 bytes based on your BLE logic)
  if (length == 8) {
      
      // 2. Decode BLE Data (Big Endian as per your snippet)
      // Bytes 0-1 are ignored in your snippet, so we skip them here too.
      int16_t val_X     = (pData[2] << 8) | pData[3];
      int16_t val_Y     = (pData[4] << 8) | pData[5];
      int16_t val_Angle = (pData[6] << 8) | pData[7];

      Serial.printf("[RECV] X: %d | Y: %d | Ang: %d\n", val_X, val_Y, val_Angle);

      // 3. Repackage for CAN Bus
      // We will send 6 bytes of data inside the 8-byte CAN frame.
      byte canData[8] = {0};

      // Pack X (2 Bytes)
      canData[0] = (val_X >> 8) & 0xFF; // High Byte
      canData[1] = val_X & 0xFF;        // Low Byte

      // Pack Y (2 Bytes)
      canData[2] = (val_Y >> 8) & 0xFF;
      canData[3] = val_Y & 0xFF;

      // Pack Angle (2 Bytes)
      canData[4] = (val_Angle >> 8) & 0xFF;
      canData[5] = val_Angle & 0xFF;

      // 4. Send via MCP2515
      byte sendStatus = CAN0.sendMsgBuf(CAN_COORD_ID, 0, 8, canData);
      
      if (sendStatus == CAN_OK) {
        Serial.println("[CAN] Message Sent Successfully");
      } else {
        Serial.println("[CAN] Error Sending Message");
      }

  } else {
      Serial.print("[ERR] Unexpected BLE length: ");
      Serial.println(length);
  }
}

// --- BLE DISCOVERY ---
class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice advertisedDevice) {
    // Check if device matches our Service UUID
    if (advertisedDevice.haveServiceUUID() && 
        advertisedDevice.getServiceUUID().equals(BLEUUID(SERVICE_UUID))) {
        
      Serial.print("[BLE] Target Found: ");
      Serial.println(advertisedDevice.toString().c_str());
      
      advertisedDevice.getScan()->stop();
      pServerAddress = new BLEAddress(advertisedDevice.getAddress());
      doConnect = true;
    }
  }
};

// --- HELPER: CONNECT ROUTINE ---
bool connectToServer() {
    Serial.print("[BLE] Connecting to ");
    Serial.println(pServerAddress->toString().c_str());

    BLEClient* pClient = BLEDevice::createClient();
    pClient->setClientCallbacks(new MyClientCallback());

    if (!pClient->connect(*pServerAddress)) {
        Serial.println("[BLE] Failed to connect");
        return false;
    }

    // Get Service
    BLERemoteService* pRemoteService = pClient->getService(SERVICE_UUID);
    if (pRemoteService == nullptr) {
        Serial.println("[BLE] Failed to find Service UUID");
        pClient->disconnect();
        return false;
    }

    // Get Characteristic
    pRemoteCharacteristic = pRemoteService->getCharacteristic(CHARACTERISTIC_UUID);
    if (pRemoteCharacteristic == nullptr) {
        Serial.println("[BLE] Failed to find Characteristic UUID");
        pClient->disconnect();
        return false;
    }

    // Enable Notifications
    if(pRemoteCharacteristic->canNotify()) {
        pRemoteCharacteristic->registerForNotify(notifyCallback);
        Serial.println("[BLE] Notifications Enabled. Bridge Ready.");
    }

    connected = true;
    return true;
}

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  Serial.println("\nESP32-C3 BLE -> CAN Bridge");
  Serial.println("==================================");

  // 1. Initialize SPI for MCP2515
  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI, CAN0_CS);

  // 2. Initialize CAN Module
  if (CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK) {
    Serial.println("[CAN] MCP2515 Initialized Successfully!");
    CAN0.setMode(MCP_NORMAL);
  } else {
    Serial.println("[CAN] Initialization FAILED. Check Wiring.");
    while (1); // Stop if CAN fails
  }

  // 3. Initialize BLE
  BLEDevice::init("ESP32-C3-Gateway");
  BLEScan* pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setActiveScan(true);
  pBLEScan->setInterval(100);
  pBLEScan->setWindow(99);

  Serial.println("[BLE] Scanning for Server...");
  pBLEScan->start(5, false);
}

void loop() {
  // 1. Connection Logic
  if (doConnect) {
    if (connectToServer()) {
      Serial.println("[SYS] Bridge Active: Listening for Coordinates...");
    } else {
      Serial.println("[SYS] Connection Failed. Retrying...");
    }
    doConnect = false;
  }

  // 2. Re-connection / Scan Logic
  if (!connected) {
    // If we lost connection, scan again periodically
    static unsigned long lastScan = 0;
    if (millis() - lastScan > 5000) {
       Serial.println("[BLE] Scanning...");
       BLEDevice::getScan()->start(5, false);
       lastScan = millis();
    }
  }

}