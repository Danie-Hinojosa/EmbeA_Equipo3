#include <SPI.h>
#include <mcp_can.h>
#include <Wire.h>
#include <math.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include "driver/ledc.h"
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>

// --- CAN Module (MCP2515) ---
#define CAN0_CS      GPIO_NUM_7   // Chip Select
#define CAN0_INT     GPIO_NUM_2   // Interrupt Pin
#define SPI_SCK      GPIO_NUM_4   // SCK
#define SPI_MISO     GPIO_NUM_5   // MISO
#define SPI_MOSI     GPIO_NUM_6   // MOSI

// --- IMU (BNO055) ---
#define I2C_SDA_PIN  8
#define I2C_SCL_PIN  9

// --- BLE CONFIGURATION ---
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define CAN_COORD_ID        0x35

// --- Encoder ---
const int encoderA = 10;
const int encoderB = 3;

MCP_CAN CAN0(CAN0_CS);
Adafruit_BNO055 bno = Adafruit_BNO055(28);

// --- BLE OBJECTS ---
static BLEAddress *pServerAddress;
static boolean doConnect = false;
static boolean connected = false;
static BLERemoteCharacteristic* pRemoteCharacteristic;

// --- CAN Variables ---
unsigned long prevTX = 0;
const unsigned int invlTX = 10;
long unsigned int rxId;
unsigned char len;
unsigned char rxBuf[8];
bool canInitialized = false;
bool imuInitialized = false;
unsigned long receivedCount = 0;

// --- Encoder Variables ---
volatile int32_t encoderCount = 0;
volatile int lastA = 0;
volatile int lastB = 0;

// --- Odometry Constants ---
const float WHEEL_DIAMETER_CM = 6.5;
const float TICKS_PER_REV = 240.0;

void IRAM_ATTR isrA() {
  int A = gpio_get_level((gpio_num_t)encoderA);
  int B = gpio_get_level((gpio_num_t)encoderB);
  if (A != lastA) encoderCount += (A == B) ? 1 : -1;
  lastA = A;
}

void IRAM_ATTR isrB() {
  int A = gpio_get_level((gpio_num_t)encoderA);
  int B = gpio_get_level((gpio_num_t)encoderB);
  if (B != lastB) encoderCount += (A != B) ? 1 : -1;
  lastB = B;
}

void initializeCAN() {
  if (CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK) {
    Serial.println("MCP2515 Initialized Successfully!");
    canInitialized = true;
    CAN0.setMode(MCP_NORMAL);
  } else {
    Serial.println("Error Initializing MCP2515... Check wiring.");
    canInitialized = false;
  }
}

void initializeIMU() {
  Serial.println("Initializing BNO055...");
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  if (!bno.begin()) {
    Serial.println("Ooops, no BNO055 detected ... Check I2C wiring!");
    imuInitialized = false;
  } else {
    Serial.println("BNO055 Initialized Successfully!");
    bno.setExtCrystalUse(true);
    imuInitialized = true;
  }
}

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

// --- CRITICAL: DATA BRIDGE (BLE -> CAN) ---
static void notifyCallback(
  BLERemoteCharacteristic* pBLERemoteCharacteristic,
  uint8_t* pData,
  size_t length,
  bool isNotify) {
  
  if (length == 8) {
      // Decode BLE Data (Big Endian)
      int16_t val_X     = (pData[2] << 8) | pData[3];
      int16_t val_Y     = (pData[4] << 8) | pData[5];
      int16_t val_Angle = (pData[6] << 8) | pData[7];

      Serial.printf("[RECV] X: %d | Y: %d | Ang: %d\n", val_X, val_Y, val_Angle);

      // Repackage for CAN Bus
      byte canData[8] = {0};

      // Pack X (2 Bytes)
      canData[0] = (val_X >> 8) & 0xFF;
      canData[1] = val_X & 0xFF;

      // Pack Y (2 Bytes)
      canData[2] = (val_Y >> 8) & 0xFF;
      canData[3] = val_Y & 0xFF;

      // Pack Angle (2 Bytes)
      canData[4] = (val_Angle >> 8) & 0xFF;
      canData[5] = val_Angle & 0xFF;

      // Send via MCP2515
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

    BLERemoteService* pRemoteService = pClient->getService(SERVICE_UUID);
    if (pRemoteService == nullptr) {
        Serial.println("[BLE] Failed to find Service UUID");
        pClient->disconnect();
        return false;
    }

    pRemoteCharacteristic = pRemoteService->getCharacteristic(CHARACTERISTIC_UUID);
    if (pRemoteCharacteristic == nullptr) {
        Serial.println("[BLE] Failed to find Characteristic UUID");
        pClient->disconnect();
        return false;
    }

    if(pRemoteCharacteristic->canNotify()) {
        pRemoteCharacteristic->registerForNotify(notifyCallback);
        Serial.println("[BLE] Notifications Enabled. Bridge Ready.");
    }

    connected = true;
    return true;
}

void sendDataGroup() {
  if (!canInitialized) return;

  float yaw = 0.0f;
  if (imuInitialized) {
    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    yaw = euler.x();
    if (yaw < 0) yaw += 360.0f; 
  }

  int32_t cnt;
  noInterrupts();
  cnt = encoderCount;
  interrupts();

  byte txData[8] = {0};
  
  memcpy(&txData[0], &yaw, sizeof(float));      
  memcpy(&txData[4], &cnt, sizeof(cnt));

  byte status = CAN0.sendMsgBuf(0x30, 0, 8, txData);

  Serial.printf("Yaw: %.2f deg | Ticks: %ld", yaw, (long)cnt);
  Serial.println(status == CAN_OK ? " [OK]" : " [ERR]");
}

void receiveCANMessage() {
  if (CAN0.readMsgBuf(&rxId, &len, rxBuf) == CAN_OK) {
    receivedCount++;
    Serial.print("RECV #"); Serial.print(receivedCount);
    Serial.print(" ID: 0x"); Serial.print(rxId, HEX);
    Serial.print(" Len: "); Serial.print(len);
    
    if (rxId == 0x40 && len == 8) {
      union { byte b[4]; float f; } data;
      data.b[0] = rxBuf[0]; data.b[1] = rxBuf[1];
      data.b[2] = rxBuf[2]; data.b[3] = rxBuf[3];
      Serial.print(" [Orient X: "); Serial.print(data.f); Serial.print("]");
    }
    Serial.println();
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("\n\nESP32-C3 Super Mini: IMU + Encoder + CAN + BLE Bridge");
  Serial.println("==========================================================");

  // 1. Initialize SPI
  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI, CAN0_CS);

  // 2. Initialize CAN
  initializeCAN();

  // 3. Initialize IMU
  initializeIMU();

  // 4. Initialize Encoder Pins
  pinMode(encoderA, INPUT_PULLUP);
  pinMode(encoderB, INPUT_PULLUP);
  pinMode(CAN0_INT, INPUT);

  lastA = gpio_get_level((gpio_num_t)encoderA);
  lastB = gpio_get_level((gpio_num_t)encoderB);
  
  // 5. Attach Interrupts
  attachInterrupt(digitalPinToInterrupt(encoderA), isrA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderB), isrB, CHANGE);

  // 6. Initialize BLE
  BLEDevice::init("ESP32-C3-Gateway");
  BLEScan* pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setActiveScan(true);
  pBLEScan->setInterval(100);
  pBLEScan->setWindow(99);

  Serial.println("[BLE] Scanning for Server...");
  pBLEScan->start(5, false);

  Serial.println("System Ready. Listening for BLE and sending CAN messages...");
}

void loop() {
    // 1. BLE Connection Logic
    if (doConnect) {
      if (connectToServer()) {
        Serial.println("[SYS] Bridge Active: Listening for Camera Coordinates...");
      } else {
        Serial.println("[SYS] Connection Failed. Retrying...");
      }
      doConnect = false;
    }

    // 2. BLE Re-connection / Scan Logic
    if (!connected) {
      static unsigned long lastScan = 0;
      if (millis() - lastScan > 5000) {
         Serial.println("[BLE] Scanning...");
         BLEDevice::getScan()->start(5, false);
         lastScan = millis();
      }
    } else {  
      // 3. Handle Received CAN Messages
      if (!digitalRead(CAN0_INT)) {
        receiveCANMessage();
      }
      // 4. Periodic CAN Transmission (IMU + Encoder)
      if (millis() - prevTX >= invlTX) {
        prevTX = millis();
        sendDataGroup();
      }
    }
}