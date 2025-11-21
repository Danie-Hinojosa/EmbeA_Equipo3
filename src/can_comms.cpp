#include <SPI.h>
#include <mcp_can.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include "driver/ledc.h"

// ==========================================================
// PIN DEFINITIONS (ESP32-C3 Super Mini)
// ==========================================================

// --- CAN Module (MCP2515) ---
#define CAN0_CS      GPIO_NUM_7   // Chip Select
#define CAN0_INT     GPIO_NUM_2   // Interrupt Pin
#define SPI_SCK      GPIO_NUM_4   // SCK
#define SPI_MISO     GPIO_NUM_5   // MISO
#define SPI_MOSI     GPIO_NUM_6   // MOSI

// --- IMU (BNO055) ---
#define I2C_SDA_PIN  8
#define I2C_SCL_PIN  9

// --- Encoder ---
const int encoderA = 10;
const int encoderB = 3;

// ==========================================================
// OBJECTS & VARIABLES
// ==========================================================

MCP_CAN CAN0(CAN0_CS);
Adafruit_BNO055 bno = Adafruit_BNO055(28);

// --- CAN Variables ---
unsigned long prevTX = 0;
const unsigned int invlTX = 1000;
long unsigned int rxId;
unsigned char len;
unsigned char rxBuf[8];
bool canInitialized = false;
bool imuInitialized = false;
unsigned long receivedCount = 0;

// --- Encoder Variables ---
volatile int32_t encoderCount = 0;
const float wheelCircumference_cm = 20.42035f; // (π * 6.5)
const int pulsesPerRevolution = 600; 
float lastDistanceCm = 0;

// ==========================================================
// INTERRUPT SERVICE ROUTINES (ENCODER)
// ==========================================================

void IRAM_ATTR isrA() {
  int a = digitalRead(encoderA);
  int b = digitalRead(encoderB);
  if (a == HIGH && b == LOW)      encoderCount++;
  else if (a == LOW && b == HIGH) encoderCount--;
}

void IRAM_ATTR isrB() {
  int a = digitalRead(encoderA);
  int b = digitalRead(encoderB);
  if (b == HIGH && a == HIGH)     encoderCount++;
  else if (b == LOW && a == LOW)  encoderCount--;
}

// ==========================================================
// SETUP
// ==========================================================

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  Serial.println("\n\nESP32-C3 Super Mini: IMU + Encoder + CAN (Core 3.0 Fixed)");
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
  
  // 5. Attach Interrupts
  attachInterrupt(digitalPinToInterrupt(encoderA), isrA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderB), isrB, CHANGE);

  Serial.println("System Ready. Sending messages...");
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

// ==========================================================
// LOOP
// ==========================================================

void loop() {
  // 1. Handle Received CAN Messages
  if (!digitalRead(CAN0_INT)) {
    receiveCANMessage();
  }

  // 2. Periodic Transmission
  if (millis() - prevTX >= invlTX) {
    prevTX = millis();
    sendDataGroup();
  }
}

// ==========================================================
// CAN TRANSMISSION LOGIC
// ==========================================================

void sendDataGroup() {
  if (!canInitialized) return;

  // --- PART 1: Get IMU Data ---
  float yaw = 0.0f;
  
  if (imuInitialized) {
    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    yaw = euler.x();
    
    if (yaw > 180) {
       yaw = -1 * (360 - yaw);
    }
  }

  // --- PART 2: Get Encoder Data ---
  int32_t cnt;
  noInterrupts();
  cnt = encoderCount;
  interrupts();
  float revs = (float)cnt / pulsesPerRevolution;
  float distance_cm = revs * wheelCircumference_cm;
  lastDistanceCm = distance_cm;

  // Helper union for Float to Bytes
  union { float f; byte b[4]; } conv;
  byte txData[8];
  byte status;

  // FRAME 1: ID 0x40 -> Yaw (Float)
  conv.f = yaw; 
  memcpy(&txData[0], conv.b, 4);
  memset(&txData[4], 0, 4);
  CAN0.sendMsgBuf(0x40, 0, 8, txData);
  delay(2);

  // --- FRAME 2 (ID 0x30): Distance, Encoder Count ---
  // Bytes 0-3: Distance (float), Bytes 4-7: Count (int32)
  memcpy(&txData[0], &distance_cm, sizeof(float));
  memcpy(&txData[4], &cnt, sizeof(int32_t));
  status = CAN0.sendMsgBuf(0x30, 0, 8, txData); 

  // --- Local Print ---
  Serial.printf("Yaw: %.2f | Dist: %.2f cm | Cnt: %d\n", yaw, distance_cm, cnt);
  Serial.println(status == CAN_OK ? " [OK]" : " [ERR]");
}

// ==========================================================
// CAN RECEPTION LOGIC
// ==========================================================

void receiveCANMessage() {
  if (CAN0.readMsgBuf(&rxId, &len, rxBuf) == CAN_OK) {
    receivedCount++;
    Serial.print("RECV #"); Serial.print(receivedCount);
    Serial.print(" ID: 0x"); Serial.print(rxId, HEX);
    Serial.print(" Len: "); Serial.print(len);
    
    // Check for Orientation X loopback (ID 0x40)
    if (rxId == 0x40 && len == 8) {
       union { byte b[4]; float f; } data;
       data.b[0] = rxBuf[0]; data.b[1] = rxBuf[1];
       data.b[2] = rxBuf[2]; data.b[3] = rxBuf[3];
       Serial.print(" [Orient X: "); Serial.print(data.f); Serial.print("]");
    }
    Serial.println();
  }
}