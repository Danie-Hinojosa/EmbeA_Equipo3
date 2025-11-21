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

// --- ESC (Motor) ---
#define ESC_PIN      GPIO_NUM_20
// Note: ESC_CH is no longer needed in ESP32 Core 3.0+
#define ESC_FREQ     50           // 50 Hz
#define ESC_RES      12           // 12-bit resolution

// ==========================================================
// OBJECTS & VARIABLES
// ==========================================================

MCP_CAN CAN0(CAN0_CS);
Adafruit_BNO055 bno = Adafruit_BNO055(28);

// --- CAN Variables ---
unsigned long prevTX = 0;
const unsigned int invlTX = 1000; // Transmission interval (1 second)
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
// HELPER FUNCTIONS (ESC & CONVERSION)
// ==========================================================

// Convert microseconds to Duty Cycle
uint32_t usToDuty(uint16_t us) {
  const uint32_t maxDuty = (1u << ESC_RES) - 1;
  return (uint32_t)((us / 20000.0f) * maxDuty);
}

// Write Microseconds to ESC
void escWriteMicroseconds(uint16_t us) {
  if (us < 1000) us = 1000;
  if (us > 2000) us = 2000;
  
  // FIX: In ESP32 Core 3.0+, ledcWrite takes the PIN, not the channel
  ledcWrite(ESC_PIN, usToDuty(us));
}

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

  // 4. Initialize ESC (PWM) - FIX FOR CORE 3.0+
  // ledcSetup and ledcAttachPin are gone. Replaced by ledcAttach.
  ledcAttach(ESC_PIN, ESC_FREQ, ESC_RES);
  
  escWriteMicroseconds(1000); // Arm ESC (Min throttle)
  delay(1000); 

  // 5. Initialize Encoder Pins
  pinMode(encoderA, INPUT_PULLUP);
  pinMode(encoderB, INPUT_PULLUP);
  pinMode(CAN0_INT, INPUT);
  
  // 6. Attach Interrupts
  attachInterrupt(digitalPinToInterrupt(encoderA), isrA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderB), isrB, CHANGE);

  Serial.println("System Ready. Sending messages...");
}

void initializeCAN() {
  // Adjust MCP_8MHZ to MCP_16MHZ if your module uses a 16MHz crystal
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

  // 2. Handle Serial Commands (for ESC testing)
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 't') escTestSweep();
  }

  // 3. Periodic Transmission
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
  float X = 0, Y = 0, Z = 0, Ax = 0, Ay = 0, Az = 0;
  if (imuInitialized) {
    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
    X = euler.x(); Y = euler.y(); Z = euler.z();
    Ax = accel.x(); Ay = accel.y(); Az = accel.z();
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

  // --- FRAME 1 (ID 0x40): IMU Orient X, Y ---
  conv.f = X; memcpy(&txData[0], conv.b, 4);
  conv.f = Y; memcpy(&txData[4], conv.b, 4);
  CAN0.sendMsgBuf(0x40, 8, txData);
  delay(50); // Small delay to prevent buffer overflow

  // --- FRAME 2 (ID 0x41): IMU Orient Z, Accel X ---
  conv.f = Z;  memcpy(&txData[0], conv.b, 4);
  conv.f = Ax; memcpy(&txData[4], conv.b, 4);
  CAN0.sendMsgBuf(0x41, 8, txData);
  delay(50);

  // --- FRAME 3 (ID 0x42): IMU Accel Y, Accel Z ---
  conv.f = Ay; memcpy(&txData[0], conv.b, 4);
  conv.f = Az; memcpy(&txData[4], conv.b, 4);
  CAN0.sendMsgBuf(0x42, 8, txData);
  delay(50);

  // --- FRAME 4 (ID 0x30): Distance, Encoder Count ---
  // Bytes 0-3: Distance (float), Bytes 4-7: Count (int32)
  memcpy(&txData[0], &distance_cm, sizeof(float));
  memcpy(&txData[4], &cnt, sizeof(int32_t));
  status = CAN0.sendMsgBuf(0x30, 0, 8, txData); // Standard ID

  // --- Local Print ---
  Serial.print("SENT -> IMU: "); Serial.print(X); 
  Serial.print(" | Dist: "); Serial.print(distance_cm);
  Serial.println(status == CAN_OK ? " [OK]" : " [ERR]");

  // --- Logic: Motor Control based on distance ---
  if (distance_cm >= 100.0f) {
    escWriteMicroseconds(1000); // STOP
  } else {
    escWriteMicroseconds(1300); // Slow Forward
  }
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

// ==========================================================
// ESC TEST SWEEP
// ==========================================================
void escTestSweep() {
  Serial.println("[TEST] ESC Sweep Start");
  escWriteMicroseconds(1000); delay(1000);
  escWriteMicroseconds(1300); delay(1000);
  escWriteMicroseconds(1700); delay(1000);
  escWriteMicroseconds(1000); delay(1000);
  Serial.println("[TEST] ESC Sweep End");
}