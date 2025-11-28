#include <SPI.h>
#include <mcp_can.h>
#include <Wire.h>
#include <math.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include "driver/ledc.h"

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

MCP_CAN CAN0(CAN0_CS);
Adafruit_BNO055 bno = Adafruit_BNO055(28);

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

  Serial.printf("Yaw: %.2f deg | Ticks: %ld\n", yaw, (long)cnt);
  Serial.println(status == CAN_OK ? " [OK]" : " [ERR]");
}


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

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  Serial.println("\n\nESP32-C3 Super Mini: IMU + Encoder + CAN");
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

  Serial.println("System Ready. Sending messages...");
}

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