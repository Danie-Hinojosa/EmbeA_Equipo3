#include <SPI.h>
#include <mcp_can.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

// --- BNO055 (IMU) Setup ---
// SDA=GPIO8, SCL=GPIO9
#define I2C_SDA_PIN 8
#define I2C_SCL_PIN 9
Adafruit_BNO055 bno = Adafruit_BNO055(55);

// --- CAN Module Wiring for ESP32-C3 Super Mini ---
#define CAN0_INT GPIO_NUM_2   // INT pin
#define CAN0_CS  GPIO_NUM_7   // CS pin
// SPI Pins
#define SPI_SCK  GPIO_NUM_4   // SCK pin
#define SPI_MISO GPIO_NUM_5   // MISO pin (SO on MCP2515)
#define SPI_MOSI GPIO_NUM_6   // MOSI pin (SI on MCP2515)
MCP_CAN CAN0(CAN0_CS);        // Set CS pin

// --- CAN DATABASE IDs ---
// We need 4 messages to send all 6 floats (Accel + Orientation)
#define CAN_ID_ORIENT_XY 0x100 // Orientation X, Y (2 floats = 8 bytes)
#define CAN_ID_ORIENT_Z  0x101 // Orientation Z  (1 float = 4 bytes)
#define CAN_ID_ACCEL_XY  0x102 // Accelerometer X, Y (2 floats = 8 bytes)
#define CAN_ID_ACCEL_Z   0x103 // Accelerometer Z  (1 float = 4 bytes)

// --- CAN Timing ---
unsigned long prevTX = 0;
const unsigned int invlTX = 100; // Send messages every 100ms

// --- CAN Receive Variables ---
long unsigned int rxId;
unsigned char len;
unsigned char rxBuf[8];

// --- Status ---
bool canInitialized = false;
bool imuInitialized = false;

void initializeCAN();
void initializeIMU();
void receiveCANMessage();
void sendIMUDataOverCAN();


void setup() {
  Serial.begin(115200);
  //while (!Serial) delay(10);

  // Initialize I2C for BNO055 (IMU)
  initializeIMU();

  // Initialize SPI with corrected pins for ESP32-C3 Super Mini
  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI, CAN0_CS); // SCK, MISO, MOSI, CS

  // Initialize CAN controller
  initializeCAN();

  // Set pin modes
  pinMode(CAN0_INT, INPUT);

  Serial.println("\n\nESP32-C3 IMU-to-CAN Broadcaster");
  Serial.println("===================================");
  Serial.print("CS Pin: GPIO");  Serial.println(CAN0_CS);
  Serial.print("INT Pin: GPIO"); Serial.println(CAN0_INT);
  Serial.println("Waiting for incoming messages...");
  Serial.println();
}

void initializeIMU() {
  Serial.println("Initializing BNO055 (IMU)...");
  // Initialize I2C with pins from your schematic
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN); 

  if(!bno.begin())
  {
    Serial.print("Ooops, no BNO055 detected ... Check wiring!");
    imuInitialized = false;
  } else {
    Serial.println("BNO055 Initialized Successfully!");
    bno.setExtCrystalUse(true);
    imuInitialized = true;
  }
}

void initializeCAN() {
  // Cambia MCP_8MHZ por MCP_16MHZ si tu módulo es de 16 MHz
  if (CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK) {
    Serial.println("MCP2515 Initialized Successfully!");
    canInitialized = true;
    CAN0.setMode(MCP_NORMAL);  // Normal mode
  } else {
    Serial.println("Error Initializing MCP2515...");
    canInitialized = false;
  }
}

void loop() {

  // Handle received CAN messages
  if (!digitalRead(CAN0_INT)) {
    receiveCANMessage();
  }

  // Send CAN message at regular intervals
  if (millis() - prevTX >= invlTX) {
    prevTX = millis();
    
    // Only send if both systems are good
    if(canInitialized && imuInitialized) {
      sendIMUDataOverCAN();
    }
  }
}


void receiveCANMessage() {
  if (CAN0.readMsgBuf(&rxId, &len, rxBuf) == CAN_OK) {
    Serial.print("RECV | ID: 0x"); Serial.print(rxId, HEX);
    Serial.print(" | Len: "); Serial.print(len);
    Serial.print(" | Data: ");
    for (int i = 0; i < len; i++) {
      if (rxBuf[i] < 0x10) Serial.print("0");
      Serial.print(rxBuf[i], HEX);
      Serial.print(" ");
    }
    Serial.println();
  }
}

void sendIMUDataOverCAN() {
  // Buffer for holding CAN data (max 8 bytes)
  byte can_buf[8];

  /* 1. Get Orientation Data (Euler Angles) */
  sensors_event_t orientationData; 
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);

  /* 2. Get Accelerometer Data */
  sensors_event_t accelData;
  bno.getEvent(&accelData, Adafruit_BNO055::VECTOR_ACCELEROMETER);

  // --- Send Message 1: Orientation X, Y ---
  // Pack 2 floats (4 bytes each) into the 8-byte buffer
  memcpy(can_buf, &orientationData.orientation.x, 4);
  memcpy(can_buf + 4, &orientationData.orientation.y, 4);
  CAN0.sendMsgBuf(CAN_ID_ORIENT_XY, 0, 8, can_buf);

  // --- Send Message 2: Orientation Z ---
  // Pack 1 float into the buffer (only need 4 bytes)
  memcpy(can_buf, &orientationData.orientation.z, 4);
  CAN0.sendMsgBuf(CAN_ID_ORIENT_Z, 0, 4, can_buf);

  // --- Send Message 3: Accelerometer X, Y ---
  memcpy(can_buf, &accelData.acceleration.x, 4);
  memcpy(can_buf + 4, &accelData.acceleration.y, 4);
  CAN0.sendMsgBuf(CAN_ID_ACCEL_XY, 0, 8, can_buf);

  // --- Send Message 4: Accelerometer Z ---
  memcpy(can_buf, &accelData.acceleration.z, 4);
  CAN0.sendMsgBuf(CAN_ID_ACCEL_Z, 0, 4, can_buf);

  Serial.println("TX   | IMU Data Sent (4 messages)");
}