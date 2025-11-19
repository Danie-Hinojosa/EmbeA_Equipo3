#include <SPI.h>
#include <mcp_can.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
/*
MCP2515   ESP32-C3 Super Mini   Function
=======   ===================   ========
VCC   ->  5.0V                  Power
GND   ->  GND                   Ground
CS    ->  GPIO7 (D7)            Chip Select
SO    ->  GPIO5 (D5)            MISO (Master In Slave Out)
SI    ->  GPIO6 (D6)            MOSI (Master Out Slave In)
SCK   ->  GPIO4 (D4)            Serial Clock
INT   ->  GPIO2 (D2)            Interrupt
*/
#define I2C_SDA_PIN 8
#define I2C_SCL_PIN 9
Adafruit_BNO055 bno = Adafruit_BNO055(28);
// CAN Module Wiring for ESP32-C3 Super Mini - Corrected Pins
#define CAN0_INT GPIO_NUM_2   // INT pin
#define CAN0_CS  GPIO_NUM_7   // CS pin

// SPI Pins (based on your specification)
#define SPI_SCK  GPIO_NUM_4   // SCK pin
#define SPI_MISO GPIO_NUM_5   // MISO pin (SO on MCP2515 connects to MISO on ESP32)
#define SPI_MOSI GPIO_NUM_6   // MOSI pin (SI on MCP2515 connects to MOSI on ESP32)

MCP_CAN CAN0(CAN0_CS);  // Set CS pin

// CAN Variables
unsigned long prevTX = 0;
const unsigned int invlTX = 1000;  // Transmission interval (1 second)
byte txData[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};  // Data to send
float orientation_x = 0.0f;

// CAN Receive Variables
long unsigned int rxId;
unsigned char len;
unsigned char rxBuf[8];

// Status tracking
bool canInitialized = false;
bool imuInitialized = false;
unsigned long receivedCount = 0;
unsigned long sentCount = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);
  
  // Initialize SPI with corrected pins for ESP32-C3 Super Mini
  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI, CAN0_CS); // SCK, MISO, MOSI, CS
  initializeIMU();
  // Initialize CAN controller
  initializeCAN();

  // Set pin modes
  pinMode(CAN0_INT, INPUT);
  
  Serial.println("\n\nESP32-C3 Super Mini CAN Bus Example");
  Serial.println("===================================");
  Serial.println("Board: ESP32-C3 Super Mini");
  Serial.print("CS Pin: GPIO"); Serial.println(CAN0_CS);
  Serial.print("INT Pin: GPIO"); Serial.println(CAN0_INT);
  Serial.print("SCK Pin: GPIO"); Serial.println(SPI_SCK);
  Serial.print("MISO Pin: GPIO"); Serial.println(SPI_MISO);
  Serial.print("MOSI Pin: GPIO"); Serial.println(SPI_MOSI);
  Serial.println("Sending messages every second...");
  Serial.println("Waiting for incoming messages...");
  Serial.println();
}

void initializeCAN() {
  // Initialize MCP2515 running at 16MHz with a baudrate of 500kb/s
  if (CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK) {
    Serial.println("MCP2515 Initialized Successfully!");
    canInitialized = true;
    CAN0.setMode(MCP_NORMAL);  // Set mode to normal to allow messages to be transmitted Loop Back
  } else {
    Serial.println("Error Initializing MCP2515...");
    Serial.println("Check wiring and try again.");
    canInitialized = false;
  }
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

void loop() {
  // Handle received CAN messages
  if (!digitalRead(CAN0_INT)) {
    receiveCANMessage();
  }
  
  // Send CAN message at regular intervals
  if (millis() - prevTX >= invlTX) {
    prevTX = millis();
    sendCANMessage();
  }
}

void receiveCANMessage() {
  if (CAN0.readMsgBuf(&rxId, &len, rxBuf) == CAN_OK) {
    receivedCount++;
    
    // Print formatted message details to serial
    Serial.print("RECV #");
    Serial.print(receivedCount);
    Serial.print(" | ID: 0x");
    Serial.print(rxId, HEX);
    Serial.print(" | Len: ");
    Serial.print(len);
    Serial.print(" | Data: ");
    
    for (int i = 0; i < len; i++) {
      if (rxBuf[i] < 0x10) Serial.print("0"); // Leading zero for single digit hex
      Serial.print(rxBuf[i], HEX);
      Serial.print(" ");
    }

    if (len >= 4 && rxId == 0x40) {
      // Use a union to re-interpret the byte array as a float
      union {
          byte b[4];
          float f;
      } data;
      
      data.b[0] = rxBuf[0];
      data.b[1] = rxBuf[1];
      data.b[2] = rxBuf[2];
      data.b[3] = rxBuf[3];
      
      Serial.print(" | Orientation X (Float): ");
      Serial.print(data.f, 2); // Print with 2 decimal places
    }
    
    // Display ASCII representation (if printable)
    Serial.print("| ASCII: ");
    for (int i = 0; i < len; i++) {
      if (rxBuf[i] >= 32 && rxBuf[i] <= 126) {
        Serial.print((char)rxBuf[i]);
      } else {
        Serial.print(".");
      }
    }
    
    Serial.println();
  }
}

void sendCANMessage() {
  if (!canInitialized) return;

  // Read IMU data
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);

  float X = euler.x();
  float Y = euler.y();
  float Z = euler.z();

  float Ax = accel.x();
  float Ay = accel.y();
  float Az = accel.z();

  // ===== Helper union =====
  union {
    float f;
    byte b[4];
  } conv;

  byte txData[8];
  byte status;

  // ---------------------------------------------------------
  // FRAME 1 — ID = 0x40 — Orientation X, Y
  // ---------------------------------------------------------
  conv.f = X;
  memcpy(&txData[0], conv.b, 4);

  conv.f = Y;
  memcpy(&txData[4], conv.b, 4);

  status = CAN0.sendMsgBuf(0x40, 8, txData);
  delay(100);
  Serial.print("ID 0x40: ");
  for (int i = 0; i < 8; i++) Serial.printf("%02X ", txData[i]);
  Serial.println(status == CAN_OK ? " OK" : " ERR");


  // ---------------------------------------------------------
  // FRAME 2 — ID = 0x41 — Orientation Z + Accel X
  // ---------------------------------------------------------
  conv.f = Z;
  memcpy(&txData[0], conv.b, 4);

  conv.f = Ax;
  memcpy(&txData[4], conv.b, 4);

  status = CAN0.sendMsgBuf(0x41, 8, txData);
  delay(100);
  Serial.print("ID 0x41: ");
  for (int i = 0; i < 8; i++) Serial.printf("%02X ", txData[i]);
  Serial.println(status == CAN_OK ? " OK" : " ERR");


  // ---------------------------------------------------------
  // FRAME 3 — ID = 0x42 — Accel Y, Accel Z
  // ---------------------------------------------------------
  conv.f = Ay;
  memcpy(&txData[0], conv.b, 4);

  conv.f = Az;
  memcpy(&txData[4], conv.b, 4);

  status = CAN0.sendMsgBuf(0x42, 8, txData);
  delay(100);
  Serial.print("ID 0x42: ");
  for (int i = 0; i < 8; i++) Serial.printf("%02X ", txData[i]);
  Serial.println(status == CAN_OK ? " OK" : " ERR");
  Serial.println();
}



/*********************************************************************************************************
  END FILE
*********************************************************************************************************/