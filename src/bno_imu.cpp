#include <SPI.h>
#include <mcp_can.h>
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

// CAN Receive Variables
long unsigned int rxId;
unsigned char len;
unsigned char rxBuf[8];

// Status tracking
bool canInitialized = false;
unsigned long receivedCount = 0;
unsigned long sentCount = 0;

void initializeCAN();
void receiveCANMessage();
void sendCANMessage();

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);
  
  // Initialize SPI with corrected pins for ESP32-C3 Super Mini
  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI, CAN0_CS); // SCK, MISO, MOSI, CS
  
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
    //CAN0.setMode(MCP_NORMAL);  // Set mode to normal to allow messages to be transmitted Loop Back
  } else {
    Serial.println("Error Initializing MCP2515...");
    Serial.println("Check wiring and try again.");
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
  if (canInitialized) {
    // Use a simple ID that increments with each message
    //unsigned long messageId = 0x100 + (sentCount % 0x100);
    unsigned long messageId = 0x40;
    
    byte sndStat = CAN0.sendMsgBuf(messageId, 8, txData);
    
    if (sndStat == CAN_OK) {
      //Serial.print("SENT #");
      //Serial.print(sentCount + 1);
      Serial.print("ID: 0x");
      Serial.print(messageId, HEX);
      Serial.print(" | Data: ");
      
      for (int i = 0; i < 8; i++) {
        if (txData[i] < 0x10) Serial.print("0");
        Serial.print(txData[i], HEX);
        Serial.print(" ");
      }
      Serial.println();
      
      //sentCount++;
      
      // Modify data for next transmission
      //for (int i = 0; i < 8; i++) {
      //  txData[i] = (txData[i] + 1) % 0x100;
      //}
    } else {
      Serial.println("ERROR: Failed to send message");
    }
  }
}

/*********************************************************************************************************
  END FILE
*********************************************************************************************************/