#include <SPI.h>
#include <mcp_can.h>
#include "driver/ledc.h"
#define ESC_PIN GPIO_NUM_20
#define ESC_CH  0
#define ESC_FREQ 50       // 50 Hz (periodo 20 ms)
#define ESC_RES  12        

// CAN Module Wiring for ESP32-C3 Super Mini - Corrected Pins
#define CAN0_INT GPIO_NUM_2   // INT pin
#define CAN0_CS  GPIO_NUM_7   // CS pin
// SPI Pins
#define SPI_SCK  GPIO_NUM_4   // SCK pin
#define SPI_MISO GPIO_NUM_5   // MISO pin (SO on MCP2515 connects to MISO on ESP32)
#define SPI_MOSI GPIO_NUM_6   // MOSI pin (SI on MCP2515 connects to MOSI on ESP32)
MCP_CAN CAN0(CAN0_CS);        // Set CS pin

// --- Encoder (A/B en pines con interrupción) ---
const int encoderA = 10;
const int encoderB = 3;

// ======= CONFIG ENCODER (usa tu diámetro 6.5 cm) =======  // <<< ADD
volatile int32_t encoderCount = 0;                          // <<< ADD
const float   wheelCircumference_cm = 20.42035f;            // <<< ADD  (π*6.5)
const int     pulsesPerRevolution  = 600;                   // <<< ADD  (ajusta a tu PPR real)

// CAN Variables
unsigned long prevTX = 0;
const unsigned int invlTX = 1000;  // Transmission interval (1 second)
// byte txData[] = { ... };  // ya no lo usamos para el envío de distancia

// CAN Receive Variables
long unsigned int rxId;
unsigned char len;
unsigned char rxBuf[8];

// Status tracking
bool canInitialized = false;
unsigned long receivedCount = 0;
unsigned long sentCount = 0;
float lastDistanceCm = 0;

void initializeCAN();
void receiveCANMessage();
void sendCANMessage();
void escTestSweep();

// Conversión microsegundos -> duty (para 50 Hz)
uint32_t usToDuty(uint16_t us) {
  const uint32_t maxDuty = (1u << ESC_RES) - 1;
  return (uint32_t)((us / 20000.0f) * maxDuty); // 20 ms = 20000 us
}

// Escribe pulso en microsegundos (typ: 1000–2000)
void escWriteMicroseconds(uint16_t us) {
  if (us < 1000)  us = 1000;
  if (us > 2000) us = 2000;
  ledcWrite(ESC_CH, usToDuty(us));
}

// Conveniencia: % de 0–100 → 1000–2000 us
void escWritePercent(float pct) {
  if (pct < 0) pct = 0;
  if (pct > 100) pct = 100;
  uint16_t us = 1000 + (uint16_t)(pct * 10.0f); // 0%→1000us, 100%→2000us
  escWriteMicroseconds(us);
}

// ---------------------- ISRs ENCODER ----------------------  // <<< ADD
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
// ----------------------------------------------------------  // <<< ADD

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  // Initialize SPI with corrected pins for ESP32-C3 Super Mini
  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI, CAN0_CS); // SCK, MISO, MOSI, CS

  // Initialize CAN controller
  initializeCAN();

  // ---- ESC: configurar PWM LEDC y armar ----
  ledcSetup(ESC_CH, ESC_FREQ, ESC_RES);
  ledcAttachPin(ESC_PIN, ESC_CH);
  ledcWrite(ESC_CH, usToDuty(1500));
  delay(500);
  // Secuencia de armado típica (ajusta si tu ESC requiere otra)
  // Mantener en mínimo (1000us) durante el arranque para la mayoría de ESCs
  escWriteMicroseconds(1000);   // mínimo
  delay(2000);

  // Set pin modes
  pinMode(CAN0_INT, INPUT);

  // --- Encoder pins + interrupts ---                           // <<< ADD
  pinMode(encoderA, INPUT_PULLUP);
  pinMode(encoderB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoderA), isrA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderB), isrB, CHANGE);
  // ------------------------------------------------------------

  Serial.println("\n\nESP32-C3 Super Mini CAN Bus Example");
  Serial.println("===================================");
  Serial.println("Board: ESP32-C3 Super Mini");
  Serial.print("CS Pin: GPIO");  Serial.println(CAN0_CS);
  Serial.print("INT Pin: GPIO"); Serial.println(CAN0_INT);
  Serial.print("SCK Pin: GPIO"); Serial.println(SPI_SCK);
  Serial.print("MISO Pin: GPIO");Serial.println(SPI_MISO);
  Serial.print("MOSI Pin: GPIO");Serial.println(SPI_MOSI);
  Serial.println("Sending messages every second...");
  Serial.println("Waiting for incoming messages...");
  Serial.println();
}

void initializeCAN() {
  // Cambia MCP_8MHZ por MCP_16MHZ si tu módulo es de 16 MHz
  if (CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK) {
    Serial.println("MCP2515 Initialized Successfully!");
    canInitialized = true;
    CAN0.setMode(MCP_NORMAL);  // Normal mode
  } else {
    Serial.println("Error Initializing MCP2515...");
    Serial.println("Check wiring and try again.");
    canInitialized = false;
  }
}

void loop() {

  // Handle received CAN messages (si cae algo en el bus)
  if (!digitalRead(CAN0_INT)) {
    receiveCANMessage();
  }
  // Serial commands: 't' -> test sweep, 'C' -> calibración (MAX->MIN), 'd' -> toggle debug
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 't' || c == 'T') {
      Serial.println("ESC test requested: realizando sweep (ser cuidadoso).");
      escTestSweep();
    } else if (c == 'C') {
      Serial.println("ESC CALIBRATION requested: enviar MAX->MIN (asegure motor desconectado o zona segura).");
      // ejecutar calibración sólo si el usuario confirma con 'Y'
      Serial.println("Confirma con 'Y' para proceder: ");
      unsigned long start = millis();
      while (millis() - start < 5000) {
        if (Serial.available()) {
          char y = Serial.read();
          if (y == 'Y') {
            Serial.println("Iniciando calibración...");
            // Calibración: MAX (2000us) 3s -> MIN (1000us) 3s
            escWriteMicroseconds(2000);
            delay(3000);
            escWriteMicroseconds(1000);
            delay(3000);
            Serial.println("Calibración finalizada. Volviendo a 1000us.");
            break;
          } else {
            Serial.println("Calibración cancelada.");
            break;
          }
        }
      }
    } else if (c == 'd') {
      // toggle debug printing of escWriteMicroseconds
      static bool escDebug = false;
      escDebug = !escDebug;
      Serial.print("ESC debug "); Serial.println(escDebug ? "ON" : "OFF");
    }
  }
  // Send CAN message at regular intervals
  if (millis() - prevTX >= invlTX) {
    prevTX = millis();
    sendCANMessage();
  }
}

// Test de diagnóstico: hace un sweep de pulsos para verificar que el ESC responde.
// Este test sólo se ejecuta si se pulsa 't' desde el monitor serie.
void escTestSweep() {
  Serial.println("[ESC TEST] Enviando 1000us (min) - esperar 2s");
  escWriteMicroseconds(1000);
  delay(2000);

  Serial.println("[ESC TEST] Enviando 1300us (avance suave) - 2s");
  escWriteMicroseconds(1300);
  delay(2000);

  Serial.println("[ESC TEST] Enviando 1700us (mas fuerte) - 2s");
  escWriteMicroseconds(1700);
  delay(2000);

  Serial.println("[ESC TEST] Enviando 2000us (max) - 2s -> ojo: puede girar a maxima potencia");
  escWriteMicroseconds(2000);
  delay(2000);

  Serial.println("[ESC TEST] Volviendo a 1000us (min) - 2s");
  escWriteMicroseconds(1000);
  delay(2000);

  Serial.println("[ESC TEST] Finalizado");
}

void receiveCANMessage() {
  if (CAN0.readMsgBuf(&rxId, &len, rxBuf) == CAN_OK) {
    receivedCount++;
    Serial.print("RECV #"); Serial.print(receivedCount);
    Serial.print(" | ID: 0x"); Serial.print(rxId, HEX);
    Serial.print(" | Len: "); Serial.print(len);
    Serial.print(" | Data: ");
    for (int i = 0; i < len; i++) {
      if (rxBuf[i] < 0x10) Serial.print("0");
      Serial.print(rxBuf[i], HEX);
      Serial.print(" ");
    }
    Serial.print("| ASCII: ");
    for (int i = 0; i < len; i++) {
      if (rxBuf[i] >= 32 && rxBuf[i] <= 126) Serial.print((char)rxBuf[i]);
      else Serial.print(".");
    }
    Serial.println();
  }
}

void sendCANMessage() {
  if (!canInitialized) return;

  // ----- Snapshot atómico del contador -----
  int32_t cnt;
  noInterrupts();
  cnt = encoderCount;
  interrupts();

  // ----- Calcular distancia -----
  float revs = (float)cnt / pulsesPerRevolution;
  float distance_cm = revs * wheelCircumference_cm;

  // ----- Empaquetar (float + int32) -----
  uint8_t payload[8];
  memcpy(&payload[0], &distance_cm, sizeof(float));  // bytes 0..3
  memcpy(&payload[4], &cnt,         sizeof(int32_t)); // bytes 4..7

  // ----- Enviar por CAN (tu librería: 4 parámetros) -----
  const unsigned long messageId = 0x30;   // ID de “distancia”
  const byte ext = 0;                      // ID estándar (11-bit)
  const byte l   = 8;                      // DLC=8
  byte sndStat = CAN0.sendMsgBuf(messageId, ext, l, payload);
  lastDistanceCm = distance_cm;

  if (sndStat == CAN_OK) {
    Serial.print("TX  ID: 0x"); Serial.print(messageId, HEX);
    Serial.print("  dist=");     Serial.print(distance_cm, 2);
    Serial.print(" cm  cnt=");   Serial.println((long)cnt);
  } else {
    Serial.println("ERROR: Failed to send message");
  }

  // Demo: control local del ESC con la distancia calculada (puedes quitarlo luego)
  if (lastDistanceCm >= 100.0f) {
    escWriteMicroseconds(1000);   // STOP
  } else {
    escWriteMicroseconds(1300);   // avance suave (ajusta a tu ESC)
  }
}


/*********************************************************************************************************
  END FILE
*********************************************************************************************************/
