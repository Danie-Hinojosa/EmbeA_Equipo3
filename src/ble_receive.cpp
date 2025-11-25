#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

float receivedX = 0.0;
float receivedY = 0.0;
bool newData = false; 

BLEServer* pServer = NULL;
BLECharacteristic* pCharacteristic = NULL;
bool deviceConnected = false;
bool oldDeviceConnected = false;

// Obtained with a scanner
#define SERVICE_UUID        "91BAD492-B950-4226-AA2B-4EDE9FA42F59"  
#define CHARACTERISTIC_UUID "CBA1D466-344C-4BE3-AB3F-189F80DD7518"  

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
      Serial.println("Device Connected");
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
      Serial.println("Device Disconnected");
    }
};

// CALLBACK TO RECEIVE DATA
class MyCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      std::string value = pCharacteristic->getValue();

      if (value.length() > 0) {
        const char* charData = value.c_str();
        
        // Parse the data expecting format "x,y" (e.g., "12.5,100")
        float tempX, tempY;
        int items = sscanf(charData, "%f,%f", &tempX, &tempY);

        if (items == 2) {
           receivedX = tempX;
           receivedY = tempY;
           newData = true; 
        } else {
           Serial.println("Error: Data format incorrect. Send 'x,y'");
        }
      }
    }
};

void setup() {
  Serial.begin(115200);

  // Create the BLE Device
  BLEDevice::init("ESP32_XY_Receiver");

  // Create the BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Create a BLE Characteristic
  pCharacteristic = pService->createCharacteristic(
                      CHARACTERISTIC_UUID,
                      BLECharacteristic::PROPERTY_READ   |
                      BLECharacteristic::PROPERTY_WRITE  |
                      BLECharacteristic::PROPERTY_NOTIFY
                    );

  // Attach the callback to handle writes
  pCharacteristic->setCallbacks(new MyCallbacks());
  
  // Create a BLE Descriptor (needed for notifications usually, good practice)
  pCharacteristic->addDescriptor(new BLE2902());

  // Start the service
  pService->start();

  // Start advertising
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(false);
  pAdvertising->setMinPreferred(0x0);
  BLEDevice::startAdvertising();
  Serial.println("Waiting for a client to send coordinates...");
}

void loop() {
    if (newData) {
        Serial.print("New Coordinates Saved -> X: ");
        Serial.print(receivedX);
        Serial.print(" | Y: ");
        Serial.println(receivedY);
        
        newData = false;

        // Aqui es donde enviamos al CAN
        

    }

    // handle disconnection logic to restart advertising ensures stability
    if (!deviceConnected && oldDeviceConnected) {
        delay(500); 
        pServer->startAdvertising();
        Serial.println("start advertising");
        oldDeviceConnected = deviceConnected;
    }
    // connecting
    if (deviceConnected && !oldDeviceConnected) {
        oldDeviceConnected = deviceConnected;
    }
}