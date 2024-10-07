#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <ESP32Servo.h> 

// BLE
BLEServer* pServer = NULL;
BLECharacteristic* pSensorCharacteristic = NULL;
BLECharacteristic* pLedCharacteristic = NULL;
bool deviceConnected = false;
bool oldDeviceConnected = false;
uint32_t bleValue = 0;

#define SERVICE_UUID                "b01619ee-78e0-49d1-8909-f98bddf7014a"
#define SENSOR_CHARACTERISTIC_UUID  "1ccf8174-5eb0-415b-9219-d68278c1babb"
#define LED_CHARACTERISTIC_UUID     "7d6a521d-9942-49d0-ae65-10d04730aa8c"

class MyServerCallbacks: public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    deviceConnected = true;
  };

  void onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
  }
};

class MyCharacteristicCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* pCharacteristic) {
    std::string ledValue = pCharacteristic->getValue(); 
    if (ledValue.length() > 0) {
      String valueStr = String(ledValue.c_str());  
      Serial.print("BLE Characteristic written: ");
      Serial.println(static_cast<int>(valueStr[0]));
      int receivedValue = static_cast<int>(valueStr[0]);
      if (receivedValue == 1) {
        digitalWrite(LED_BUILTIN, HIGH);
      } else {
        digitalWrite(LED_BUILTIN, LOW);
      }
    }
  }
};

// LED
#define RED_PIN     D2
#define GREEN_PIN   D3
#define BLUE_PIN    D4

// SERVO
Servo myServo;
int potpin = 0;
int valServo = 0;

// BUTTON
#define BUTTON_PIN  D7  

// TEMPERATUUR
#define TEMP_PIN    A0  
int temperatuur = 0;

unsigned long previousTempMillis = 0;
const unsigned long tempInterval = 5000; 
const unsigned long debounceDelay = 50;

bool buttonPressed = false;
bool lastButtonState = LOW;
unsigned long lastDebounceTime = 0;

void meetTemperatuur(){
  uint16_t val = analogRead(TEMP_PIN);
  double voltage = (double)val * (5.0 / 1024.0); 
  double tempC = voltage * 100.0;
  Serial.print("TEMPERATUUR: ");
  Serial.print(tempC);
  Serial.println(" C");
  temperatuur = (int)tempC;
}

void zetLedAanPinken(){
  for (int i = 0; i < 5; i++) {
    analogWrite(RED_PIN, 255);
    analogWrite(GREEN_PIN, 0);
    analogWrite(BLUE_PIN, 255);
    delay(500);
    analogWrite(RED_PIN, 0);
    analogWrite(GREEN_PIN, 255);
    analogWrite(BLUE_PIN, 0);
    delay(500);
  }
}

void deurenHelft(){
  analogWrite(RED_PIN, 0);
  analogWrite(GREEN_PIN, 255);
  analogWrite(BLUE_PIN, 0);
  valServo = 90;
  myServo.write(valServo);
  Serial.println(valServo);
}

void deurenGesloten(){
  analogWrite(RED_PIN, 255);
  analogWrite(GREEN_PIN, 0);
  analogWrite(BLUE_PIN, 0);
  valServo = 0;
  myServo.write(valServo);
  Serial.println(valServo);
}

void setup() {

  Serial.begin(9600);

  BLEDevice::init("ESP32_Frans_Kenzo");
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService *pService = pServer->createService(SERVICE_UUID);

  pSensorCharacteristic = pService->createCharacteristic(
                        SENSOR_CHARACTERISTIC_UUID,
                        BLECharacteristic::PROPERTY_READ   |
                        BLECharacteristic::PROPERTY_WRITE  |
                        BLECharacteristic::PROPERTY_NOTIFY |
                        BLECharacteristic::PROPERTY_INDICATE
                      );

  pLedCharacteristic = pService->createCharacteristic(
                        LED_CHARACTERISTIC_UUID,
                        BLECharacteristic::PROPERTY_WRITE
                      );


  pLedCharacteristic->setCallbacks(new MyCharacteristicCallbacks());


  pSensorCharacteristic->addDescriptor(new BLE2902());
  pLedCharacteristic->addDescriptor(new BLE2902());

  pService->start();

  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(false);
  pAdvertising->setMinPreferred(0x0); 
  BLEDevice::startAdvertising();
  Serial.println("Waiting for a BLE client connection to notify...");

  pinMode(LED_BUILTIN, OUTPUT);

  pinMode(RED_PIN, OUTPUT);
  pinMode(GREEN_PIN, OUTPUT);
  pinMode(BLUE_PIN, OUTPUT);

  myServo.attach(D6);

  pinMode(BUTTON_PIN, INPUT);

  pinMode(TEMP_PIN, INPUT);
}

void loop() {
  if (deviceConnected) {
    String sensorValueStr = String(temperatuur); 
    pSensorCharacteristic->setValue(sensorValueStr.c_str());
    pSensorCharacteristic->notify();
    Serial.print("BLE Notified Sensor Value: ");
    Serial.println(sensorValueStr);
    bleValue++;
    delay(3000);
  }


  if (!deviceConnected && oldDeviceConnected) {
    Serial.println("BLE Device disconnected.");
    delay(500); 
    pServer->startAdvertising();
    Serial.println("BLE Advertising restarted.");
    oldDeviceConnected = deviceConnected;
  }

  
  if (deviceConnected && !oldDeviceConnected) {
    
    oldDeviceConnected = deviceConnected;
    Serial.println("BLE Device Connected.");
  }

  
  bool currentButtonState = digitalRead(BUTTON_PIN);

  if (currentButtonState != lastButtonState) {
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (currentButtonState != buttonPressed) {
      buttonPressed = currentButtonState;

      if (buttonPressed == HIGH) {
        Serial.println("NOOD BUTTON INGEDRUKT | DEUREN OPENEN VOLLEDIG!");
        valServo = 180;
        myServo.write(valServo);
        Serial.println(valServo);
        zetLedAanPinken();
      }
    }
  }

  lastButtonState = currentButtonState;

  unsigned long currentMillis = millis();

  if (currentMillis - previousTempMillis >= tempInterval) {
    previousTempMillis = currentMillis;

    meetTemperatuur();

    if (temperatuur >= 100) {
      Serial.println("TEMPERATUUR BOVEN 100°C | DEUREN WORDEN GEOPEND!");
      deurenHelft();
    } else {
      Serial.println("TEMPERATUUR ONDER 100°C | DEUREN BLIJVEN GESLOTEN!");
      deurenGesloten();
    }
  }
}
