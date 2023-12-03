
#include <NimBLEDevice.h>
#include "Wire.h"
#include <MPU6050_light.h>
#include <DFRobot_QMC5883.h>
#include <esp_sleep.h>

#define LED D10


DFRobot_QMC5883 compass(&Wire, /*I2C addr*/QMC5883_ADDRESS);
MPU6050 mpu(Wire);
struct SensorData {
  int16_t ax;
  int16_t ay;
  int16_t az;
  int16_t h;
  int16_t v;
};

BLEServer *pServer = NULL;
BLECharacteristic * Ch_SendData;
BLECharacteristic * Ch_SendStatus;
volatile bool deviceConnected = false;
//нужно настроить реконнект с устройством!
bool oldDeviceConnected = false;
bool deviceMooving = false;
uint8_t txValue = 0;
//в RTC память 
RTC_DATA_ATTR bool sleepPhase1 = false;
RTC_DATA_ATTR bool wait_connection = true;

const int MAX_STATUS_LENGTH = 50;
char statusMessage[MAX_STATUS_LENGTH];

//Переменные для контроля работы устройства
unsigned long sleepTimer = 0;
const unsigned long AFTER_SLEEP_TIMER = 5000; // 5 секунд
//во время вещание следим за движением
const unsigned long SLEEP_TIMEOUT_CONNECTED = 50000; // 50 секунд

//короткие и долгие режимы сна
const unsigned long SLEEP_TIMEOUT_SHORT = 20; // 20 секунд
const unsigned long SLEEP_TIMEOUT_LONG = 150; // 150 секунд
//если к устройству ничего не подключено
const unsigned long SLEEP_TIMEOUT_DISCONNECTED = 30000; // 30 секунд

#define SERVICE_UUID "6E400001-B5A3-F393-E0A9-E50E24DCCA9A"
#define UUID_Reboot "6E400002-B5A3-F393-E0A9-E50E24DCCA9B"
#define UUID_SendData "6E400003-B5A3-F393-E0A9-E50E24DCCA9C"
#define UUID_SendStatus "6E400004-B5A3-F393-E0A9-E50E24DCCA9D"

void startDeepSleepSchort() {
    // Установка таймера сна
    esp_sleep_enable_timer_wakeup(SLEEP_TIMEOUT_SHORT * 1000000);
    // Сохраняем shouldWakeUp в RTC памяти
    // Вход в глубокий сон
    blinkLong(1);
    sleepPhase1 = true;
    esp_deep_sleep_start();
}
void startDeepSleep() {
    // Установка таймера сна
    esp_sleep_enable_timer_wakeup(SLEEP_TIMEOUT_LONG * 1000000);
    // Сохраняем shouldWakeUp в RTC памяти
    // Вход в глубокий сон
    esp_deep_sleep_start();
}

//Следим за тем что бы было движение
void motionTracking(float tx,float ty,float tz) {
  if (abs(tx) > 40 || abs(ty) > 40 || abs(tz) > 40) {
      // Если обнаружено движение
      sleepTimer = millis();
  }
}

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
      Serial.println("Device Connected");
    }

    void onDisconnect(BLEServer* pServer) {
      Serial.println("This whyyyyy???????");
      ESP.restart();
    }
  
};

class MyCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      std::string rxValue = pCharacteristic->getValue();
       if (rxValue.length() > 0) {
        Serial.println("*********");
        Serial.print("Received Value: ");
        for (int i = 0; i < rxValue.length(); i++)
          Serial.print(rxValue[i]);

        Serial.println();
        Serial.println("*********");
      }
      if (rxValue == "R") {
        ESP.restart(); // Перезагрузка устройства
      }
    }
};

void startBLE(){
  // Create the BLE Device
  BLEDevice::init("CompasLeft");

  // Create the BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Create a BLE Characteristic
  Ch_SendData = pService->createCharacteristic(
                                        UUID_SendData,
                                    /******* Enum Type NIMBLE_PROPERTY now *******      
                                        BLECharacteristic::PROPERTY_NOTIFY
                                        );
                                    **********************************************/  
                                        NIMBLE_PROPERTY::NOTIFY
                                       );

  BLECharacteristic * Ch_GetData = pService->createCharacteristic(
                                            UUID_Reboot,
                                    /******* Enum Type NIMBLE_PROPERTY now *******       
                                            BLECharacteristic::PROPERTY_WRITE
                                            );
                                    *********************************************/  
                                            NIMBLE_PROPERTY::WRITE
                                            );

  Ch_SendStatus = pService->createCharacteristic(
                                        UUID_SendStatus,
                                    /******* Enum Type NIMBLE_PROPERTY now *******      
                                        BLECharacteristic::PROPERTY_NOTIFY
                                        );
                                    **********************************************/  
                                        NIMBLE_PROPERTY::NOTIFY
                                       );                         

  Ch_GetData->setCallbacks(new MyCallbacks());

  // Start the service
  pService->start();

  // Start advertising
  pServer->start();
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);  // functions that help with iPhone connections issue
  pAdvertising->setMaxPreferred(0x12);

  BLEDevice::startAdvertising();
}

void setup() {
  Serial.begin(115200);
  Wire.begin(6,7);
  while (!compass.begin())
  {
    Serial.println("Could not find a valid 5883 sensor, check wiring!");
    delay(500);
  }
  pinMode(LED, OUTPUT);
  digitalWrite(LED, LOW);
  byte status = mpu.begin();
  startBLE();
  
}
//wait_connection при старте true
void loop() {
  if(wait_connection == true && deviceConnected == false){
    unsigned long startTime = millis();
    blinkNormal(5);
    while (!deviceConnected && millis() - startTime < 20000) {
      checkBLEConnection();
    }
    if (!deviceConnected) {
      //устройство не подключено в течение 10 секунд
      wait_connection = false;
      startDeepSleepSchort();
    }
  }
  if (deviceConnected) {
    sendData();
    delay(200);
  // Режим, когда устройство не подключено по BLE
  }
  //режим в коротком сне
  if(sleepPhase1){
    mpu.update();
    float tx = mpu.getGyroX();
    float ty = mpu.getGyroY();
    float tz = mpu.getGyroZ();
    //disableNonBLE();
    //checkBLEConnectionTimeout();
    motionTracking(tx, ty, tz);
    if (millis() - sleepTimer > SLEEP_TIMEOUT_DISCONNECTED) {
      blinkNormal(2);
      //если нет движения в течение SLEEP_TIMEOUT_DISCONNECTED - 30 секунд, войдите в глубокий сон
      startDeepSleepSchort();
    }else{
      blinkNormal(3);
      sleepPhase1 = false;
      wait_connection = true;
    }
    delay(1000);
  }
}

void checkBLEConnection() {

  if (deviceConnected) {
    return;
  } else {
  }
}

void sendData(){
  //мониторим акк
  uint32_t Vbatt = 0;
  for(int i = 0; i < 16; i++) {
    Vbatt = Vbatt + analogReadMilliVolts(A0);
  }
  
  float Vbattf = 2 * Vbatt / 16 / 1000.0;
  
  SensorData sensorData;
  mpu.update();

  float declinationAngle = (4.0 + (26.0 / 60.0)) / (180 / PI);
  compass.setDeclinationAngle(declinationAngle);
  sVector_t mag = compass.readRaw();
  compass.getHeadingDegrees();

  int16_t ax = mpu.getAngleX();
  int16_t ay = mpu.getAngleY();
  int16_t az = mpu.getAngleZ();
  int16_t headingInt = mag.HeadingDegress;
  float tx = mpu.getGyroX();
  float ty = mpu.getGyroY();
  float tz = mpu.getGyroZ();
  sensorData.ax = ax;
  sensorData.ay = ay;
  sensorData.az = az;
  sensorData.h = headingInt;
  sensorData.v = static_cast<int16_t>(round(Vbattf * 100));
  Serial.println(Vbattf, 3);

  Ch_SendData->setValue((uint8_t*)&sensorData, sizeof(sensorData));
  Ch_SendData->notify();
  //слежение за движением если нет движения отключаемся на время SLEEP_TIMEOUT_CONNECTED
  motionTracking(tx, ty, tz);
  if (millis() - sleepTimer > SLEEP_TIMEOUT_CONNECTED) {
      //если нет движения
      startDeepSleepSchort();
  }
}
//отправляем сообщение через характеристику Ch_SendStatus
void sendStatusMessage(const char* message) {
    int messageLength = strlen(message);
    if (messageLength > MAX_STATUS_LENGTH - 1) {
        messageLength = MAX_STATUS_LENGTH - 1;
    }
    strncpy(statusMessage, message, messageLength);
    statusMessage[messageLength] = '\0';

    Ch_SendStatus->setValue((uint8_t*)statusMessage, messageLength);
    Ch_SendStatus->notify();
}
void blinkSlow(int times) {
  for (int i = 0; i < times; i++) {
    digitalWrite(LED, LOW);
    delay(1000);
    digitalWrite(LED, HIGH);
    delay(1000);
    digitalWrite(LED, LOW);
    delay(1000);
  }
}
void blinkFast(int times) {
  for (int i = 0; i < times; i++) {
    digitalWrite(LED, LOW);
    delay(20);
    digitalWrite(LED, HIGH);
    delay(20);
    digitalWrite(LED, LOW);
    delay(20);
  }
}
void blinkNormal(int times) {
  for (int i = 0; i < times; i++) {
    digitalWrite(LED, LOW);
    delay(200);
    digitalWrite(LED, HIGH);
    delay(200);
    digitalWrite(LED, LOW);
    delay(200);
  }
}
void blinkLong(int times) {
  for (int i = 0; i < times; i++) {
    digitalWrite(LED, LOW);
    delay(5000);
    digitalWrite(LED, HIGH);
    delay(5000);
    digitalWrite(LED, LOW);
    delay(5000);
  }
}