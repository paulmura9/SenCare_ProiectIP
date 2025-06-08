#include <DHT.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>

#define DHTPIN 23
#define DHTTYPE DHT11
#define PULSE_PIN 36
#define ECG_PIN 34
#define LO_MINUS_PIN 35
#define LO_PLUS_PIN 32
#define THRESHOLD 2000
#define LED_BUILTIN 2 

DHT dht(DHTPIN, DHTTYPE);

#define SERVICE_UUID        "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
#define CHARACTERISTIC_UUID "6e400003-b5a3-f393-e0a9-e50e24dcca9e"
BLECharacteristic* pCharacteristic;
bool deviceConnected = false;

class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    deviceConnected = true;
  }
  void onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
    pServer->startAdvertising();
  }
};


int bpmValue = 0;
int peakValue = 0;
int previousValue = 0;
int beatCount = 0;
unsigned long lastPulseRead = 0;
int pulseReadCounter = 0;
bool pulseDetected = false;

void Pulse_readValue() {
  if (millis() - lastPulseRead >= 100) {
    pulseReadCounter++;
    lastPulseRead = millis();
    int currentValue = analogRead(PULSE_PIN);

    if (currentValue >= previousValue) {
      peakValue = currentValue;
      pulseDetected = false;
    } else {
      if (peakValue >= THRESHOLD && !pulseDetected) {
        pulseDetected = true;
        beatCount++;
      }
      peakValue = 0;
    }

    previousValue = currentValue;

    if (pulseReadCounter >= 40) {
      pulseReadCounter = 0;
      bpmValue = beatCount * 6;
      beatCount = 0;
    }
  }
}



#define VREF 3.3
#define ADC_MAX 4095
#define EKG_BUFFER_SIZE 8

float buffer[EKG_BUFFER_SIZE];
int bufferIndex = 0;
float emaFiltered = 0.0;
const float alpha = 0.15;

bool ekgCalibrated = false;
unsigned long calibrationStart = 0;

int getFilteredEkgMv() {
  if (digitalRead(LO_MINUS_PIN) == HIGH || digitalRead(LO_PLUS_PIN) == HIGH) {
    return -1;
  }

  int raw = analogRead(ECG_PIN);

  if (!ekgCalibrated) {
    if (millis() - calibrationStart < 2000) {
      return -1;
    } else {
      ekgCalibrated = true;
    }
  }

  float voltage = (raw * VREF) / ADC_MAX;
  float centered = voltage - (VREF / 2.0);
  float mv = centered * 1000.0;

  buffer[bufferIndex] = mv;
  bufferIndex = (bufferIndex + 1) % EKG_BUFFER_SIZE;

  float sum = 0.0;
  for (int i = 0; i < EKG_BUFFER_SIZE; i++) {
    sum += buffer[i];
  }
  float avgMv = sum / EKG_BUFFER_SIZE;

  emaFiltered = alpha * avgMv + (1.0 - alpha) * emaFiltered;

  return (int)emaFiltered;
}

// ------------ R-peak detection -------------
#define R_PEAK_THRESHOLD_MV 400
#define R_PEAK_MIN_INTERVAL_MS 300
unsigned long lastRpeakTime = 0;
unsigned long ledOnTime = 0;

void checkRpeak(int ekgMv) {
  if (ekgMv > R_PEAK_THRESHOLD_MV) {
    unsigned long now = millis();
    if (now - lastRpeakTime > R_PEAK_MIN_INTERVAL_MS) {
      lastRpeakTime = now;
      digitalWrite(LED_BUILTIN, HIGH);
      ledOnTime = now;
      Serial.println("R-peak detected!");
    }
  }

  if (digitalRead(LED_BUILTIN) == HIGH && millis() - ledOnTime >= 10) {
    digitalWrite(LED_BUILTIN, LOW);
  }
}


void setup() {
  Serial.begin(115200);
  dht.begin();
  pinMode(PULSE_PIN, INPUT);
  pinMode(ECG_PIN, INPUT);
  pinMode(LO_MINUS_PIN, INPUT);
  pinMode(LO_PLUS_PIN, INPUT);
  pinMode(LED_BUILTIN, OUTPUT);

  calibrationStart = millis();

  BLEDevice::init("ESP32_BLE_JSON");
  BLEServer* pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService* pService = pServer->createService(SERVICE_UUID);
  pCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_NOTIFY
  );
  pCharacteristic->addDescriptor(new BLE2902());
  pService->start();
  pServer->getAdvertising()->start();

  Serial.println("BLE started. Waiting for client...");
}


void loop() {
  Pulse_readValue();

  static unsigned long lastEcgTime = 0;
  if (millis() - lastEcgTime >= 50 && deviceConnected) {
    bool leadsOff = digitalRead(LO_MINUS_PIN) == HIGH || digitalRead(LO_PLUS_PIN) == HIGH;

    if (leadsOff) {
      digitalWrite(LED_BUILTIN, HIGH);  
      pCharacteristic->setValue("!");
      pCharacteristic->notify();
      Serial.println("!");
    } else {
      int filteredMv = getFilteredEkgMv();
      if (filteredMv != -1) {
        checkRpeak(filteredMv);
        String json = "{\"ekg\":" + String(filteredMv) + "}";
        pCharacteristic->setValue(json.c_str());
        pCharacteristic->notify();
        Serial.println(filteredMv);
      }
    }

    lastEcgTime = millis();
  }

  static unsigned long lastSampleTime = 0;
  if (millis() - lastSampleTime >= 10000 && deviceConnected) {
    float humidity = dht.readHumidity();
    float temp = dht.readTemperature();

    String tenSecJson = "{\"bpm\":" + String(bpmValue) +
                        ",\"temp\":" + String(temp) +
                        ",\"hum\":" + String(humidity) + "}";
    pCharacteristic->setValue(tenSecJson.c_str());
    pCharacteristic->notify();
    Serial.println("[10s] ➜ " + tenSecJson);

    lastSampleTime = millis();
  }
}