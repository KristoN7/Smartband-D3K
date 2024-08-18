// Sensors data
#include <Arduino.h>
#include <Wire.h>
#include <MAX30105.h>
#include <MPU6050.h>

//MicroSD
#include <SPI.h>
#include <SD.h>

// Wifi i BLE
#include <WiFi.h>
#include <NimBLEDevice.h>
#include <NimBLEUtils.h>
#include <NimBLEServer.h>

#define LED_PIN_IDLE 15
#define LED_PIN_BLE 2
#define LED_PIN_TRAINING 4
#define BUTTON_PIN 13

volatile int currentState = 0;
volatile bool stateChanged = false;

// UUIDs for BLE Service and Characteristics
#define SERVICE_UUID        "12345678-1234-1234-1234-123456789012"
#define SSID_CHAR_UUID      "e2e3f5a4-8c4f-11eb-8dcd-0242ac130003"
#define PASSWORD_CHAR_UUID  "e2e3f5a4-8c4f-11eb-8dcd-0242ac130004"

//SD Card
#define CS_PIN 5

NimBLEServer* pServer = nullptr;
NimBLEService* pService = nullptr; 
NimBLEAdvertising* pAdvertising = nullptr; 
NimBLECharacteristic* pSsidCharacteristic = nullptr;
NimBLECharacteristic* pPasswordCharacteristic = nullptr;
bool deviceConnected = false;

String ssid = "";
String password = "";

MAX30105 particleSensor;
MPU6050 mpu;

File dataFile;
char filename[32];
int fileIndex = 1;

const int debounceDelay = 2000; // to eliminate debouncing effect on physical switch (ms)
volatile unsigned long lastDebounceTime = 0;

//FUNCTIONS

void IRAM_ATTR handleButtonPress() {
    unsigned long currentTime = millis();
    if((currentTime - lastDebounceTime) > debounceDelay){
        currentState = (currentState + 1) % 3; // Possible states: 0 - idle, 1 - BLE, 2 - exercise 
        stateChanged = true;
        lastDebounceTime = currentTime;
    }
}

void startTraining() {

    // search for first available name for file
    do {
        snprintf(filename, sizeof(filename), "/training_%d.txt", fileIndex++);
    } while (SD.exists(filename));
    
    File dataFile = SD.open(filename, FILE_WRITE);
    if (!dataFile) {
        Serial.println("Failed to open file for writing");
        return;
    }

    Serial.print("Training data will be saved to: ");
    Serial.println(filename);

    dataFile.println("IR, Red, Ax, Ay, Az, Gx, Gy, Gz");
    dataFile.close();
}

void connectToWiFi() {
    Serial.print("Connecting to WiFi with SSID: ");
    Serial.println(ssid);
    Serial.print("Password: ");
    Serial.println(password);
    
    WiFi.disconnect(true); // Reset WiFi
    delay(1000);

    WiFi.begin(ssid.c_str(), password.c_str());

    int retries = 0;
    while (WiFi.status() != WL_CONNECTED && retries < 20) {
        delay(500);
        Serial.print(".");
        retries++;
    }

    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("\nConnected to WiFi!");
        Serial.print("IP Address: ");
        Serial.println(WiFi.localIP());
    } else {
        Serial.println("\nFailed to connect to WiFi.");
    }
}

class ServerCallbacks: public NimBLEServerCallbacks {
    void onConnect(NimBLEServer* pServer) override {
        deviceConnected = true;
        Serial.println("Client connected.");
    };

    void onDisconnect(NimBLEServer* pServer) override {
        deviceConnected = false;
        Serial.println("Client disconnected.");
    }
};

class CharacteristicCallbacks: public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic* pCharacteristic) override {
        Serial.println("Attempting to write new credentials.");
        std::string uuid = pCharacteristic->getUUID().toString();
        Serial.print("UUID: ");
        Serial.println(uuid.c_str());

        String value = pCharacteristic->getValue().c_str();
        Serial.print("Value Length: ");
        Serial.println(value.length());
        Serial.print("Value: ");
        for (char c : value) {
            Serial.print(c);
            Serial.print(" ");
        }
        Serial.println();

        if (uuid == SSID_CHAR_UUID) {
            ssid = value;
            Serial.print("Received SSID: ");
            Serial.println(ssid.c_str());
        } else if (uuid == PASSWORD_CHAR_UUID) {
            password = value;
            Serial.print("Received Password: ");
            Serial.println(password.c_str());
            // Attempt to connect to WiFi
            connectToWiFi();
        }
    }
};


void setup() {
    Serial.begin(115200);
    Serial.println("Starting setup...");

    Wire.begin();

    pinMode(LED_PIN_IDLE, OUTPUT);
    pinMode(LED_PIN_BLE, OUTPUT);
    pinMode(LED_PIN_TRAINING, OUTPUT);
    pinMode(BUTTON_PIN, INPUT);

    attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), handleButtonPress, RISING);

    // The initial LED state 
    digitalWrite(LED_PIN_IDLE, LOW);
    digitalWrite(LED_PIN_BLE, LOW);
    digitalWrite(LED_PIN_TRAINING, LOW);

    // MAX30102 initialization
    if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
        Serial.println("MAX30102 not found. Check wiring.");
        while (1);
    }
    particleSensor.setup(); // Configure sensor with default settings
    particleSensor.setPulseAmplitudeRed(0x0A); // Turn Red LED to low to indicate sensor is running
    particleSensor.setPulseAmplitudeIR(0x0A); // Turn IR LED to low to indicate sensor is running

    // MPU6050 initialization
    mpu.initialize();
    if (!mpu.testConnection()) {
        Serial.println("MPU6050 connection failed. Check wiring.");
        while (1);
    }

    // SD card module initalization
    while (!SD.begin(CS_PIN)) {
        Serial.println("Card Mount Failed");
    }

    uint8_t cardType = SD.cardType();
    if (cardType == CARD_NONE) {
        Serial.println("No SD card attached");
        return;
    }

     Serial.println("SD Card initialized.");

     Serial.print("SD Card Type: ");
    if (cardType == CARD_MMC) {
        Serial.println("MMC");
    } else if (cardType == CARD_SD) {
        Serial.println("SDSC");
    } else if (cardType == CARD_SDHC) {
        Serial.println("SDHC");
    } else {
        Serial.println("UNKNOWN");
    }

    uint64_t cardSize = SD.cardSize() / (1024 * 1024);
    Serial.printf("SD Card Size: %lluMB\n", cardSize);

    // Header
    dataFile.println("IR, Red, Ax, Ay, Az, Gx, Gy, Gz");
    dataFile.close();
}

void loop() {
    if (stateChanged) {
        stateChanged = false;
        switch (currentState) {
            case 0: // Idle state, no data are collected from sensors
                digitalWrite(LED_PIN_IDLE, HIGH);
                digitalWrite(LED_PIN_BLE, LOW);
                digitalWrite(LED_PIN_TRAINING, LOW);
                
                // Disable all devices
                NimBLEDevice::deinit(true);
                pServer = nullptr;
                pService = nullptr;
                pAdvertising = nullptr;

                SD.end();
                
                // To do: disable MAX30102 and MPU6050

                Serial.println("Idle mode");
                break;

            case 1: // BLE state, BLE server is on, should broadcast
                digitalWrite(LED_PIN_IDLE, LOW);
                digitalWrite(LED_PIN_BLE, HIGH);
                digitalWrite(LED_PIN_TRAINING, LOW);

                SD.end();
                // To do: connect mobile phone to ESP32 during this state

                    if (pAdvertising->isAdvertising() == false or pAdvertising == nullptr or pServer == nullptr or pService == nullptr) {
                    
                    NimBLEDevice::init("ESP32_Smartband");

                    pServer = NimBLEDevice::createServer();
                    pServer->setCallbacks(new ServerCallbacks());

                    pService = pServer->createService(SERVICE_UUID);

                    pSsidCharacteristic = pService->createCharacteristic(
                                            SSID_CHAR_UUID,
                                            NIMBLE_PROPERTY::WRITE
                                        );
                    pSsidCharacteristic->setCallbacks(new CharacteristicCallbacks());

                    pPasswordCharacteristic = pService->createCharacteristic(
                                                PASSWORD_CHAR_UUID,
                                                NIMBLE_PROPERTY::WRITE
                                            );
                    pPasswordCharacteristic->setCallbacks(new CharacteristicCallbacks());

                    pService->start();

                    pAdvertising = NimBLEDevice::getAdvertising();
                    pAdvertising->addServiceUUID(SERVICE_UUID);
                    pAdvertising->start();
                    }

                    Serial.println("BLE mode activated and advertising started");
                break;

            case 2: // Exercise state, collect data from sensors
                digitalWrite(LED_PIN_IDLE, LOW);
                digitalWrite(LED_PIN_BLE, LOW);
                digitalWrite(LED_PIN_TRAINING, HIGH);

                NimBLEDevice::deinit(true); 
                NimBLEDevice::deinit(true);
                pServer = nullptr;
                pService = nullptr;
                pAdvertising = nullptr;



                if (!SD.begin(CS_PIN)) {
                    Serial.println("Card Mount Failed");
                } else {
                    Serial.println("Training mode - SD logging started");
                    //To do, check if data is collected
                    startTraining();
                }
                break;

            default:
                digitalWrite(LED_PIN_IDLE, LOW);
                digitalWrite(LED_PIN_BLE, LOW);
                digitalWrite(LED_PIN_TRAINING, LOW);
                //To do, adjust state 4
                break;
        }
    }

    if(currentState == 2){
        // Fetch data from MAX30102 sensor
        long irValue = particleSensor.getIR();
        long redValue = particleSensor.getRed();

        // Fetch data from MPU6050 sensor
        int16_t ax, ay, az;
        int16_t gx, gy, gz;
        mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

        // Serial print the raw data
        Serial.print("IR: ");
        Serial.print(irValue);
        Serial.print("\tRed: ");
        Serial.print(redValue);
        Serial.print("\tAx: ");
        Serial.print(ax);
        Serial.print("\tAy: ");
        Serial.print(ay);
        Serial.print("\tAz: ");
        Serial.print(az);
        Serial.print("\tGx: ");
        Serial.print(gx);
        Serial.print("\tGy: ");
        Serial.print(gy);
        Serial.print("\tGz: ");
        Serial.println(gz);

        dataFile = SD.open(filename, FILE_APPEND);
        if (dataFile) {
            // Zapis danych z czujników do pliku
            dataFile.print(irValue);
            dataFile.print(", ");
            dataFile.print(redValue);
            dataFile.print(", ");
            dataFile.print(ax);
            dataFile.print(", ");
            dataFile.print(ay);
            dataFile.print(", ");
            dataFile.print(az);
            dataFile.print(", ");
            dataFile.print(gx);
            dataFile.print(", ");
            dataFile.print(gy);
            dataFile.print(", ");
            dataFile.print(gz);
            dataFile.println();
            dataFile.close();
            Serial.println("Data written to SD card");
        } else {
            Serial.println("Failed to open file for appending");
        }

        delay(10); // Adjust delay as needed
    }
}