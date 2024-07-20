// Sensors data
#include <Arduino.h>
#include <Wire.h>
#include <MAX30105.h>
#include <MPU6050.h>

// Wifi
#include <WiFi.h>
#include <NimBLEDevice.h>
#include <NimBLEUtils.h>
#include <NimBLEServer.h>

// UUIDs for BLE Service and Characteristics
#define SERVICE_UUID        "12345678-1234-1234-1234-123456789012"
#define SSID_CHAR_UUID      "e2e3f5a4-8c4f-11eb-8dcd-0242ac130003"
#define PASSWORD_CHAR_UUID  "e2e3f5a4-8c4f-11eb-8dcd-0242ac130004"

NimBLEServer* pServer = nullptr;
NimBLECharacteristic* pSsidCharacteristic = nullptr;
NimBLECharacteristic* pPasswordCharacteristic = nullptr;
bool deviceConnected = false;

std::string ssid;
std::string password;

MAX30105 particleSensor;
MPU6050 mpu;

void connectToWiFi() {
    Serial.print("Connecting to WiFi with SSID: ");
    Serial.println(ssid.c_str());

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

        std::string value = pCharacteristic->getValue();
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

    Serial.println("Starting BLE work!");

    // Initialize BLE
    NimBLEDevice::init("ESP32_BLE_WiFi_Provisioning");

    // Create BLE Server
    pServer = NimBLEDevice::createServer();
    pServer->setCallbacks(new ServerCallbacks());

    // Create BLE Service
    NimBLEService* pService = pServer->createService(SERVICE_UUID);

    // Create BLE Characteristics for SSID and Password
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

    // Start the service
    pService->start();
    Serial.println("BLE service started.");

    // Start advertising
    NimBLEAdvertising* pAdvertising = NimBLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->start();
    Serial.println("Waiting for a client connection to notify...");
}

void loop() {
  /*
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

    delay(1000); // Adjust delay as needed
    */
}
