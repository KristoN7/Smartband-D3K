// Sensors data
#include <Arduino.h>
#include <Wire.h>
#include <MAX30105.h>
#include <MPU6050.h>

//MicroSD (using SdFat instead of SD)
#include <SPI.h>
#include <SdFat.h>
#include <SD.h> 

// Wifi and BLE
#include <WiFi.h>
#include <NimBLEDevice.h>
#include <NimBLEUtils.h>
#include <NimBLEServer.h>
#include <HTTPClient.h>
#include <Update.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

//Working states 
#define LED_PIN_IDLE 15
#define LED_PIN_BLE 2
#define LED_PIN_TRAINING 4
#define BUTTON_PIN 13
#define BATTERY_PIN 34  // ADC pin connected to the voltage divider

//Error codes
#define ERROR_SD_NOT_FOUND 1
#define ERROR_SD_ERROR 2
#define ERROR_SENSOR_MPU6050 3
#define ERROR_SENSOR_MAX30102 4
#define ERROR_NO_STORAGE 5

//For MTU negotiating
#define BLE_ATT_MTU_MAX 512

enum DeviceState
{
    IDLE = 0,
    BLE = 1,
    TRAINING = 2,
    ERROR = 3,
};
volatile DeviceState currentState = IDLE; //0 - IDLE, 1 - BLE, 2 - Collecting and saving sensor data
volatile bool stateChanged = false;
volatile bool buttonInterruptOccurred = false;

volatile bool buttonPressed = false;  // button state (pressed (held) or not pressed)
const unsigned long stateChangeDelay = 1000;  // a delay between IDLE and BLE states, so the user won't spam the button
const int debounceDelay = 250; // to eliminate debouncing effect on physical switch (ms)
volatile unsigned long lastDebounceTime = 0;
const unsigned long longPressThreshold = 2000;  // long press, for entering the TRAINING STATE in miliseconds (2 seconds)
unsigned long buttonPressTime = 0;  // time, when the button was pressed
unsigned long lastStateChangeTime = 0;  // time since the last state change, for delaying state entering

// UUIDs for BLE Service and Characteristics
#define SERVICE_UUID        "12345678-1234-1234-1234-123456789012" //012
#define SSID_CHAR_UUID      "e2e3f5a4-8c4f-11eb-8dcd-0242ac130003"
#define PASSWORD_CHAR_UUID  "e2e3f5a4-8c4f-11eb-8dcd-0242ac130004"
#define FILE_TRANSFER_UUID "e2e3f5a4-8c4f-11eb-8dcd-0242ac130005" //05
#define CONFIRMATION_UUID "e2e3f5a4-8c4f-11eb-8dcd-0242ac130006"  // confirmation from app regarding file transmission success
#define TIME_SYNC_UUID "e2e3f5a4-8c4f-11eb-8dcd-0242ac130007"
#define BATT_LEVEL_UUID "e2e3f5a4-8c4f-11eb-8dcd-0242ac130021"
#define FIRMWARE_VERSION_UUID "e2e3f5a4-8c4f-11eb-8dcd-0242ac130022"
#define FILES_TO_SEND_UUID "e2e3f5a4-8c4f-11eb-8dcd-0242ac130023"
#define PRIVATE_KEY_UUID "e2e3f5a4-8c4f-11eb-8dcd-0242ac130025"

//SD Card using SdFat
#define CS_PIN 5
SdFat sdCard;
SdFile dataFile;
SdFile root;
SdFile entry;

// Definitions for voltage divider and Li-Po battery

#define MAX_ADC_VALUE 4095  // Maximum ADC value for ESP32 (12-bit)
#define REFERENCE_VOLTAGE 3.3  // Reference voltage for ADC (ESP32 power supply)
#define R1 1000000.0  // R1 = 100 kΩ
#define R2 2000000.0  // R2 = 200 kΩ

// Voltage range for Li-Po battery (full charge and minimum safe voltage)
#define FULL_BATTERY_VOLTAGE 4.08
#define LOW_BATTERY_VOLTAGE 3.15  // Minimum safe voltage

//NimBLE pointers
NimBLEServer* pServer = nullptr;
NimBLEService* pService = nullptr; 
NimBLEAdvertising* pAdvertising = nullptr; 

NimBLECharacteristic* pSsidCharacteristic = nullptr;
NimBLECharacteristic* pPasswordCharacteristic = nullptr;
NimBLECharacteristic* pFileTransferCharacteristic = nullptr;
NimBLECharacteristic* pConfirmationCharacteristic = nullptr;
NimBLECharacteristic* pTimeSyncCharacteristic = nullptr;
NimBLECharacteristic* pBatteryStatusCharacteristic = nullptr;
NimBLECharacteristic* pRevisionNumberCharacteristic = nullptr;
NimBLECharacteristic* pFilesToSendCharacteristic = nullptr;
NimBLECharacteristic* pPrivateKeyCharacteristic = nullptr;

NimBLECharacteristicCallbacks* ssidCallbacks = nullptr;
NimBLECharacteristicCallbacks* passwordCallbacks = nullptr;
NimBLECharacteristicCallbacks* confirmationCallbacks = nullptr;
NimBLECharacteristicCallbacks* timeSyncCallbacks = nullptr;
NimBLECharacteristicCallbacks* batteryStatusCallbacks = nullptr;
NimBLECharacteristicCallbacks* revisionNumberCallbacks = nullptr;
NimBLECharacteristicCallbacks* filesToSendCallbacks = nullptr;
NimBLECharacteristicCallbacks* privateKeyCallbacks = nullptr;

bool deviceConnected = false;
bool isSubscribed = false;
uint16_t currentMTUSize = 23;

//For wifi connection to OTA
String ssid = "";
String password = "";

MAX30105 sensorMAX;
MPU6050 sensorMPU;

char filename[32];
int fileIndex = 1; //Used to create new files with new names automatically
int currentFileIndex = 1;  // last file sent +1

//The difference between these will define the sample rate (initially 10 milliseconds)
unsigned long sampleStartTime = 0;
unsigned long sampleEndTime = 0;
const int samplingRateInMillis = 10;

//For time counter in UTC format
unsigned long syncedTime = 0; // time synchronized using app's time (UNIX timestamp)
unsigned long lastSyncMillis = 0; // time in milliseconds when synchronization occurred

//For checking if a device is actually worn (in case of false going into TRAINING mode)
bool checkIfIsWorn = false;
unsigned long trainingStartTime = 0;
bool smartbandWorn = true; // flag to monitor if the device is on hand during a period of time
unsigned long smartbandTakenOffTime = 0;

//For checking if any device connects with smartband through BLE
unsigned long bleDisconnection = 0; //when this reaches the bleTimeout, the device goes from BLE to IDLE state
const unsigned long bleTimeout = 180000; //3 minutes

//Buffer to SDCard input optimization
const size_t bufferSize = 8192; // 8192 bytes
char sdBuffer[bufferSize];  // Buffer to store data before writing to SD card
size_t bufferIndex = 0;     // Current position in the buffer

//Firmware (change manually)
const String firmwareVersion = "V24";

// Resources: battery level and space on SD card
unsigned long lastResourceCheckTime = 0;
const unsigned long resourceCheckInterval = 20000; // 20 seconds

//For checking whether sensors are working properly
const unsigned long sensorCheckInterval = 20000; // 20 secods
unsigned long lastSensorCheckTime = 0;
bool max30102Working = false;
bool mpu6050Working = false;

const String privateKey = "nuvijridkvinorj";
bool provisioningComplete = false;

unsigned long lastSentChunkTime = 0;
uint16_t packetInterval = 4;


//FUNCTIONS

void checkMax30102(uint32_t irValue, uint32_t redValue) {    
    if (irValue == 0 || redValue == 0) {
        max30102Working = false;
        Serial.println("MAX30102 not working properly.");
    } else {
        max30102Working = true;
    }
}

void checkMpu6050(int16_t ax, int16_t ay, int16_t az) {
    if (ax == 0 && ay == 0 && az == 0) {
        mpu6050Working = false;
        Serial.println("MPU6050 not working properly.");
    } else {
        mpu6050Working = true;
    }
}

bool isSpaceAvailable() {
    if (!SD.begin(CS_PIN)) {
        Serial.println("SD Card failed while isSpaceAvailable()");
        currentState = ERROR;
        stateChanged = true;
        return false; 
    }

    uint64_t totalBytes = SD.totalBytes();
    uint64_t usedBytes = SD.usedBytes();

    return (totalBytes - usedBytes) >= (0.05 * totalBytes);
}

int countTrainingFiles() {
    int fileCount = 0;
    
    if (!sdCard.begin(CS_PIN, SD_SCK_MHZ(25))) {
        Serial.println("SD Card failed while countTrainingFiles()");
        currentState = ERROR;
        stateChanged = true;
        return 0; //number that seems unlikely to reach
    }

    char filename[32];

    if (!root.open("/")) {
        Serial.println("Failed to open root directory");
        return 0;
    }

    while (entry.openNext(&root, O_RDONLY)) {
        entry.getName(filename, sizeof(filename));
        if (strncmp(filename, "training_", 9) == 0 && strstr(filename, ".bin")) {
            fileCount++;
        }
        entry.close();
    }

    root.close();
    // SD.end(); // Not necessary for SdFat
    return fileCount;
}

void signalError(int errorCode) {
    for (int i = 0; i < errorCode; i++) {
        digitalWrite(LED_PIN_BLE, HIGH);
        digitalWrite(LED_PIN_TRAINING, HIGH);
        delay(300);
        digitalWrite(LED_PIN_BLE, LOW);
        digitalWrite(LED_PIN_TRAINING, LOW);
        delay(300);
    }
    delay(1000); 
}


void IRAM_ATTR handleButtonPress() { //changing between states is possible after 2 seconds delay
    buttonInterruptOccurred = true; //all the code moved to loop() because of interruption timeout (program took to long while interruption)
}

unsigned long getCurrentTime() {
    if (syncedTime == 0) {
        // Not synchronized
        Serial.println("Time not synchronized");
        return 0;
    }

    unsigned long elapsedMillis = millis() - lastSyncMillis;
    unsigned long currentTime = syncedTime + elapsedMillis / 1000; // Adding elapsed time to synced time
    return currentTime;
}

// Function to calculate battery percentage
int calculateBatteryPercentage(float batteryVoltage) {
    // Ensure the voltage is within the range from 3.15V to 4.08V
    if (batteryVoltage >= FULL_BATTERY_VOLTAGE) {
        return 100;
    } else if (batteryVoltage <= LOW_BATTERY_VOLTAGE) {
        return 0;
    } else {
        // Calculate the battery percentage
        float percentage = (batteryVoltage - LOW_BATTERY_VOLTAGE) / (FULL_BATTERY_VOLTAGE - LOW_BATTERY_VOLTAGE) * 100;
        return (int)percentage;
    }
}

// Function to read battery voltage using ADC
float readBatteryVoltage() {
    // Read the ADC value
    int adcValue = analogRead(BATTERY_PIN);
    
    // Convert ADC value to voltage across the voltage divider
    float voltageDivider = ((float)adcValue / MAX_ADC_VALUE) * REFERENCE_VOLTAGE;

    // Calculate the actual battery voltage based on the voltage divider
    float batteryVoltage = voltageDivider * (R1 + R2) / R2;
    
    return batteryVoltage;
}

void startTraining() {

    smartbandTakenOffTime = 0;
    do {
        snprintf(filename, sizeof(filename), "/training_%d.bin", fileIndex++);
    } while (sdCard.exists(filename));

    if (!dataFile.open(filename, O_RDWR | O_CREAT | O_AT_END)) {
        Serial.println("Failed to open file for writing with SdFat");
        return;
    }

    unsigned long trainingStartTime = getCurrentTime() + 3600; //3600 for Poland's UTC+1 timezone offset
    // Writing the UNIX timestamp as binary data
    dataFile.write((uint8_t*)&trainingStartTime, sizeof(trainingStartTime));

    Serial.printf("Started training session: %s, time: %lu\n", filename, trainingStartTime);
}

void flushBufferToSD() {
    if (bufferIndex == 0) {
        return; // Nothing to flush
    }

    // Ensure the file is open before writing
    if (!dataFile.isOpen()) {
        Serial.println("SD card not ready for writing");
        return;
    }

    // Write buffer content to SD card
    dataFile.write((uint8_t*)sdBuffer, bufferIndex);

    Serial.println(dataFile.size());
    // Clear buffer
    bufferIndex = 0;
}

void collectAndBufferData() {
    uint32_t irValue = sensorMAX.getIR();
    uint32_t redValue = sensorMAX.getRed();
    int16_t ax, ay, az;
    //int16_t gx, gy, gz; //gyroscope data
    //sensorMPU.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);   
    sensorMPU.getAcceleration(&ax, &ay, &az);

    unsigned long currentTime = millis();

    if (currentTime - lastSensorCheckTime >= sensorCheckInterval) {
        checkMax30102(irValue, redValue);
        checkMpu6050(ax, ay, az);
        lastSensorCheckTime = currentTime;

        if (!max30102Working || !mpu6050Working) {
            currentState = ERROR;
            stateChanged = true;
            flushBufferToSD();
            return;
        }
    }

    
    if (irValue < 5000 && smartbandTakenOffTime == 0) {
        smartbandTakenOffTime = millis();
    } else if (irValue < 5000 && millis() - smartbandTakenOffTime >= 15000 && smartbandTakenOffTime > 0) { // 15 seconds
        Serial.println("Device is not worn, proceeding to IDLE");
        currentState = IDLE;
        stateChanged = true;
        smartbandTakenOffTime = 0;
    } else if (smartbandTakenOffTime > 0 && irValue >= 5000) {
        smartbandTakenOffTime = 0;
    }
    
    if (bufferIndex + sizeof(irValue) + sizeof(redValue) + sizeof(ax) * 3 >= bufferSize) {
        flushBufferToSD();
    }

    memcpy(&sdBuffer[bufferIndex], &irValue, sizeof(irValue));
    bufferIndex += sizeof(irValue);
    memcpy(&sdBuffer[bufferIndex], &redValue, sizeof(redValue));
    bufferIndex += sizeof(redValue);
    memcpy(&sdBuffer[bufferIndex], &ax, sizeof(ax));
    bufferIndex += sizeof(ax);
    memcpy(&sdBuffer[bufferIndex], &ay, sizeof(ay));
    bufferIndex += sizeof(ay);
    memcpy(&sdBuffer[bufferIndex], &az, sizeof(az));
    bufferIndex += sizeof(az);
}

void endTraining() {
    flushBufferToSD();

    if (dataFile.isOpen()) {
        dataFile.close();
        Serial.println("SD file closed");
    }
}

bool isWorn() {
    long irValue = sensorMAX.getIR();
    if (irValue < 5000) { //TODO: Check if this value is good
        return false; 
    } else {
        return true;
    }
}

void performOTAUpdate(const char* firmwareUrl) {
    Serial.println("Starting OTA update...");

    HTTPClient http;
    http.begin(firmwareUrl);  // Use HTTPS with root CA if necessary for secure connection
    int httpCode = http.GET();

    if (httpCode == HTTP_CODE_OK) {
        int contentLength = http.getSize();
        Serial.printf("Content Length: %d bytes\n", contentLength);

        WiFiClient* stream = http.getStreamPtr();
        if (contentLength > 0 && stream != nullptr) {
            // Check available free space for OTA
            size_t freeSpace = ESP.getFreeSketchSpace();
            Serial.printf("Free space for OTA: %d bytes\n", freeSpace);

            // Check if the available space is sufficient
            if (contentLength > freeSpace) {
                Serial.println("Not enough space for OTA update. Exiting...");
                return;
            }

            bool canBegin = Update.begin(contentLength);
            if (!canBegin) {
                Serial.printf("Unable to start OTA update: %s\n", Update.getError());
                return;
            }

            size_t totalRead = 0;
            uint8_t buffer[128];  // Adjust buffer size as needed
            while (http.connected() && (totalRead < contentLength)) {
            size_t availableBytes = stream->available();
            if (availableBytes > 0) {
                int bytesToRead = min(availableBytes, sizeof(buffer));
                int bytesRead = stream->readBytes(buffer, bytesToRead);
                totalRead += bytesRead;

                // Write data and handle errors
                size_t written = Update.write(buffer, bytesRead);
                if (written != bytesRead) {
                    Serial.printf("Error: Written %d bytes but read %d bytes\n", written, bytesRead);
                    Update.abort();  // Abort the update process on failure
                    return;
                }
                Serial.printf("Read %d bytes, total read: %d/%d bytes\n", bytesRead, totalRead, contentLength);
            }
}

            if (Update.end()) {
                if (Update.isFinished()) {
                    Serial.println("OTA update completed successfully. Restarting...");
                    ESP.restart();
                } else {
                    Serial.println("OTA update did not finish correctly.");
                }
            } else {
                Serial.printf("Update error: %s\n", Update.getError());
            }
        } else {
            Serial.println("Invalid content length or stream is null!");
        }
    } else {
        Serial.printf("HTTP error: %d\n", httpCode);
    }

    http.end();
}

// WiFi connection setup
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

        Serial.println("Performing an update through OTA");
        size_t freeSketchSpace = ESP.getFreeSketchSpace();
        Serial.printf("Free space for OTA: %u bytes\n", freeSketchSpace);
        delay(3000);
        performOTAUpdate("http://192.168.1.47:9000/ota/firmware.bin");

    } else {
        Serial.println("\nFailed to connect to WiFi.");
    }
}

class PrivateKeyCallbacks : public NimBLECharacteristicCallbacks {
public:
    void onWrite(NimBLECharacteristic* pCharacteristic) override {
        String receivedKey = pCharacteristic->getValue().c_str();
        if (receivedKey == privateKey) {
            provisioningComplete = true;
            Serial.println("Provisioning successful, key verified.");
        } else {
            provisioningComplete = false;
            Serial.println("Provisioning failed, key mismatch.");
        }
    }
};

// BLE characteristic callbacks for WiFi
class CharacteristicCallbacks : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic* pCharacteristic) override {
        if (!provisioningComplete) {
            Serial.println("Provisioning not completed. Transmission blocked.");
            return;
        }
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
            connectToWiFi();
        }
    }
};

class BatteryStatusCallbacks : public NimBLECharacteristicCallbacks {
public:
    void onRead(NimBLECharacteristic* pCharacteristic) override {
        if (!provisioningComplete) {
            Serial.println("Provisioning not completed. Transmission blocked.");
            return;
        }
        float battery = readBatteryVoltage();
        int battPerc = calculateBatteryPercentage(battery);
        pCharacteristic->setValue(battPerc);
        pCharacteristic->notify();
        Serial.println("Battery status read.");
    }
};

class RevisionNumberCallbacks : public NimBLECharacteristicCallbacks {
public:
    void onRead(NimBLECharacteristic* pCharacteristic) override {
        if (!provisioningComplete) {
            Serial.println("Provisioning not completed. Transmission blocked.");
            return;
        }
        pCharacteristic->setValue(firmwareVersion);
        pCharacteristic->notify();
        Serial.println("Revision number read.");
    }
};

class FilesToSendCallbacks : public NimBLECharacteristicCallbacks {
public:
    void onRead(NimBLECharacteristic* pCharacteristic) override {
        if (!provisioningComplete) {
            Serial.println("Provisioning not completed. Transmission blocked.");
            return;
        }
        int filesToSend = countTrainingFiles();
        pCharacteristic->setValue(filesToSend);
        pCharacteristic->notify();
        Serial.println("Files to send count read.");
    }
};

void logStackUsage() {
    TaskHandle_t taskHandle = xTaskGetCurrentTaskHandle();
    UBaseType_t freeStack = uxTaskGetStackHighWaterMark(taskHandle);
    Serial.printf("Free stack space for current task: %d bytes\n", freeStack);
}

//Callback for sending training files to apps
class FileTransferCallbacks : public NimBLECharacteristicCallbacks {
private:
    SdFile file;
    size_t chunkSize = currentMTUSize - 3; // Based on MTU size
    size_t fileSize = 0;
    size_t bytesSent = 0;
    String currentFileName;
    uint8_t *buffer = nullptr;
    int32_t bytesRead = 0;

public:
    bool transferInProgress = false;
    bool sendEndMessage = false;
    bool messageSizeSent = false;

public:
    void onRead(NimBLECharacteristic* pCharacteristic) override {
            Serial.println("READ.");
    }
    

    void onSubscribe(NimBLECharacteristic* pCharacteristic, ble_gap_conn_desc* desc, uint16_t subValue) override {
        if (!provisioningComplete) {
            Serial.println("Provisioning not completed. Transmission blocked.");
            return;
        }

        if(subValue == 0x0000){
            // Client unsubscribed
            isSubscribed = false;
            Serial.println("Client unsubscribed from notifications");
            resetFileTransferState();
        }   
        else if(!isSubscribed){
            // Client subscribed
            isSubscribed = true;
            chunkSize = currentMTUSize - 3;
            Serial.println("Client subscribed to notifications");

            if (!transferInProgress && !sendEndMessage) {
                if (!sdCard.begin(CS_PIN, SD_SCK_MHZ(25))) {
                    Serial.println("Card Mount Failed");
                    currentState = ERROR;
                    stateChanged = true;
                    return;
                }
                sendNextFile();

                
            }
            transferInProgress = true;
            messageSizeSent = false;
        }
    }

    void sendMessageSize(NimBLECharacteristic* pCharacteristic) {
        uint32_t messageSize = fileSize;
        uint8_t sizeBytes[4];

        // Little Endian Conversion
        sizeBytes[0] = (messageSize >> 0) & 0xFF;
        sizeBytes[1] = (messageSize >> 8) & 0xFF;
        sizeBytes[2] = (messageSize >> 16) & 0xFF;
        sizeBytes[3] = (messageSize >> 24) & 0xFF;

        pCharacteristic->setValue(sizeBytes, sizeof(sizeBytes));
        pCharacteristic->notify();

        messageSizeSent = true;
        Serial.printf("Sent message size: %d bytes\n", messageSize);
    }

    void startFileTransfer(const char* fileName) {
        delay(10);
        currentFileName = String(fileName);

        if (!file.open(fileName, O_RDONLY)) {
            Serial.println("Failed to open file for reading");
            sendNextFile(); // Move to the next file if this one fails
        }

        fileSize = file.fileSize();
        bytesSent = 0;
        transferInProgress = true;

        Serial.println("File transfer started");

        return;
    }

    void sendNextChunk(NimBLECharacteristic* pCharacteristic) {

        if (!transferInProgress || !file.isOpen() || !isSubscribed || !deviceConnected) {
            return;
        }

        if (!buffer) {
            buffer = (uint8_t*)malloc(chunkSize);
            if(!buffer){
                Serial.println("Failed to allocate buffer");
                resetFileTransferState();
                return;
            }
        }

        bytesRead = file.read(buffer, chunkSize);
        if (bytesRead > 0) {
            // Send the chunk
            pCharacteristic->setValue(buffer, bytesRead);
            pCharacteristic->notify();

            bytesSent += bytesRead;
            Serial.printf("Sent %d/%d bytes\n", bytesSent, fileSize);
        }
        else{
            Serial.println("EOF or error");
            free(buffer);
            buffer = nullptr;
            file.close();
            transferInProgress = false;
            sendEndMessage = true;
        }

        return;
    }

    void sendEnd(NimBLECharacteristic* pCharacteristic) {
        //const char* endMessage = "END";
        uint8_t endMessage[16] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; //16 bytes of only zeroes, very unlikely for the sensors to give that data
        pCharacteristic->setValue(endMessage, sizeof(endMessage));
        pCharacteristic->notify();

        Serial.println("Sent ending message");

        // Reset flags to start the sending process again
        sendEndMessage = false;
        transferInProgress = false;
    }

    void deleteFile() {
        if (!sdCard.begin(CS_PIN, SD_SCK_MHZ(25))) {
            Serial.println("SD Card Fail");
            currentState = ERROR;
            stateChanged = true;
            return;
        }

        if (!currentFileName.isEmpty() && sdCard.exists(currentFileName.c_str())) {
            if (sdCard.remove(currentFileName.c_str())) {
                Serial.print("File deleted from SD card: ");
                Serial.println(currentFileName);
            } else {
                Serial.println("Failed to delete file from SD card.");
            }
        } else {
            Serial.println("Invalid file name or file does not exist.");
        }

        sdCard.end();
    }

    void sendNextFile() {
        char filename[32];
        currentFileIndex = 1;

        while (true) {
            snprintf(filename, sizeof(filename), "/training_%d.bin", currentFileIndex);
            if (sdCard.exists(filename)) {
                Serial.print("Found file to send: ");
                Serial.println(filename);
                startFileTransfer(filename);
                return;
            } else {
                currentFileIndex++;
            }
            if (currentFileIndex > 1000) { //1000 seems pretty unlikely to be ever reached, todo: implement a better limit system
                Serial.println("No more files to send.");
                return;
            }
        }
    }

    void resetFileTransferState() {
        transferInProgress = false;
        sendEndMessage = false;
        messageSizeSent = false;
        bytesSent = 0;
        if (file.isOpen()) {
            file.close();
            Serial.println("File closed after disconnection.");
        }
        sdCard.end();
    }
};

FileTransferCallbacks* fileTransferCallbacks = nullptr;

//Simple callbacks for informing the esp32 that someone connected through BLE
class ServerCallbacks: public NimBLEServerCallbacks {
    void onConnect(NimBLEServer* pServer, ble_gap_conn_desc* desc) override {
        deviceConnected = true;
        isSubscribed = false;
        bleDisconnection = millis();
        provisioningComplete = false;
        Serial.println("Client connected. Awaiting private key for provisioning.");

        struct ble_gap_upd_params connParams;
        connParams.itvl_min = 6; // Minimalny interwał = 15 ms (12 * 1.25 ms)
        connParams.itvl_max = 15; // Maksymalny interwał = 30 ms (24 * 1.25 ms)
        connParams.latency = 0;   // Brak opóźnienia (slave latency)
        connParams.supervision_timeout = 200; // Timeout = 200 * 10 ms = 2 sekundy

                // BLE Throughput Booster #1: Maksymalny rozmiar PDU
        pServer->setDataLen(desc->conn_handle, 251);

        // BLE Throughput Booster #2: Optymalizacja parametrów połączenia
         pServer->updateConnParams(desc->conn_handle, 12, 12, 0, 200);
        vTaskDelay(pdMS_TO_TICKS(100));

        int rc= ble_gap_set_prefered_le_phy(desc->conn_handle,
        
                                          BLE_GAP_LE_PHY_2M, // Preferowane PHY TX
                                          BLE_GAP_LE_PHY_1M, // Preferowane PHY RX
                                          BLE_HCI_LE_PHY_CODED_ANY); // Brak kodowania
        if (rc == 0) {
            Serial.println("PHY preference set to 2M successfully");
        } else {
            Serial.printf("Failed to set PHY preference; error code: %d\n", rc);
        }

        uint8_t tx_phy, rx_phy;
       
        rc =  ble_gap_read_le_phy(desc->conn_handle, &tx_phy, &rx_phy);
        if (rc == 0) {
            Serial.printf("Negotiated PHY: TX = %d, RX = %d\n", tx_phy, rx_phy);
        } else {
            Serial.printf("Failed to read PHY; error code: %d\n", rc);
        }
    };

    void onDisconnect(NimBLEServer* pServer) override {
        bleDisconnection = millis();
        deviceConnected = false;
        isSubscribed = false;
        Serial.println("Client disconnected.");
        provisioningComplete = false;
        // Reset file transfer state
        if (fileTransferCallbacks != nullptr) {
            // Reset flags to start from the beginning if disconnected
            auto* ftCallback = static_cast<FileTransferCallbacks*>(fileTransferCallbacks);
            ftCallback->resetFileTransferState();
        }
    }

    void onMTUChange(uint16_t MTU, ble_gap_conn_desc* desc) override {
        Serial.print("MTU size updated to: ");
        Serial.println(MTU);  // This prints the negotiated MTU size
        currentMTUSize = MTU;
    }
};

// Characteristic response for successful file transmission
class ConfirmationCallbacks : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic* pCharacteristic) override {
        if (!provisioningComplete) {
            Serial.println("Provisioning not completed. Transmission blocked.");
            return;
        }
        std::string confirmation = pCharacteristic->getValue();

        if (confirmation == "OK"){
            isSubscribed = false;
            Serial.println("Confirmation received from app. Deleting file and setting isSubscribed to false.");
            if (fileTransferCallbacks != nullptr) {
                //casting to FileTransferCalllbacks, because, NIMBLECharactersiticCallbacks does not have deleteFile() method, so workaround necessary
                static_cast<FileTransferCallbacks *>(fileTransferCallbacks)->deleteFile();
            }
        } else {
            Serial.println("Invalid confirmation received.");
        }
    }
};

class TimeSyncCallbacks : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic* pCharacteristic) override {
        if (!provisioningComplete) {
            Serial.println("Provisioning not completed. Transmission blocked.");
            return;
        }
        std::string value = pCharacteristic->getValue();

        // IMPORANT: send UNIX timestamp (seconds passed since 1970-01-01 00:00:00 UTC)
        // IMPORTANT2: send bytes in Little Endian
        if (value.length() == sizeof(unsigned long)) {
            syncedTime = *(unsigned long*)value.data(); //basically a casting to unsinged long from 4-byte string
            lastSyncMillis = millis();
            Serial.print("Synchronized time: ");
            Serial.println(syncedTime);
        } else {
            Serial.println("Invalid time data received");
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
    pinMode(BUTTON_PIN, INPUT_PULLUP); // using an inner built-in resistor on ESP32
    pinMode(BATTERY_PIN, INPUT);  // Set up the ADC pin for battery voltage readings

    attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), handleButtonPress, CHANGE);

    // The initial LED state 
    digitalWrite(LED_PIN_IDLE, LOW);
    digitalWrite(LED_PIN_BLE, LOW);
    digitalWrite(LED_PIN_TRAINING, LOW);

    // MAX30102 initialization
    while(!sensorMAX.begin(Wire, I2C_SPEED_FAST)) {
        Serial.println("MAX30102 not found. Check wiring.");
        signalError(ERROR_SENSOR_MAX30102);
    }
    sensorMAX.setup(); // Configure sensor with default settings
    sensorMAX.setPulseAmplitudeRed(0);
    sensorMAX.setPulseAmplitudeIR(0);

    // MPU6050 initialization
    sensorMPU.initialize();
    while(!sensorMPU.testConnection()) {
        Serial.println("MPU6050 connection failed. Check wiring.");
        signalError(ERROR_SENSOR_MPU6050);
    }
    sensorMPU.setSleepEnabled(true);

    // SD card module initialization
    if (!sdCard.begin(CS_PIN, SD_SCK_MHZ(25))) { 
        Serial.println("Card Mount Failed");
        signalError(ERROR_SD_NOT_FOUND);
    }



    uint64_t cardSize = (sdCard.card()->sectorCount() * 512ULL) / (1024 * 1024);
    Serial.printf("SD Card Size: %lluMB\n", cardSize);

    delay(500); // for power stabilization

    // BLE initialization
    NimBLEDevice::init("ESP32 D3K");
    NimBLEDevice::setMTU(512);

    pServer = NimBLEDevice::createServer();
    pServer->setCallbacks(new ServerCallbacks());

    pService = pServer->createService(SERVICE_UUID);

    ssidCallbacks = new CharacteristicCallbacks();
    pSsidCharacteristic = pService->createCharacteristic(
        SSID_CHAR_UUID,
        NIMBLE_PROPERTY::WRITE
    );
    pSsidCharacteristic->setCallbacks(ssidCallbacks);

    passwordCallbacks = new CharacteristicCallbacks();
    pPasswordCharacteristic = pService->createCharacteristic(
        PASSWORD_CHAR_UUID,
        NIMBLE_PROPERTY::WRITE
    );
    pPasswordCharacteristic->setCallbacks(passwordCallbacks);

    fileTransferCallbacks = new FileTransferCallbacks();
    pFileTransferCharacteristic = pService->createCharacteristic(
        FILE_TRANSFER_UUID,
        NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY
    );
    pFileTransferCharacteristic->setCallbacks(fileTransferCallbacks);

    confirmationCallbacks = new ConfirmationCallbacks();
    pConfirmationCharacteristic = pService->createCharacteristic(
        CONFIRMATION_UUID,
        NIMBLE_PROPERTY::WRITE
    );
    pConfirmationCharacteristic->setCallbacks(confirmationCallbacks);

    timeSyncCallbacks = new TimeSyncCallbacks();
    pTimeSyncCharacteristic = pService->createCharacteristic(
        TIME_SYNC_UUID,
        NIMBLE_PROPERTY::WRITE
    );
    pTimeSyncCharacteristic->setCallbacks(timeSyncCallbacks);

    batteryStatusCallbacks = new BatteryStatusCallbacks();
    pBatteryStatusCharacteristic = pService->createCharacteristic(
        BATT_LEVEL_UUID,
        NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY
    );
    pBatteryStatusCharacteristic->setCallbacks(batteryStatusCallbacks);

    revisionNumberCallbacks = new RevisionNumberCallbacks();
    pRevisionNumberCharacteristic = pService->createCharacteristic(
        FIRMWARE_VERSION_UUID,
        NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY
    );
    pRevisionNumberCharacteristic->setCallbacks(revisionNumberCallbacks);

    filesToSendCallbacks = new FilesToSendCallbacks();
    pFilesToSendCharacteristic = pService->createCharacteristic(
        FILES_TO_SEND_UUID,
        NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY
    );
    pFilesToSendCharacteristic->setCallbacks(filesToSendCallbacks);

    privateKeyCallbacks = new PrivateKeyCallbacks();
    pPrivateKeyCharacteristic = pService->createCharacteristic(
        PRIVATE_KEY_UUID,
        NIMBLE_PROPERTY::WRITE
    );
    pPrivateKeyCharacteristic->setCallbacks(privateKeyCallbacks);

    pService->start();

    pAdvertising = NimBLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    //pAdvertising->start();

    Serial.println("BLE initialized");

    // IDLE MODE IN THE BEGINNING
    digitalWrite(LED_PIN_IDLE, HIGH);
    digitalWrite(LED_PIN_BLE, LOW);
    digitalWrite(LED_PIN_TRAINING, LOW);
    sdCard.end();
    Serial.println("Initial idle mode");
}

void loop() {

    //Serial.println("Goldap");

    if (buttonInterruptOccurred || buttonPressed) {
        buttonInterruptOccurred = false; 

        unsigned long currentTime = millis();

        // debounce check
        if ((currentTime - lastDebounceTime) > debounceDelay) { 
            lastDebounceTime = currentTime;  // debounce time

            if (digitalRead(BUTTON_PIN) == LOW && !buttonPressed) {
                 //button pressed, LOW
                buttonPressed = true; 
                buttonPressTime = currentTime;
            } 
            else if (digitalRead(BUTTON_PIN) == HIGH && buttonPressed) {
                //button not pressed, HIGH
                buttonPressed = false;

                // check if it was short or long press
                if (currentTime - buttonPressTime < longPressThreshold) {
                    // short press
                    if (currentTime - lastStateChangeTime > stateChangeDelay) {
                        // short press, but change only if 1 second has passed since the last state change
                        if (currentState == IDLE) {
                            currentState = BLE;
                        } else if (currentState == BLE) {
                            currentState = IDLE;
                        } else if (currentState == TRAINING) {
                            currentState = IDLE;
                        }
                        lastStateChangeTime = currentTime; 
                    }
                } else if(currentState != TRAINING){
                    // long press
                    currentState = TRAINING;  
                    lastStateChangeTime = currentTime; 
                }
                else{
                    currentState = IDLE;
                }
                stateChanged = true; 
            }
            else if(digitalRead(BUTTON_PIN) == LOW && buttonPressed && currentTime - buttonPressTime >= longPressThreshold && currentState != TRAINING){
                currentState = TRAINING;
                stateChanged = true;
                buttonPressed = false;
                lastStateChangeTime = currentTime;
            }
        }
    }

    if (stateChanged) {
        float batteryVoltage = readBatteryVoltage();
        stateChanged = false;

        switch (currentState) {
            case IDLE: // Idle state, no data are collected from sensors
                digitalWrite(LED_PIN_IDLE, HIGH);
                digitalWrite(LED_PIN_BLE, LOW);
                digitalWrite(LED_PIN_TRAINING, LOW);
                
                endTraining();
                checkIfIsWorn = false;

                if (pAdvertising != nullptr && pAdvertising->isAdvertising()) {
                    pAdvertising->stop();
                }

                sdCard.end(); 
                
                sensorMAX.setPulseAmplitudeRed(0);
                sensorMAX.setPulseAmplitudeIR(0);
                sensorMPU.setSleepEnabled(true);

                Serial.println("Idle mode");

                esp_sleep_enable_ext0_wakeup(GPIO_NUM_13, 0);  // 0 = LOW level to trigger wakeup
                esp_light_sleep_start(); // Device will sleep here until GPIO13 wakes it
                break;

            case BLE: // BLE state, BLE server is on, should broadcast
                // Check whether battery voltage is high enough 
                if (batteryVoltage < 3.15) {
                    Serial.println("Low battery level. Recharge");
                    currentState = IDLE;
                    stateChanged = true;
                    break;
                }

                digitalWrite(LED_PIN_IDLE, LOW);
                digitalWrite(LED_PIN_BLE, HIGH);
                digitalWrite(LED_PIN_TRAINING, LOW);

                if (!sdCard.begin(CS_PIN, SD_SCK_MHZ(25))) {  // Use SdFat object for initialization
                    Serial.println("Card Mount failed during BLE initalization state");
                    currentState = ERROR;
                    stateChanged = true;
                }

                delay(500); //for power stabilization

                if (!pAdvertising->isAdvertising()) {
                    pAdvertising->start();
                    Serial.println("BLE advertising started.");
                }

                endTraining();
                checkIfIsWorn = false;
                bleDisconnection = millis();

                sensorMAX.setPulseAmplitudeRed(0);
                sensorMAX.setPulseAmplitudeIR(0);
                sensorMPU.setSleepEnabled(true);

                Serial.println("BLE mode activated and advertising started");
                break;

            case TRAINING: // Exercise state, collect data from sensors
            // Check whether battery voltage is high enough 
                if (batteryVoltage < 3.15) {
                    Serial.println("Low battery level. Recharge");
                    currentState = IDLE;
                    stateChanged = true;
                    break;
                }

                if (pAdvertising != nullptr && pAdvertising->isAdvertising()) {
                    pAdvertising->stop();
                }

                // Check if there's enough space on SD card for new file (Above 5% of all space available).
                if ((!isSpaceAvailable())) {
                    Serial.println("Not enough storage on SD card. Send training files to app");
                    currentState = ERROR;
                    stateChanged = true;
                    break;
                }

                sensorMAX.setup(); // Configure sensor with default settings
                sensorMAX.setPulseAmplitudeRed(60);
                sensorMAX.setSampleRate(0x18);
                sensorMAX.setFIFOAverage(8);
                sensorMPU.setSleepEnabled(false);

                digitalWrite(LED_PIN_IDLE, LOW);
                digitalWrite(LED_PIN_BLE, LOW);
                digitalWrite(LED_PIN_TRAINING, HIGH);

                if (!sdCard.begin(CS_PIN, SD_SCK_MHZ(25))) {
                    Serial.println("Card Mount Failed during training initialization");
                    currentState = ERROR;
                    stateChanged = true;
                } else {
                    Serial.println("Training mode - SD logging started");
                    checkIfIsWorn = true;
                    trainingStartTime = millis();
                }
                break;

            case ERROR:
                while(!sdCard.begin(CS_PIN, SD_SCK_MHZ(25))) {
                    Serial.println("Card Mount Failed during ERROR state");
                    signalError(ERROR_SD_NOT_FOUND);
                }

                while ((!isSpaceAvailable())) {
                    Serial.println("Not enough storage on SD card. Send training files to app");
                    signalError(ERROR_NO_STORAGE);
                }

                while(!sensorMAX.begin(Wire, I2C_SPEED_FAST)) {
                    Serial.println("Failed to initialize MAX30102 sensor.");
                    signalError(ERROR_SENSOR_MAX30102);
                }
                
                if (!sensorMPU.testConnection()) {
                    Serial.println("Failed to initialize MPU6050 sensor.");
                    signalError(ERROR_SENSOR_MPU6050);
                    sensorMPU.initialize();
                    sensorMPU.setSleepEnabled(false);
                } 

                currentState = IDLE;
                stateChanged = true;
                checkIfIsWorn = false;
                break;

            default:
                digitalWrite(LED_PIN_IDLE, LOW);
                digitalWrite(LED_PIN_BLE, LOW);
                digitalWrite(LED_PIN_TRAINING, LOW);
                checkIfIsWorn = false;
                break;
        }
    }

    if(checkIfIsWorn){
        if(!isWorn()){
            if(millis() - trainingStartTime >= 15000){  //15 seconds
                Serial.println("Device is not worn, proceeding to IDLE");
                currentState = IDLE;
                stateChanged = true;
                checkIfIsWorn = false;
            }
        }
        else{
            startTraining();
            checkIfIsWorn = false;
        }
    }

    if(currentState == TRAINING && !checkIfIsWorn){
        sampleStartTime = millis();

        if (sampleStartTime - lastResourceCheckTime >= resourceCheckInterval) {
            lastResourceCheckTime = sampleStartTime;

            if (readBatteryVoltage() < 3.15) {
                Serial.println("Low battery level. Recharge.");
                endTraining();
                currentState = IDLE;
                stateChanged = true;
                return;
            }

            if (!isSpaceAvailable()) {
                Serial.println("No more space on SD card. Please send files through BLE.");
                endTraining();
                currentState = IDLE;
                stateChanged = true;
                return;
            }
        }

        collectAndBufferData();

        do{
            sampleEndTime = millis();
            //Serial.println(sampleEndTime - sampleStartTime);        
            } while (sampleEndTime - sampleStartTime < samplingRateInMillis); // 10 miliseconds = 100Hz sampling rate
    }
    
    if(currentState == BLE){
        if(millis() - bleDisconnection >= bleTimeout && !deviceConnected){
            Serial.println("No connection for a longer time, switching to IDLE");
            currentState = IDLE;
            stateChanged = true;
        }

        if(millis() - lastSentChunkTime >= 25){ //packet interval
            lastSentChunkTime = millis();

            if(isSubscribed && (fileTransferCallbacks->transferInProgress || fileTransferCallbacks->sendEndMessage)){
                if (!fileTransferCallbacks->messageSizeSent) {
                    fileTransferCallbacks->sendMessageSize(pFileTransferCharacteristic); // Send size of the message
                    fileTransferCallbacks->messageSizeSent = true;
                } else if (fileTransferCallbacks->transferInProgress) {
                    fileTransferCallbacks->sendNextChunk(pFileTransferCharacteristic); // Send the next data chunk
                } else if (fileTransferCallbacks->sendEndMessage) {
                    fileTransferCallbacks->sendEnd(pFileTransferCharacteristic); // Notify the end of the file
                    fileTransferCallbacks->sendEndMessage = false;  // Reset state for next file transfer
                }
            }
        }
    }


}