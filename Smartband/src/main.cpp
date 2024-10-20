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

//JSON
#include <ArduinoJson.h>

//Time, note: send bytes in Little Endian
#include <time.h>

//Working states 
#define LED_PIN_IDLE 15
#define LED_PIN_BLE 2
#define LED_PIN_TRAINING 4
#define BUTTON_PIN 13

//Error codes
#define ERROR_SD_NOT_FOUND 1
#define ERROR_SD_CORRUPT 2
#define ERROR_SENSOR_MPU6050 3
#define ERROR_SENSOR_MAX30102 4

enum DeviceState
{
    IDLE = 0,
    BLE = 1,
    TRAINING = 2,
    ERROR = 3
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

//SD Card
#define CS_PIN 5

//NimBLE pointers
NimBLEServer* pServer = nullptr;
NimBLEService* pService = nullptr; 
NimBLEAdvertising* pAdvertising = nullptr; 

NimBLECharacteristic* pSsidCharacteristic = nullptr;
NimBLECharacteristic* pPasswordCharacteristic = nullptr;
NimBLECharacteristic* pFileTransferCharacteristic = nullptr;
NimBLECharacteristic* pConfirmationCharacteristic = nullptr;
NimBLECharacteristic* pTimeSyncCharacteristic = nullptr;

NimBLECharacteristicCallbacks* ssidCallbacks = nullptr;
NimBLECharacteristicCallbacks* passwordCallbacks = nullptr;
NimBLECharacteristicCallbacks* fileTransferCallbacks = nullptr;
NimBLECharacteristicCallbacks* confirmationCallbacks = nullptr;
NimBLECharacteristicCallbacks* timeSyncCallbacks = nullptr;
bool deviceConnected = false;

//For wifi connection (withheld)
String ssid = "";
String password = "";

MAX30105 sensorMAX;
MPU6050 sensorMPU;


File dataFile;
char filename[32];
//char filenameJSON[32];
int fileIndex = 1; //Used to create new files with new names automatically
int currentFileIndex = 1;  // last file sent +1

//The difference between these will define the sample rate (initially 10 miliseconds)
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
const unsigned long bleTimeout = 180000;

//FUNCTIONS

void signalError(int errorCode) {
    for (int i = 0; i < errorCode; i++) {
        digitalWrite(LED_PIN_BLE, HIGH);
        digitalWrite(LED_PIN_TRAINING, HIGH);
        delay(500);
        digitalWrite(LED_PIN_BLE, LOW);
        digitalWrite(LED_PIN_TRAINING, LOW);
        delay(500);
    }
    delay(2000); 
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

void startTraining() {
    unsigned long trainingStartTime = getCurrentTime() + 3600; //3600, because Poland is in UTC+1 timezone

    do {
          snprintf(filename, sizeof(filename), "/training_%d.csv", fileIndex++); 
    } while (SD.exists(filename));
        
    dataFile = SD.open(filename, FILE_WRITE);
    if (!dataFile) {
        Serial.println("Failed to open file for writing");
        return;
    }

    //Saving time in UTC
    char timeString[30];
    struct tm *timeInfo = gmtime((time_t *)&trainingStartTime);
    strftime(timeString, sizeof(timeString), "%Y-%m-%d %H:%M:%S", timeInfo);

    // CSV header
    //dataFile.printf("Training start time (UTC): %s\n", timeString); //UTC
    dataFile.printf("%s\n", timeString); //UTC
    //dataFile.printf("%lu\n", trainingStartTime); //UNIX
    dataFile.println("IR,RED,Ax,Ay,Az");
}

void collectAndSaveData() {
    
    long irValue = sensorMAX.getIR();
    long redValue = sensorMAX.getRed();

    int16_t ax, ay, az; //accelerometer data
    //int16_t gx, gy, gz; //gyroscope data
    //sensorMPU.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    sensorMPU.getAcceleration(&ax, &ay, &az);

    if(irValue < 5000 && smartbandTakenOffTime == 0){
        smartbandTakenOffTime = millis();
    } 
    else if(irValue < 5000 && millis() - smartbandTakenOffTime >= 15000 && smartbandTakenOffTime > 0){ //15 seconds
        Serial.println("Device is not worn, proceeding to IDLE");
        currentState = IDLE;
        stateChanged = true;
        smartbandTakenOffTime = 0;
    }
        
    else if(smartbandTakenOffTime > 0 && irValue>=5000){
        smartbandTakenOffTime = 0;
    }
        

    // Saving the data to file in a new row
    dataFile.print(irValue);
    dataFile.print(",");
    dataFile.print(redValue);
    dataFile.print(",");
    dataFile.print(ax);
    dataFile.print(",");
    dataFile.print(ay);
    dataFile.print(",");
    dataFile.println(az);
    //dataFile.print(",");
    // dataFile.print(gx);
    // dataFile.print(",");
    // dataFile.print(gy);
    // dataFile.print(",");
    // dataFile.println(gz);
    
}

void endTraining() {
    dataFile.close();
    //TODO: check if closed properly
}

bool isWorn() {
    long irValue = sensorMAX.getIR();
    if (irValue < 5000) { //TODO: Check if this value is good
        return false; 
    } else {
        return true;
    }
}

//Wifi connection withheld, all data is being send through BLE. 
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


//Simple callbacks for informing the esp32 that someone connected through BLE
class ServerCallbacks: public NimBLEServerCallbacks {
    void onConnect(NimBLEServer* pServer) override {
        deviceConnected = true;
        bleDisconnection = 0;
        Serial.println("Client connected.");
    };

    void onDisconnect(NimBLEServer* pServer) override {
        bleDisconnection = millis();
        deviceConnected = false;
        Serial.println("Client disconnected.");
    }
};

//Callback to connect to WiFi, works, but not used
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

//Callback for sending training files to apps
class FileTransferCallbacks : public NimBLECharacteristicCallbacks {
private:
    File file;
    size_t chunkSize = 512; // based on MTU size and performance
    size_t fileSize = 0;
    size_t bytesSent = 0;
    bool transferInProgress = false;
    String currentFileName;
    bool sendEndMessage = false;

public:
    void onRead(NimBLECharacteristic* pCharacteristic) override {
        if (!transferInProgress && !sendEndMessage) {
            if (!SD.begin(CS_PIN)) {
                Serial.println("Card Mount Failed");
                currentState = ERROR;
                stateChanged = true;
                return;
            }
            sendNextFile();
        }

        if (transferInProgress) {
            sendNextChunk(pCharacteristic);
        } else if (sendEndMessage){
            sendEnd(pCharacteristic);
        }
    }

    void startFileTransfer(const char* fileName) {
        delay(10);
        
        currentFileName = String(fileName);

        file = SD.open(fileName, FILE_READ);
        
        if (!file) {
            Serial.println("Failed to open file for reading");
            sendNextFile(); //todo: check if it is working properly. 
        }

        fileSize = file.size();
        bytesSent = 0;
        transferInProgress = true;

        Serial.println("File transfer started");
    }

    void sendNextChunk(NimBLECharacteristic* pCharacteristic) {
        if (!file || !file.available()) {
            file.close();
            SD.end();
            transferInProgress = false;
            Serial.println("File transfer completed");
            return;
        }

        // Read the next chunk
        uint8_t buffer[chunkSize];
        size_t bytesRead = file.read(buffer, chunkSize);

        // Send the chunk
        pCharacteristic->setValue(buffer, bytesRead);
        pCharacteristic->notify();

        bytesSent += bytesRead;
        Serial.printf("Sent %d/%d bytes\n", bytesSent, fileSize);

        if (bytesSent >= fileSize) {
            file.close();
            transferInProgress = false;
            sendEndMessage = true;
            Serial.println("File transfer completed");
        }
    }

    void sendEnd(NimBLECharacteristic* pCharacteristic) {
        const char* endMessage = "END";
        pCharacteristic->setValue((uint8_t*)endMessage, strlen(endMessage));
        pCharacteristic->notify();

        Serial.println("Sent 'END' message");

        // Reset flags to start the sending process again
        sendEndMessage = false;
        transferInProgress = false;
    }

    void deleteFile() {
        if (SD.exists(currentFileName.c_str())) {
            SD.remove(currentFileName.c_str());
            Serial.print("File deleted from SD card. File deleted: ");
            Serial.println(currentFileName);
        } else {
            Serial.println("File not found on SD card.");
        }
    }

    void sendNextFile() {

        char filename[32];
        currentFileIndex = 1;

        while (true) {
            snprintf(filename, sizeof(filename), "/training_%d.csv", currentFileIndex);
            if (SD.exists(filename)) {
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
};

// Characteristic response for successful file transmission
class ConfirmationCallbacks : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic* pCharacteristic) override {
        std::string confirmation = pCharacteristic->getValue();

        if (confirmation == "OK"){
            Serial.println("Confirmation received from app. Deleting file.");
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
    pinMode(BUTTON_PIN, INPUT_PULLUP); //using an inner built-in resistor on ESP32

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

    // SD card module initalization
    while (!SD.begin(CS_PIN)) {
        Serial.println("Card Mount Failed");
        signalError(ERROR_SD_NOT_FOUND);
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
    //todo: check if it is still neccessary
    dataFile.println("IR, Red, Ax, Ay, Az, Gx, Gy, Gz");
    dataFile.close();

    //IDLE MODE IN THE BEGINNING

    digitalWrite(LED_PIN_IDLE, HIGH);
    digitalWrite(LED_PIN_BLE, LOW);
    digitalWrite(LED_PIN_TRAINING, LOW);
    SD.end();
    Serial.println("Initial idle mode");
}

void loop() {

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
                    // shor press, but change only if 1 second has passed since the last state change
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
        stateChanged = false;
        switch (currentState) {
            case IDLE: // Idle state, no data are collected from sensors
                digitalWrite(LED_PIN_IDLE, HIGH);
                digitalWrite(LED_PIN_BLE, LOW);
                digitalWrite(LED_PIN_TRAINING, LOW);
                
                endTraining();  // closing the CSV file
                checkIfIsWorn = false;
                //convertCsvToJson(filename, filenameJSON);  // JSON conversion, moved to mobile apps      

                if (pServer != nullptr || pAdvertising != nullptr || pService != nullptr) {
                    NimBLEDevice::deinit(true);
                    pServer = nullptr;
                    pService = nullptr;
                    pAdvertising = nullptr;
                }

                if (ssidCallbacks != nullptr) {
                    delete ssidCallbacks;
                    ssidCallbacks = nullptr;
                }

                if (passwordCallbacks != nullptr) {
                    delete passwordCallbacks;
                    passwordCallbacks = nullptr;
                }

                if (fileTransferCallbacks != nullptr) {
                    delete fileTransferCallbacks;
                    fileTransferCallbacks = nullptr;
                }

                if (confirmationCallbacks != nullptr) {
                    delete confirmationCallbacks;
                    confirmationCallbacks = nullptr;
                }

                if (timeSyncCallbacks != nullptr) {
                    delete timeSyncCallbacks;
                    timeSyncCallbacks = nullptr;
                }

                SD.end();
                
                sensorMAX.setPulseAmplitudeRed(0);
                sensorMAX.setPulseAmplitudeIR(0);
                sensorMPU.setSleepEnabled(true);

                Serial.println("Idle mode");

                esp_sleep_enable_ext0_wakeup(GPIO_NUM_13, 0);  // 0 = LOW level to trigger wakeup

                esp_light_sleep_start();  // Device will sleep here until GPIO13 wakes it
                break;

            case BLE: // BLE state, BLE server is on, should broadcast
                digitalWrite(LED_PIN_IDLE, LOW);
                digitalWrite(LED_PIN_BLE, HIGH);
                digitalWrite(LED_PIN_TRAINING, LOW);

                if (!SD.begin(CS_PIN)) {
                    Serial.println("Card Mount Failed");
                    currentState = ERROR;
                    stateChanged = true;
                }

                endTraining();  // closing the CSV file

                checkIfIsWorn = false;

                // Todo: connect mobile phone to ESP32 during this state

                if (pAdvertising->isAdvertising() == false or pAdvertising == nullptr or pServer == nullptr or pService == nullptr) {
                    
                NimBLEDevice::init("ESP32 D3K");

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
                                            NIMBLE_PROPERTY::WRITE);
                pTimeSyncCharacteristic->setCallbacks(timeSyncCallbacks);

                pService->start();

                pAdvertising = NimBLEDevice::getAdvertising();
                pAdvertising->addServiceUUID(SERVICE_UUID);
                pAdvertising->start();
                }

                bleDisconnection = millis();

                sensorMAX.setPulseAmplitudeRed(0);
                sensorMAX.setPulseAmplitudeIR(0);
                sensorMPU.setSleepEnabled(true);

                Serial.println("BLE mode activated and advertising started");
            break;

            case TRAINING: // Exercise state, collect data from sensors

                sensorMAX.setup(); // Configure sensor with default settings
                sensorMAX.setPulseAmplitudeRed(60);
                //sensorMAX.setPulseAmplitudeIR(0x0A); //dont need to set it, the default setting have IR powered already
                sensorMAX.setSampleRate(0x18);
                sensorMAX.setFIFOAverage(8);
                sensorMPU.setSleepEnabled(false);

                digitalWrite(LED_PIN_IDLE, LOW);
                digitalWrite(LED_PIN_BLE, LOW);
                digitalWrite(LED_PIN_TRAINING, HIGH);

                if (pServer != nullptr || pAdvertising != nullptr || pService != nullptr) {
                    NimBLEDevice::deinit(true);
                    pServer = nullptr;
                    pService = nullptr;
                    pAdvertising = nullptr;
                }

                if (ssidCallbacks != nullptr) {
                    delete ssidCallbacks;
                    ssidCallbacks = nullptr;
                }

                if (passwordCallbacks != nullptr) {
                    delete passwordCallbacks;
                    passwordCallbacks = nullptr;
                }

                if (fileTransferCallbacks != nullptr) {
                    delete fileTransferCallbacks;
                    fileTransferCallbacks = nullptr;
                }

                if (confirmationCallbacks != nullptr) {
                    delete confirmationCallbacks;
                    confirmationCallbacks = nullptr;
                }

                if (timeSyncCallbacks != nullptr) {
                    delete timeSyncCallbacks;
                    timeSyncCallbacks = nullptr;
                }

                if (!SD.begin(CS_PIN)) {
                    Serial.println("Card Mount Failed");
                    currentState = ERROR;
                    stateChanged = true;
                } else {
                    Serial.println("Training mode - SD logging started");
                    checkIfIsWorn = true;
                    trainingStartTime = millis();
                }
                break;

            case ERROR:
                while(!SD.begin(CS_PIN)) {
                    Serial.println("Card Mount Failed");
                    signalError(ERROR_SD_NOT_FOUND);
                }

                currentState = IDLE;
                stateChanged = true;

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
            if(millis() - trainingStartTime >= 15000){ //15 seconds
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
        collectAndSaveData();

        do{
            sampleEndTime = millis();
            //Serial.println(sampleEndTime - sampleStartTime);
        } while (sampleEndTime - sampleStartTime < samplingRateInMillis); // 10 miliseconds = 100Hz sampling rate
        
    }
    
    if(currentState == BLE && !deviceConnected){
        if(millis() - bleDisconnection >= bleTimeout){
            Serial.println("No connection for a longer time, switching to IDLE");
            currentState = IDLE;
            stateChanged = true;
        }
    }
}
