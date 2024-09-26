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

volatile int currentState = 0; //0 - IDLE, 1 - BLE, 2 - Collecting and saving sensor data 
volatile bool stateChanged = false;

// UUIDs for BLE Service and Characteristics
#define SERVICE_UUID        "12345678-1234-1234-1234-123456789012"
#define SSID_CHAR_UUID      "e2e3f5a4-8c4f-11eb-8dcd-0242ac130003"
#define PASSWORD_CHAR_UUID  "e2e3f5a4-8c4f-11eb-8dcd-0242ac130004"
#define FILE_TRANSFER_UUID "e2e3f5a4-8c4f-11eb-8dcd-0242ac130005"
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

const int debounceDelay = 2000; // to eliminate debouncing effect on physical switch (ms)
volatile unsigned long lastDebounceTime = 0;

//The difference between these will define the sample rate (initially 10 miliseconds)
unsigned long sampleStartTime = 0;
unsigned long sampleEndTime = 0;
const int samplingRateInMillis = 10;

//For time counter in UTC format
unsigned long syncedTime = 0; // time synchronized using app's time (UNIX timestamp)
unsigned long lastSyncMillis = 0; // time in milliseconds when synchronization occurred

//JSON (withheld)

/*
StaticJsonDocument<2048> jsonDocument;
JsonArray irArray = jsonDocument.createNestedArray("IR");
JsonArray redArray = jsonDocument.createNestedArray("Red");
JsonArray axArray = jsonDocument.createNestedArray("Ax");
JsonArray ayArray = jsonDocument.createNestedArray("Ay");
JsonArray azArray = jsonDocument.createNestedArray("Az");
JsonArray gxArray = jsonDocument.createNestedArray("Gx");
JsonArray gyArray = jsonDocument.createNestedArray("Gy");
JsonArray gzArray = jsonDocument.createNestedArray("Gz");
int readingsCounter = 0;
*/

//FUNCTIONS

void IRAM_ATTR handleButtonPress() { //changing between states is possible after 2 seconds delay
    unsigned long currentTime = millis();
    if((currentTime - lastDebounceTime) > debounceDelay){ 
        currentState = (currentState + 1) % 3; // Possible states: 0 - idle, 1 - BLE, 2 - exercise 
        stateChanged = true;
        lastDebounceTime = currentTime;
    }
}

unsigned long getCurrentTime() {
    if (syncedTime == 0) {
        // Not synchronized
        Serial.println("Time not synchronized");
        return 0;
    }

    unsigned long elapsedMillis = millis() - lastSyncMillis;
    unsigned long currentTime = syncedTime + elapsedMillis / 1000; // Dodaj upływ czasu do zsynchronizowanego czasu
    return currentTime;
}

void startTraining() {
    unsigned long trainingStartTime = getCurrentTime() + 3600; //3600, because Poland is in UTC+1 timezone

    do {
          snprintf(filename, sizeof(filename), "/training_%d.csv", fileIndex++); 
    } while (SD.exists(filename));
        
    //do {
    //      snprintf(filenameJSON, sizeof(filename), "/training_%d.json", fileIndex++); 
    //} while (SD.exists(filename));

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
     dataFile.printf("Training start time (UTC): %s\n", timeString); //UTC
    dataFile.printf("%lu\n", trainingStartTime); //UNIX
    dataFile.println("IR,RED,Ax,Ay,Az,Gx,Gy,Gz");
}

void collectAndSaveData() {
    
    long irValue = sensorMAX.getIR();
    long redValue = sensorMAX.getRed();

    int16_t ax, ay, az; //accelerometer data
    int16_t gx, gy, gz; //gyroscope data
    sensorMPU.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    // Saving the data to file in a new row
    dataFile.print(irValue);
    dataFile.print(",");
    dataFile.print(redValue);
    dataFile.print(",");
    dataFile.print(ax);
    dataFile.print(",");
    dataFile.print(ay);
    dataFile.print(",");
    dataFile.print(az);
    dataFile.print(",");
    dataFile.print(gx);
    dataFile.print(",");
    dataFile.print(gy);
    dataFile.print(",");
    dataFile.println(gz);
    
}

void endTraining() {
    dataFile.close();
    //TODO: check if closed properly
}


//CSV to JSON withheld on ESP32 because of memory constraints. Moved to the app. Also this method never worked, becuase of limited ways to modify json files
/*
void convertCsvToJson(const char* csvFilename, const char* jsonFilename) {
    File csvFile = SD.open(csvFilename, FILE_READ);
    if (!csvFile) {
        Serial.println("Failed to open CSV file for reading");
        return;
    }
    File jsonFile = SD.open(jsonFilename, FILE_WRITE);
    if (!jsonFile) {
        Serial.println("Failed to open JSON file for writing");
        csvFile.close();
        return;
    }
    jsonFile.println("{");
    jsonFile.println("\"IR\": [");
    jsonFile.println("\"RED\": [");
    jsonFile.println("\"Ax\": [");
    jsonFile.println("\"Ay\": [");
    jsonFile.println("\"Az\": [");
    jsonFile.println("\"Gx\": [");
    jsonFile.println("\"Gy\": [");
    jsonFile.println("\"Gz\": [");

    while (csvFile.available()) {
        String line = csvFile.readStringUntil('\n');
        line.trim();
        if (line.length() == 0) continue;  
        char lineArray[line.length() + 1];
        line.toCharArray(lineArray, line.length() + 1);
        int index = 0;
        char* value = strtok(lineArray, ",");
        while (value != nullptr) {
            jsonFile.print(value);
            value = strtok(nullptr, ",");
            if (value != nullptr) {
                jsonFile.print(",");
            }
            index++;
        }
        jsonFile.println();
    }

    jsonFile.println("]");
    jsonFile.println("}");
    csvFile.close();
    jsonFile.close();
    Serial.println("CSV file successfully converted to JSON");
}
*/

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
        Serial.println("Client connected.");
    };

    void onDisconnect(NimBLEServer* pServer) override {
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

// Characteristic responde for file successful file transmission
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
    pinMode(BUTTON_PIN, INPUT);

    attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), handleButtonPress, RISING);

    // The initial LED state 
    digitalWrite(LED_PIN_IDLE, LOW);
    digitalWrite(LED_PIN_BLE, LOW);
    digitalWrite(LED_PIN_TRAINING, LOW);

    // MAX30102 initialization
    if (!sensorMAX.begin(Wire, I2C_SPEED_FAST)) {
        Serial.println("MAX30102 not found. Check wiring.");
        while (1);
    }
    sensorMAX.setup(); // Configure sensor with default settings
    sensorMAX.setPulseAmplitudeRed(0x0A); // Turn Red LED to low to indicate sensor is running
    sensorMAX.setPulseAmplitudeIR(0x0A); // Turn IR LED to low to indicate sensor is running
    sensorMAX.setSampleRate(0x18);
    sensorMAX.setFIFOAverage(8);

    // MPU6050 initialization
    sensorMPU.initialize();
    if (!sensorMPU.testConnection()) {
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
    //todo: check if it is still neccessary
    dataFile.println("IR, Red, Ax, Ay, Az, Gx, Gy, Gz");
    dataFile.close();

    //IDLE MODE IN THE BEGINNING

    digitalWrite(LED_PIN_IDLE, HIGH);
    digitalWrite(LED_PIN_BLE, LOW);
    digitalWrite(LED_PIN_TRAINING, LOW);
    SD.end();
    Serial.println("Idle mode");
}

void loop() {
    if (stateChanged) {
        stateChanged = false;
        switch (currentState) {
            case 0: // Idle state, no data are collected from sensors
                digitalWrite(LED_PIN_IDLE, HIGH);
                digitalWrite(LED_PIN_BLE, LOW);
                digitalWrite(LED_PIN_TRAINING, LOW);
                
                endTraining();  // closing the CSV file
                //convertCsvToJson(filename, filenameJSON);  // JSON conversion, moved to mobile apps      

                // Disable all devices
                NimBLEDevice::deinit(true);
                pServer = nullptr;
                pService = nullptr;
                pAdvertising = nullptr;

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

                SD.end();
                
                // Todo: disable MAX30102 and MPU6050

                Serial.println("Idle mode");
                break;

            case 1: // BLE state, BLE server is on, should broadcast
                digitalWrite(LED_PIN_IDLE, LOW);
                digitalWrite(LED_PIN_BLE, HIGH);
                digitalWrite(LED_PIN_TRAINING, LOW);

                if (!SD.begin(CS_PIN)) {
                    Serial.println("Card Mount Failed");
                }

                // Todo: connect mobile phone to ESP32 during this state

                if (pAdvertising->isAdvertising() == false or pAdvertising == nullptr or pServer == nullptr or pService == nullptr) {
                    
                NimBLEDevice::init("D3K Smartband");

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
                //Todo, adjust state 4
                break;
        }
    }

    if(currentState == 2){

        sampleStartTime = millis();
        collectAndSaveData();

        do{
            sampleEndTime = millis();
            //Serial.println(sampleEndTime - sampleStartTime);
        } while (sampleEndTime - sampleStartTime < samplingRateInMillis); // 10 miliseconds = 100Hz sampling rate
        
        //this methord saved data to json in batches, creating multiple json objects in one file, changed to csv files
        // // Fetch data from MAX30102 sensor 
        // long irValue = particleSensor.getIR();
        // long redValue = particleSensor.getRed();

        // // Fetch data from MPU6050 sensor
        // int16_t ax, ay, az;
        // int16_t gx, gy, gz;
        // sensorMPU.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

        // // Add data to corresponding arrays
        // irArray.add(irValue);
        // redArray.add(redValue);
        // axArray.add(ax);
        // ayArray.add(ay);
        // azArray.add(az);
        // gxArray.add(gx);
        // gyArray.add(gy);
        // gzArray.add(gz);

        // readingsCounter++;

        // // Serial print the raw data
        // Serial.print("IR: ");
        // Serial.print(irValue);
        // Serial.print("\tRed: ");
        // Serial.print(redValue);
        // Serial.print("\tAx: ");
        // Serial.print(ax);
        // Serial.print("\tAy: ");
        // Serial.print(ay);
        // Serial.print("\tAz: ");
        // Serial.print(az);
        // Serial.print("\tGx: ");
        // Serial.print(gx);
        // Serial.print("\tGy: ");
        // Serial.print(gy);
        // Serial.print("\tGz: ");
        // Serial.println(gz);

        // if (readingsCounter >= 100) {  //100 samples are equal to 1 second of work time
        //     File dataFile = SD.open(filename, FILE_APPEND);
        //     if (dataFile) {
        //         serializeJson(jsonDocument, dataFile);
        //         dataFile.println(); 
        //         dataFile.close();
        //         Serial.println("Batch of data written to SD card in JSON format");

        //         // Clear arrays for next batch
        //         jsonDocument.clear();
        //         irArray = jsonDocument.createNestedArray("IR");
        //         redArray = jsonDocument.createNestedArray("Red");
        //         axArray = jsonDocument.createNestedArray("Ax");
        //         ayArray = jsonDocument.createNestedArray("Ay");
        //         azArray = jsonDocument.createNestedArray("Az");
        //         gxArray = jsonDocument.createNestedArray("Gx");
        //         gyArray = jsonDocument.createNestedArray("Gy");
        //         gzArray = jsonDocument.createNestedArray("Gz");
        //         readingsCounter = 0;
        //     } else {
        //         Serial.println("Failed to open file for appending");
        //     }
        // }

        //delay(10); // Change to milis() for proper sampling rate
    }
}