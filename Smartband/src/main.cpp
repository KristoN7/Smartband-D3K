#include <Arduino.h>

//BLE through NimBLE framework
#include <NimBLEDevice.h>
#include <NimBLEUtils.h>
#include <NimBLEServer.h>

//Time, note: send bytes in Little Endian
#include <time.h>

// UUIDs for BLE Service and Characteristics
#define SERVICE_UUID        "12345678-1234-1234-1234-123456789111"
#define MESSAGE_TRANSFER_UUID "e2e3f5a4-8c4f-11eb-8dcd-0242ac130011"
#define CONFIRMATION_UUID "e2e3f5a4-8c4f-11eb-8dcd-0242ac130006"  // confirmation from app regarding file transmission success
#define TIME_SYNC_UUID "e2e3f5a4-8c4f-11eb-8dcd-0242ac130007"


//BLUE LED
#define LED_PIN_BLE 2

NimBLEServer* pServer = nullptr;
NimBLEService* pService = nullptr; 
NimBLEAdvertising* pAdvertising = nullptr; 
NimBLECharacteristic* pMessageTransferCharacteristic = nullptr;
NimBLECharacteristic* pConfirmationCharacteristic = nullptr;
NimBLECharacteristic* pTimeSyncCharacteristic = nullptr;

NimBLECharacteristicCallbacks* fileTransferCallbacks = nullptr;
NimBLECharacteristicCallbacks* confirmationCallbacks = nullptr;
NimBLECharacteristicCallbacks* timeSyncCallbacks = nullptr;
bool deviceConnected = false;

//For time counter in UTC format
unsigned long syncedTime = 0; // time synchronized using app's time (UNIX timestamp)
unsigned long lastSyncMillis = 0; // time in milliseconds when synchronization occurred
unsigned long time1; //for displaying time in loop()

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

class MessageTransferCallbacks : public NimBLECharacteristicCallbacks {
private:
    String message = "ABCDEFGHIJKLMNOPQRSTUVWXYZ";
    size_t chunkSize = 2; // based on MTU size and performance
    size_t messageSize = 0;
    size_t bytesSent = 0;
    bool transferInProgress = false;
    bool sendEndMessage = false;

public:
    void onRead(NimBLECharacteristic* pCharacteristic) override {
        if (!transferInProgress && !sendEndMessage) {
            startMessageTransfer(message);
        }

        if (transferInProgress) {
            sendNextChunk(pCharacteristic);
        } else if (sendEndMessage) {
            sendEnd(pCharacteristic);
        }
    }

    void startMessageTransfer(const String& messageToSend) {
        uint8_t tempBuffer[messageToSend.length() + 1]; // Create a buffer for the bytes (including null terminator)
        messageToSend.getBytes(tempBuffer, messageToSend.length() + 1); // Extract the string as bytes
        messageSize = strlen((char*)tempBuffer); // Get the actual byte size

        bytesSent = 0;
        transferInProgress = true;

        Serial.println("Message transfer started");
    }

    void sendNextChunk(NimBLECharacteristic* pCharacteristic) {
        if (message.isEmpty()) {
            transferInProgress = false;
            Serial.println("Message is empty");
            return;
        }

        // Calculate the number of bytes left to send
        size_t bytesRemaining = messageSize - bytesSent;
        size_t bytesToSend = min(chunkSize, bytesRemaining);

        // Copy the next chunk of data into a buffer
        uint8_t buffer[chunkSize];
        message.substring(bytesSent, bytesSent + bytesToSend).getBytes(buffer, bytesToSend + 1); // +1 for null terminator

        // Send the chunk
        pCharacteristic->setValue(buffer, bytesToSend);
        pCharacteristic->notify();

        bytesSent += bytesToSend;
        Serial.printf("Sent %d/%d bytes\n", bytesSent, messageSize);

        if (bytesSent >= messageSize) {
            transferInProgress = false;
            sendEndMessage = true; // Set flag to send "END" message next
            Serial.println("Message transfer completed");
        }
    }

    void sendEnd(NimBLECharacteristic* pCharacteristic) {
        const char* endMessage = "END";
        pCharacteristic->setValue((uint8_t*)endMessage, strlen(endMessage));
        pCharacteristic->notify();

        Serial.println("Sent 'END' message");

        // Reset flags to start the process again
        sendEndMessage = false;
        transferInProgress = false;
    }
};

// Characteristic response for successful message transmission
class ConfirmationCallbacks : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic* pCharacteristic) override {
        std::string confirmation = pCharacteristic->getValue();

        if (confirmation == "OK"){
            Serial.println("Confirmation received from app.");
        } else {
            Serial.println("Invalid confirmation received.");
        }
    }
};

// Characteristic response for successful time data transmission
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

unsigned long getCurrentTime() {
    if (syncedTime == 0) {
        // Not synchronized
        // Serial.println("Time not synchronized");
        return 0;
    }

    unsigned long elapsedMillis = millis() - lastSyncMillis;
    unsigned long currentTime = syncedTime + elapsedMillis / 1000; // Adding elapsed timed to synced time
    return currentTime;
}


void setup() {
    Serial.begin(115200);
    Serial.println("Starting setup...");

    pinMode(LED_PIN_BLE, OUTPUT);
    digitalWrite(LED_PIN_BLE, HIGH);

    if (pAdvertising->isAdvertising() == false or pAdvertising == nullptr or pServer == nullptr or pService == nullptr) {
                    
        NimBLEDevice::init("ESP32_Smartband_mini");

        pServer = NimBLEDevice::createServer();
        pServer->setCallbacks(new ServerCallbacks());

        pService = pServer->createService(SERVICE_UUID);

    
        pMessageTransferCharacteristic = pService->createCharacteristic(
            MESSAGE_TRANSFER_UUID,
            NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY
        );
        pMessageTransferCharacteristic->setCallbacks(new MessageTransferCallbacks());

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

    pService->start();

    pAdvertising = NimBLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->start();
    }

    Serial.println("BLE mode activated and advertising started");

    time1 = millis();
}

void loop() {
    if(getCurrentTime() > 0 && millis() - time1 > 3000){
        Serial.println(getCurrentTime());
        time1 = millis();
    }
}
