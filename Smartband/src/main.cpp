#include <Arduino.h>

//BLE through NimBLE framework
#include <NimBLEDevice.h>
#include <NimBLEUtils.h>
#include <NimBLEServer.h>

// UUIDs for BLE Service and Characteristics
#define SERVICE_UUID        "12345678-1234-1234-1234-123456789113"
#define MESSAGE_TRANSFER_UUID "e2e3f5a4-8c4f-11eb-8dcd-0242ac130013"

//BLUE LED
#define LED_PIN_BLE 2

NimBLEServer* pServer = nullptr;
NimBLEService* pService = nullptr; 
NimBLEAdvertising* pAdvertising = nullptr; 
NimBLECharacteristic* pMessageTransferCharacteristic = nullptr;
bool deviceConnected = false;

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
<<<<<<< HEAD
    String message = "TEST123 BartekEdition ABECADLO Z PIECA SPADLO stworzył konstruktor domyślny. Doman i Krycha collab. TELEINFORMATYKA OPASKA PROJEKT D3000 ŻÓŁT ĄĆĘĘŚŁĆŻŹÓŃÓŚŃŻŹŁĆĄŚĘÓŚĄ.";
=======
    String message = "Lewandowski ABECADLO Z PIECA SPADLO stworzył konstruktor domyślny. Doman i Krycha collab. TELEINFORMATYKA OPASKA PROJEKT D3000 ŻÓŁT ĄTROLOÓŚŃŻŹŁĆĄŚĘÓŚĄ.";
>>>>>>> d950e92219c64e6389c9286e829e3b50dcde574d
    size_t chunkSize = 32; // based on MTU size and performance
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

void setup() {
    Serial.begin(115200);
    Serial.println("Starting setup...");

    pinMode(LED_PIN_BLE, OUTPUT);
    digitalWrite(LED_PIN_BLE, HIGH);

    if (pAdvertising->isAdvertising() == false or pAdvertising == nullptr or pServer == nullptr or pService == nullptr) {
                    
        NimBLEDevice::init("ESP32_Smartband_1");

        pServer = NimBLEDevice::createServer();
        pServer->setCallbacks(new ServerCallbacks());

        pService = pServer->createService(SERVICE_UUID);

    
        pMessageTransferCharacteristic = pService->createCharacteristic(
            MESSAGE_TRANSFER_UUID,
            NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY
            );
        pMessageTransferCharacteristic->setCallbacks(new MessageTransferCallbacks());

    pService->start();

    pAdvertising = NimBLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->start();
    }

    Serial.println("BLE mode activated and advertising started");
}

void loop() {
    }