#include <Arduino.h>

//BLE through NimBLE framework
#include <NimBLEDevice.h>
#include <NimBLEUtils.h>
#include <NimBLEServer.h>

//Time, note: send bytes in Little Endian
#include <time.h>

// UUIDs for BLE Service and Characteristics
#define SERVICE_UUID        "12345678-1234-1234-1234-123456789113"
#define MESSAGE_TRANSFER_UUID "e2e3f5a4-8c4f-11eb-8dcd-0242ac130013"
#define CONFIRMATION_UUID "e2e3f5a4-8c4f-11eb-8dcd-0242ac130006"  // confirmation from app regarding file transmission success
#define TIME_SYNC_UUID "e2e3f5a4-8c4f-11eb-8dcd-0242ac130007"

#define BLE_ATT_MTU_MAX 512

//BLUE LED
#define LED_PIN_BLE 2

uint8_t sampleData[] = {
    0x10, 0x0E, 0x00, 0x00,   //Unix timestamp
    0x7F, 0x13, 0x00, 0x00,   // IR 
    0x77, 0x24, 0x00, 0x00,   // Red
    0x34, 0x04,               // accX
    0xB8, 0x04,               // accY
    0x28, 0x40,                // accZ
    0x49, 0x13, 0x00, 0x00,   
    0x43, 0x22, 0x00, 0x00,   
    0x38, 0x04,               
    0x64, 0x04,               
    0x1C, 0x40,               
    0xC1, 0x12, 0x00, 0x00,   
    0x68, 0x1F, 0x00, 0x00,   
    0x54, 0x04,               
    0xE0, 0x04,               
    0x6C, 0x3F,               
    0x06, 0x12, 0x00, 0x00,   
    0x9E, 0x1C, 0x00, 0x00,   
    0x60, 0x04,               
    0xF4, 0x04,               
    0xA8, 0x3F,               
    0x57, 0x11, 0x00, 0x00,   
    0x49, 0x1A, 0x00, 0x00,   
    0xB0, 0x04,               
    0xC4, 0x04,               
    0xF4, 0x3E,               
    0xD5, 0x10, 0x00, 0x00,   
    0x8E, 0x18, 0x00, 0x00,   
    0x98, 0x04,               
    0xF4, 0x04,               
    0xD8, 0x3F,               
    0x67, 0x10, 0x00, 0x00,   
    0x7C, 0x17, 0x00, 0x00,   
    0x84, 0x04,               
    0xF0, 0x04,               
    0x38, 0x40,               
    0x1E, 0x10, 0x00, 0x00,   
    0xB7, 0x16, 0x00, 0x00,   
    0x14, 0x04,               
    0xA4, 0x04,               
    0x48, 0x40,               
    0xCB, 0x0F, 0x00, 0x00,   
    0x2B, 0x16, 0x00, 0x00,   
    0x48, 0x04,               
    0xF0, 0x04,               
    0xBC, 0x3F,               
    0x60, 0x0F, 0x00, 0x00,   
    0x7C, 0x15, 0x00, 0x00,   
    0x4C, 0x04,               
    0x0C, 0x05,               
    0x18, 0x40,               
    0xE1, 0x0E, 0x00, 0x00,   
    0xC6, 0x14, 0x00, 0x00,   
    0x3C, 0x04,               
    0x20, 0x05,               
    0x08, 0x40,               
};

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

uint16_t currentMTUSize = 24;

//For time counter in UTC format
unsigned long syncedTime = 0; // time synchronized using app's time (UNIX timestamp)
unsigned long lastSyncMillis = 0; // time in milliseconds when synchronization occurred
unsigned long time1; //for displaying time in loop()

class ServerCallbacks: public NimBLEServerCallbacks {
    void onConnect(NimBLEServer* pServer) override {
        deviceConnected = true;
        Serial.println("Client connected.");
    }

    void onMTUChange(uint16_t MTU, ble_gap_conn_desc* desc) override {
        Serial.print("MTU size updated to: ");
        Serial.println(MTU);  // This prints the negotiated MTU size
        currentMTUSize = MTU;
    }

    void onDisconnect(NimBLEServer* pServer) override {
        deviceConnected = false;
        Serial.println("Client disconnected.");
    }

};

class MessageTransferCallbacks : public NimBLECharacteristicCallbacks {
private:
    //String message = "2024-10-21 09:00:00\nIR,RED,Ax,Ay,Az\n6213,8447,1212,1404,16320\n8623,13290,1432,1384,16096\n11467,19230,1516,1248,16144\n14751,26472,1148,1256,16012\n18845,36170,956,1512,16276\n24059,48666,812,1428,16360\n30245,63505,920,1332,16508\n36968,79137,964,1412,16408\n43246,93696,952,1332,16276\n48949,108681,1056,1300,16052\n54289,123951,1192,1348,16148\n59101,137182,988,1404,16292\n62486,147352,820,1316,16464\n65579,155525,872,1372,16252\n67712,160586,1052,1424,16304\n69141,163993,1104,1344,16184\n69965,165873,1048,1436,16356\n70056,165437,788,1472,16484\n68354,160624,788,1352,16344\n63723,148544,1072,1328,16192\n57054,133495,1096,1336,16208\n51498,123223,1012,1320,16092\n48765,119111,1392,1304,16052\n48788,121747,1408,1328,15996\n52521,136009,1220,1416,16368\n61381,156174,1004,1456,16132\n72947,159878,1116,1344,15900\n82711,172704,1676,1112,16888\n85739,176456,636,1492,15776\n90185,181719,-356,1576,16676\n96184,185054,1300,1224,16100\n98153,186204,1324,1428,16172\n99199,186981,980,1412,16368\n100112,187606,732,1296,16228\n100827,188121,588,1272,16224\n101378,188575,600,1416,16520\n101849,188938,380,1444,16388\n102201,189158,684,1464,16356\n102473,189322,1028,1364,16100\n102738,189470,1180,1308,16180\n102940,189531,780,1332,16424\n103085,189539,856,1368,16296\n103212,189571,1240,1240,16268\n103298,189551,1112,1288,16176\n103381,189565,1048,1460,16232\n103459,189583,1064,1328,16300\n103557,189662,1032,1292,16200\n103643,189704,980,1360,16352\n103716,189715,1120,1268,16268\n103790,189759,1128,1332,16264\n103852,189827,1032,1356,16296\n103903,189857,1180,1304,16236\n103962,189913,1324,1328,16300\n104029,189951,1148,1360,16168\n104070,189973,1060,1392,16220\n104130,190011,976,1444,16360\n104137,190033,836,1408,16364\n104156,190059,820,1420,16424\n104186,190065,832,1424,16220\n104230,190081,944,1360,16256\n104266,190096,1044,1188";
    size_t chunkSize = currentMTUSize - 3; // based on MTU size and performance
    size_t sampleSize = sizeof(sampleData);
    size_t bytesSent = 0;
    bool transferInProgress = false;
    bool sendEndMessage = false;

public:
    void onRead(NimBLECharacteristic* pCharacteristic) override {
        chunkSize = currentMTUSize - 3;
        if (!transferInProgress && !sendEndMessage) {
            startMessageTransfer();
        }

        if (transferInProgress) {
            sendNextChunk(pCharacteristic);
        } else if (sendEndMessage) {
            sendEnd(pCharacteristic);
        }
    }

    void startMessageTransfer() {
        bytesSent = 0;
        transferInProgress = true;

        Serial.println("Message transfer started");
    }

    void sendNextChunk(NimBLECharacteristic* pCharacteristic) {
        // Calculate the number of bytes left to send
        size_t bytesRemaining = sampleSize - bytesSent;
        size_t bytesToSend = min(chunkSize, bytesRemaining);

        // Send the chunk
        pCharacteristic->setValue(&sampleData[bytesSent], bytesToSend);
        pCharacteristic->notify();

        bytesSent += bytesToSend;
        Serial.printf("Sent %d/%d bytes\n", bytesSent, sampleSize);

        if (bytesSent >= sampleSize) {
            transferInProgress = false;
            sendEndMessage = true; // Set flag to send "END" message next
            Serial.println("Message transfer completed");
        }
    }

    void sendEnd(NimBLECharacteristic* pCharacteristic) {
        uint8_t endMessage[16] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
        pCharacteristic->setValue(endMessage, sizeof(endMessage));
        pCharacteristic->notify();

        Serial.println("Sent ending message");

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
        NimBLEDevice::setMTU(512);

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
        if (deviceConnected) {
            // In case you want to retrieve it again
            Serial.print("Current MTU size: ");
            Serial.println(currentMTUSize);
        }
    }
}
