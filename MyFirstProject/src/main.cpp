#include <Arduino.h>
#include <Wire.h>
#include <MAX30105.h>
#include <MPU6050.h>

MAX30105 particleSensor;
MPU6050 mpu;

void setup() {
    Serial.begin(115200);
    Wire.begin();

    // Inicjalizacja MAX30102
    if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
        Serial.println("MAX30102 not found. Check wiring.");
        while (1);
    }
    particleSensor.setup(); // Configure sensor with default settings
    particleSensor.setPulseAmplitudeRed(0x0A); // Turn Red LED to low to indicate sensor is running
    particleSensor.setPulseAmplitudeIR(0x0A); // Turn IR LED to low to indicate sensor is running
    particleSensor.setPulseAmplitudeGreen(0); // Turn off Green LED (if it exists in MAX30105)

    // Inicjalizacja MPU6050
    mpu.initialize();
    if (!mpu.testConnection()) {
        Serial.println("MPU6050 connection failed. Check wiring.");
        while (1);
    }
}

void loop() {
    // Odczyt danych z MAX30102
    long irValue = particleSensor.getIR();
    long redValue = particleSensor.getRed();

    // Wyświetlanie surowych danych z MAX30102
    Serial.print("IR: ");
    Serial.print(irValue);
    Serial.print("\tRed: ");
    Serial.print(redValue);

    // Odczyt danych z MPU6050
    int16_t ax, ay, az;
    int16_t gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    // Wyświetlanie surowych danych z MPU6050
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

    delay(1000);
}
