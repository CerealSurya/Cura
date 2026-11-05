
#include <Arduino.h>
#include <Wire.h>
#include <SparkFun_VL53L5CX_Library.h>
#include <NimBLEDevice.h>

// BLE identifiers
#define SERVICE_UUID        "12345678-1234-1234-1234-123456789000"
#define CHARACTERISTIC_UUID "12345678-1234-1234-1234-123456789001"

// Sensor objects
SparkFun_VL53L5CX sensor;
VL53L5CX_ResultsData data;

// BLE characteristic
NimBLECharacteristic* pCharacteristic;

uint16_t frameNumber = 0;

void setup() {
    Serial.begin(115200);
    delay(1000);

    // -------------------------
    // Initialize VL53L5CX
    // -------------------------

    Serial.println("Starting VL53L5CX...");

    Wire.begin(21, 22);

    if (sensor.begin() == false) {
        Serial.println("Sensor initialization failed!");
        while (1) {
            delay(1000);
        }
    }

    sensor.setResolution(8 * 8);
    sensor.startRanging();

    Serial.println("VL53L5CX initialized!");

    // -------------------------
    // Initialize BLE
    // -------------------------

    Serial.println("Starting Bluetooth...");

    NimBLEDevice::init("TOF_ESP32");

    NimBLEServer* server = NimBLEDevice::createServer();

    NimBLEService* service = server->createService(
        SERVICE_UUID
    );

    pCharacteristic = service->createCharacteristic(
        CHARACTERISTIC_UUID,
        NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY
    );

    pCharacteristic->setValue("Sensor starting");

    service->start();

    NimBLEAdvertising* advertising = NimBLEDevice::getAdvertising();

    advertising->addServiceUUID(SERVICE_UUID);
    advertising->start();

    Serial.println("BLE advertising as TOF_ESP32!");
    Serial.println("Waiting for sensor measurements...");
}

void loop() {

    // Wait until the sensor has new measurements
    if (sensor.isDataReady()) {

        if (sensor.getRangingData(&data)) {

            Serial.println("Distance measurements:");

            // Print all 64 measurements
            for (int i = 0; i < 64; i++) {
                Serial.print(data.distance_mm[i]);
                Serial.print("\t");

                if ((i + 1) % 8 == 0) {
                    Serial.println();
                }
            }

            // -------------------------
            // Transmit measurements over BLE
            // -------------------------

            // Send 8 measurements per BLE notification.
            // Each packet is 19 bytes, fitting the
            // default BLE notification payload limit.

            for (int chunk = 0; chunk < 8; chunk++) {

                uint8_t packet[19];

                // Frame number (2 bytes)
                packet[0] = frameNumber & 0xFF;
                packet[1] = (frameNumber >> 8) & 0xFF;

                // Chunk number (1 byte)
                packet[2] = chunk;

                // Eight 16-bit distance measurements
                for (int j = 0; j < 8; j++) {

                    uint16_t distance =
                        (uint16_t)data.distance_mm[chunk * 8 + j];

                    packet[3 + j * 2] = distance & 0xFF;
                    packet[4 + j * 2] = (distance >> 8) & 0xFF;
                }

                pCharacteristic->setValue(packet, sizeof(packet));
                pCharacteristic->notify();

                delay(10);
            }

            frameNumber++;
        }
    }

    delay(5);
}
