#include <Arduino.h>
#include "drone_library.h"

// Timing for IMU broadcast — avoids blocking the main loop
unsigned long lastIMUBroadcast = 0;
const unsigned long IMU_BROADCAST_MS = 100; // 10 Hz — safe for WebSocket + flight loop
 

void setup() {
    
    Serial.begin(115200);
    delay(1000); // Give the serial monitor time to connect
    initWiFi();
    initMotors();
    initSensors(5, 4); // SDA, SCL pins for I2C
    Serial.println("Setup Complete.");
    

    
}

void loop() {
    handleComm(); // Handle incoming WebSocket commands
    DroneCommands commands = getRemoteCommands();
    
    applyFlightControl(0, 0, 0, 50, true); 

    // Broadcast IMU data at regular intervals
    if (millis() - lastIMUBroadcast > IMU_BROADCAST_MS) {
        broadcastIMU();
        lastIMUBroadcast = millis();
    }
}