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
    // Handle incoming WebSocket commands
    handleComm(); 
    DroneCommands commands = getRemoteCommands();
    
    // If hover mode is active, maintain a base throttle with no adjustments. Otherwise, apply direct throttle for testing.
    if (commands.hovering == 1){
        applyFlightControl(0, 0, 0, commands.throttle, commands.kp, commands.ki, commands.kd); // Maintain hover throttle with no adjustments
    } else {
        MotorSpeeds speeds = calculateMixer(commands.throttle, commands.pitch, commands.roll, false);
        applyMotorPower(speeds.tl, speeds.tr, speeds.bl, speeds.br); // For simplicity, apply throttle directly to all motors. Replace with PID adjustments for better control.
    }
    

    // Broadcast IMU data at regular intervals
    if (millis() - lastIMUBroadcast > IMU_BROADCAST_MS) {
        broadcastIMU();
        lastIMUBroadcast = millis();
    }
}