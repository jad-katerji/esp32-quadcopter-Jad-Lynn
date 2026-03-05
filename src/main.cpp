#include <Arduino.h>
#include "drone_library.h"


// PID Control Variables
float kp = 50.0, ki = 0.0, kd = 0.0;

int baseThrottle = 127; // LEDs at half-brightness when level

void setup() {
    
    Serial.begin(115200);
    delay(1000); // Give the serial monitor time to connect
    initWiFi();
    initMotors();
    initSensors(4, 5);
    Serial.println("Setup Complete.");
   
    
}

void loop() {
    handleComm(); // Handle incoming WebSocket commands
    DroneCommands commands = getRemoteCommands();

    DroneSensors imu = readSensors();

    MotorSpeeds current_speeds = applyFlightControl(0.0,0.0,0.0, 50);

    
}