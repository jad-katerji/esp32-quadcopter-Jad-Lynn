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
    
    Serial.println("Setup Complete.");
    Serial.println("Enter a motor power percentage (0-100):");
    
}

void loop() {
    handleComm(); // Handle incoming WebSocket commands
    DroneCommands commands = getRemoteCommands();

    // For testing: Print received commands
    Serial.print("Throttle: ");Serial.print(commands.throttle);
    Serial.print(" | Roll: ");Serial.print(commands.roll);
    Serial.print(" | Pitch: ");Serial.println(commands.pitch);

    applyMotorPower(commands.throttle, commands.throttle, commands.throttle, commands.throttle);
    delay(10); 
}