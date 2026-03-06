#include <Arduino.h>
#include "drone_library.h"



void setup() {
    
    Serial.begin(115200);
    delay(1000); // Give the serial monitor time to connect
    initWiFi();
    initMotors();
    
    Serial.println("Setup Complete.");
    

    
}

void loop() {
    handleComm(); // Handle incoming WebSocket commands
    DroneCommands commands = getRemoteCommands();
    
    applyMotorPower(commands.throttle, commands.throttle, commands.throttle, commands.throttle); // For testing, apply the same throttle to all motors. Replace with PID control later.
}