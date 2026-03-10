#include <Arduino.h>
#include "drone_library.h"



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
}