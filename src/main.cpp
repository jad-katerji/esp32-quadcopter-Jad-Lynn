#include <Arduino.h>
#include "drone_library.h"



void setup() {
    
    Serial.begin(115200);
    delay(1000); // Give the serial monitor time to connect
    initWiFi();
    initMotors();
    initSensors(4, 5);
    Serial.println("Setup Complete.");
<<<<<<< HEAD
    

=======
   
>>>>>>> PID-
    
}

void loop() {
    handleComm(); // Handle incoming WebSocket commands
    DroneCommands commands = getRemoteCommands();
<<<<<<< HEAD
=======

    DroneSensors imu = readSensors();

    MotorSpeeds current_speeds = applyFlightControl(0.0,0.0,0.0, 50);

>>>>>>> PID-
    
}