#ifndef DRONE_LIBRARY_H
#define DRONE_LIBRARY_H

#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Wire.h>
#include <WiFi.h>
#include <WebSocketsServer.h>
#include <WebServer.h>

//-------------------------------------------Sensors------------------------------------------

struct DroneSensors {
    float accX, accY, accZ;
    float gyroX, gyroY, gyroZ;
    float magX, magY, magZ; 
};
// Sensor Functions
bool initSensors(int sda, int scl); // Initialize I2C and MPU6050
DroneSensors readSensors(); // Read accelerometer and gyroscope data

//-------------------------------------------Control-------------------------------------------

struct MotorPowers {
    int tl, tr, bl, br; // Top Left, Top Right, Bottom Left, Bottom Right
};

// PID Drill Function
// Parameters: current sensor data, target pitch, target roll, and base throttle

MotorPowers CalcPID(DroneSensors s, float targetPitch, float targetRoll, int baseThrottle);

//------------------------------------------Communication------------------------------------------

// Struct for the control commands from your phone
struct DroneCommands {
    int throttle; // 0-100
    int roll;     // -50 to 50
    int pitch;    // -50 to 50
};


void initWiFi();
void handleComm(); 
DroneCommands getRemoteCommands();

//------------------------------------------Motors------------------------------------------

extern int hovering_throttle; // Declare hovering_throttle as an external variable
// Define Motor Pins (ESP32 GPIOs)
#define MOTOR_TL 23 // Top Left
#define MOTOR_TR 22 // Top Right
#define MOTOR_BL 21 // Bottom Left
#define MOTOR_BR 19 // Bottom Right

void initMotors(); // Initialize PWM channels and arm ESCs
void applyMotorPower(int tl, int tr, int bl, int br); // Set individual motor speeds (0-100%)




#endif