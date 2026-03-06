#ifndef DRONE_LIBRARY_H
#define DRONE_LIBRARY_H

#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Wire.h>
#include <WiFi.h>
#include <WebSocketsServer.h>
#include <WebServer.h>

//------------------------------------------Motors------------------------------------------

// Define Motor Pins (ESP32 GPIOs)
#define MOTOR_TL 23 // Top Left
#define MOTOR_TR 22 // Top Right
#define MOTOR_BL 21 // Bottom Left
#define MOTOR_BR 19 // Bottom Right

struct MotorSpeeds { // for storing the current motor speeds for telemetry
    int tl, tr, bl, br;
};  

void initMotors(); // Initialize PWM channels and arm ESCs
void applyMotorPower(int tl, int tr, int bl, int br); // Set individual motor speeds (0-100%)

//------------------------------------------PID Controller------------------------------------------

struct PIDAxis {
    float kp, ki, kd;
    float integral = 0, lastError = 0;
    float iLimit = 50.0;

    PIDAxis(float p, float i, float d); // Constructor declaration
    float calculate(float target, float current, float dt); // Function declaration
};

MotorSpeeds applyFlightControl(float targetP, float targetR, float targetY, int throttle, bool debug= false); // Calculate PID outputs and apply to motors. throttle is parsed as 0-255 for finer control, but will be constrained in the function to ensure safety. 

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

//------------------------------------------Sensors------------------------------------------

extern int hovering_throttle; // Declare hovering_throttle as an external variable
// Define Motor Pins (ESP32 GPIOs)
#define MOTOR_TL 23 // Top Left
#define MOTOR_TR 22 // Top Right
#define MOTOR_BL 21 // Bottom Left
#define MOTOR_BR 19 // Bottom Right

void initMotors(); // Initialize PWM channels and arm ESCs
void applyMotorPower(int tl, int tr, int bl, int br); // Set individual motor speeds (0-100%)






struct DroneSensors {
    float accX, accY, accZ;
    float gyroX, gyroY, gyroZ;
    float magX, magY, magZ; 
    
    float smoothedroll;
    float smoothedpitch;
};

// Sensor Functions
bool initSensors(int sda, int scl); // Initialize I2C and MPU6050
DroneSensors readSensors(); // Read accelerometer and gyroscope data


#endif