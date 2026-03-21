#include "drone_library.h"
#include "webpages.h"
#define MAG_ADDR 0x0C


//-------------------------------------------------------------PID Controller code----------------------------------------------------------------

PIDAxis::PIDAxis(float p, float i, float d) : kp(p), ki(i), kd(d) {}

float PIDAxis::calculate(float target, float current, float dt) {
    float error = target - current;
    
    // Apply deadband ONLY to the Proportional output to stop jitter
    float pError = (abs(error) < 5.0) ? 0 : error; 

    // Always update Integral and Derivative to keep the math smooth
    integral += error * dt;
    integral = constrain(integral, -iLimit, iLimit);
    
    float derivative = (abs(error) < 5.0) ? 0 :constrain((error - lastError) / dt, -dlimit, dlimit); // Derivative kick prevention (if error is small, consider it as zero to prevent large derivative spikes)
    lastError = error;

    return (kp * pError) + (ki * integral) + (kd * derivative);
}
 
// Create the instances (Global to the library)
PIDAxis pitchPID(10.0, 2.0, 0.0);
PIDAxis rollPID(10.0, 2.0, 0.0);
//PIDAxis yawPID(2.0, 0.0, 0.1);


MotorSpeeds applyFlightControl(float targetP, float targetR, float targetY, int throttle, bool debug) { // hovering function. targetP (Pitch), targetR (Roll), targetY (Yaw) are desired angles in degrees, throttle parsed as 0-255 for finer control
    
    // 1. Get IMU Data (assuming degrees)
    DroneSensors imu = readSensors();
    float dt = 0.004; // 4ms loop time

    // 2. Calculate PID outputs for each axis
    float pitchAdj = pitchPID.calculate(targetP, imu.pitch, dt);
    float rollAdj  = rollPID.calculate(targetR, imu.roll, dt);
    //float yawAdj   = yawPID.calculate(targetY, imu.yaw, dt);

    // 3. Motor Mixing (X-Configuration)
    int mTL = throttle + pitchAdj + rollAdj ;
    int mTR = throttle + pitchAdj - rollAdj ;
    int mBL = throttle - pitchAdj + rollAdj ;
    int mBR = throttle - pitchAdj - rollAdj ;

    // Constrain motor outputs to 0-255 for safety
    MotorSpeeds speeds = {
        map(constrain(mTL, 0, 255), 0, 255, 0, 100),
        map(constrain(mTR, 0, 255), 0, 255, 0, 100),
        map(constrain(mBL, 0, 255), 0, 255, 0, 100),
        map(constrain(mBR, 0, 255), 0, 255, 0, 100)
    };

    // 4. Apply
    applyMotorPower(speeds.tl, speeds.tr, speeds.bl, speeds.br);

    if (debug) {
        delay(500); // Slow down for readability
        Serial.println();
        Serial.print("Pitch:"); Serial.print(imu.pitch); Serial.print(" | Roll:"); Serial.print(imu.roll);
        Serial.print(" | Adj P:"); Serial.print(pitchAdj); Serial.print(" | Adj R:"); Serial.print(rollAdj);
        Serial.print(" | Motors (TL, TR, BL, BR): "); Serial.print(speeds.tl); Serial.print(", ");Serial.print(speeds.tr); Serial.print(", ");Serial.print(speeds.bl); Serial.print(", ");Serial.println(speeds.br);
    } 

    return speeds;
}
//------------------------------------------------------------Communication code-------------------------------------------------------------

WebServer server(80);
WebSocketsServer webSocket = WebSocketsServer(81);
DroneCommands currentCommands = {0, 0, 0};

void onPacket(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
    if(type == WStype_TEXT) {
        // Expecting a string like "T:50,R:0,P:10,H:0"
        sscanf((char*)payload, "T:%d,R:%d,P:%d,H:%d", 
               &currentCommands.throttle, 
               &currentCommands.roll, 
               &currentCommands.pitch,
               &currentCommands.hovering);
    }
}

void initWiFi() {
    WiFi.softAP("ESP32_Drone", "12345678"); // SSID and Password
    // Serve the HTML string when the phone visits 192.168.4.1
    server.on("/", []() {
        server.send(200, "text/html", html_control_page);
    });

     // Route: IMU Dashboard (NEW)
    server.on("/imu", []() {
        server.send(200, "text/html", html_imu_page);
    });
    
    server.begin();
    webSocket.begin();
    webSocket.onEvent(onPacket);
    Serial.println("WiFi AP Started: ESP32_Drone");
    Serial.println("Control page:    http://192.168.4.1/");
    Serial.println("IMU Dashboard:   http://192.168.4.1/imu");
}

void handleComm() {
    server.handleClient();
    webSocket.loop();
}

DroneCommands getRemoteCommands() {
    return currentCommands;
}


// Sends JSON prefixed with "IMU:" so the control page ignores it
void broadcastIMU() {
    DroneSensors s = readSensors();
    char buf[256];
    snprintf(buf, sizeof(buf),
        "IMU:{\"ax\":%.3f,\"ay\":%.3f,\"az\":%.3f,"
              "\"gx\":%.3f,\"gy\":%.3f,\"gz\":%.3f,"
              "\"roll\":%.2f,\"pitch\":%.2f}",
        s.accX,  s.accY,  s.accZ,
        s.gyroX, s.gyroY, s.gyroZ,
        s.roll,  s.pitch
    );
    webSocket.broadcastTXT(buf);
}

//-----------------------------------------------------------Motor code--------------------------------------------------------------------

// PWM Settings
const int freq = 50;
const int resolution = 16; // 16-bit resolution for smoothness
int hovering_throttle = 100; // Base throttle for hovering (tune this experimentally)
void initMotors() {
    // Attach pins to PWM channels
    
    ledcSetup(0, freq, resolution); 
    ledcSetup(1, freq, resolution);
    ledcSetup(2, freq, resolution);
    ledcSetup(3, freq, resolution);

    ledcAttachPin(MOTOR_TL, 0);
    ledcAttachPin(MOTOR_TR, 1);
    ledcAttachPin(MOTOR_BL, 2);
    ledcAttachPin(MOTOR_BR, 3);

    // ESC Arming Sequence: Send "Zero" throttle for 2 seconds
    applyMotorPower(0, 0, 0, 0);
    delay(2000);
}

void applyMotorPower(int tl, int tr, int bl, int br) {
    //input is 0-100% for each motor
    // 50Hz means 20ms total period.
    // 1ms pulse (OFF) = 5% of 20ms = 3277 (in 16-bit)
    // 2ms pulse (FULL) = 10% of 20ms = 6554 (in 16-bit)
    
    auto percentToDuty = [](int percent) {
        int p = constrain(percent, 0, 100);
        return map(p, 0, 100, 3277, 6554);
    };

    ledcWrite(0, percentToDuty(tl));
    ledcWrite(1, percentToDuty(tr));
    ledcWrite(2, percentToDuty(bl));
    ledcWrite(3, percentToDuty(br));
}


MotorSpeeds calculateMixer(int throttle, float pitchAdj, float rollAdj, bool debug) {
    MotorSpeeds motors;

    // X-Config Mixer Math
    motors.tl = throttle + pitchAdj + rollAdj;
    motors.tr = throttle + pitchAdj - rollAdj;
    motors.bl = throttle - pitchAdj + rollAdj;
    motors.br = throttle - pitchAdj - rollAdj;

    // Constrain values between 0 and 100% (or 255 for PWM)
    motors.tl = constrain(motors.tl, 0, 100);
    motors.tr = constrain(motors.tr, 0, 100);
    motors.bl = constrain(motors.bl, 0, 100);
    motors.br = constrain(motors.br, 0, 100);

    if(debug) {
        Serial.println();
        Serial.print("Calculated Motors (TL, TR, BL, BR): ");
        Serial.print(motors.tl); Serial.print(", ");
        Serial.print(motors.tr); Serial.print(", ");
        Serial.print(motors.bl); Serial.print(", ");
        Serial.println(motors.br);
        delay(500); // Slow down for readability
    }
    return motors;
}
    


//-----------------------------------------------------------Sensor code-------------------------------------------------------------------

Adafruit_MPU6050 mpu;

bool initSensors(int sda, int scl) {
    Wire.begin(sda, scl);
    
    if (!mpu.begin()) return false;

    // Set the Digital Low-Pass Filter bandwidth to 21 Hz
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

    delay(100);
    return true; 
}
float previousPitch = 0, previousRoll = 0; // Initialize for complementary filter
float gyroRoll = 0, gyroPitch = 0; // Initialize for gyro integration
unsigned long lastTime = 0;

DroneSensors readSensors() {

    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    DroneSensors data;
    // Store Raw
    data.accX = a.acceleration.x;
    data.accY = a.acceleration.y;
    data.accZ = a.acceleration.z;
    data.gyroX = g.gyro.x;
    data.gyroY = g.gyro.y;
    data.gyroZ = g.gyro.z;

    // Complementary Filter for Angle Estimation (currently testing using gyro only)
    unsigned long currentTime = micros();
    float dt = (currentTime - lastTime) / 1000000.0; // Convert microseconds to seconds
    lastTime = currentTime;

    // getting the angle from the gyro 
    gyroRoll += data.gyroX * dt;
    gyroPitch += data.gyroY * dt;

    data.roll = atan2(-data.accY, -data.accZ) * 180 / PI; // Accelerometer-based roll
    data.pitch = atan2(data.accX, sqrt(data.accY * data.accY + data.accZ * data.accZ)) * 180 / PI; // Accelerometer-based pitch

    return data;
   
}
