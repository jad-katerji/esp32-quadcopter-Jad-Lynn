#include "drone_library.h"

#define MAG_ADDR 0x0C


<<<<<<< HEAD

=======
//-------------------------------------------------------------PID Controller code----------------------------------------------------------------

PIDAxis::PIDAxis(float p, float i, float d) : kp(p), ki(i), kd(d) {}

float PIDAxis::calculate(float target, float current, float dt) {
    float error = target - current;
    
    // Apply deadband ONLY to the Proportional output to stop jitter
    float pError = (abs(error) < 1.0) ? 0 : error; 

    // Always update Integral and Derivative to keep the math smooth
    integral += error * dt;
    integral = constrain(integral, -iLimit, iLimit);
    
    float derivative = (error - lastError) / dt;
    lastError = error;

    return (kp * pError) + (ki * integral) + (kd * derivative);
}
 
// Create the instances (Global to the library)
PIDAxis pitchPID(2.0, 0.0, 0.0);
PIDAxis rollPID(2.0, 0.00, 0.0);
//PIDAxis yawPID(2.0, 0.0, 0.1);


MotorSpeeds applyFlightControl(float targetP, float targetR, float targetY, int throttle, bool debug) {
    // throttle parsed as 0-255
    // targetP (Pitch), targetR (Roll), targetY (Yaw) are desired angles in degrees

    // 1. Get IMU Data (assuming degrees)
    DroneSensors imu = readSensors();
    float dt = 0.004; // 4ms loop time

    // 2. Calculate PID outputs for each axis
    float pitchAdj = pitchPID.calculate(targetP, imu.smoothedpitch, dt);
    float rollAdj  = rollPID.calculate(targetR, imu.smoothedroll, dt);
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
        Serial.print("Pitch:"); Serial.print(imu.smoothedpitch); Serial.print(" | Roll:"); Serial.print(imu.smoothedroll);
        Serial.print(" | Adj P:"); Serial.print(pitchAdj); Serial.print(" | Adj R:"); Serial.print(rollAdj);
        Serial.print(" | Motors (TL, TR, BL, BR): "); Serial.print(speeds.tl); Serial.print(", ");Serial.print(speeds.tr); Serial.print(", ");Serial.print(speeds.bl); Serial.print(", ");Serial.println(speeds.br);
    } 

    return speeds;
}
>>>>>>> PID-
//------------------------------------------------------------Communication code-------------------------------------------------------------
const char* html_control_page = R"=====(
<!DOCTYPE html>
<html>
<head>
    <meta name="viewport" content="width=device-width, initial-scale=1.0, maximum-scale=1.0, user-scalable=0">
    <style>
        body { text-align: center; font-family: sans-serif; background: #1a1a1a; color: white; overflow: hidden; }
        .slider { width: 80%; height: 50px; margin: 20px; }
        #joystick { width: 200px; height: 200px; background: #333; border-radius: 50%; margin: auto; position: relative; }
        #knob { width: 50px; height: 50px; background: #ff4444; border-radius: 50%; position: absolute; top: 75px; left: 75px; }
    </style>
</head>
<body>
    <h2>ESP32 DRONE CONTROL </h2>
    <p>Throttle: <span id="tVal">0</span>%</p>
    <input type="range" id="throttle" class="slider" min="0" max="100" value="0">
    <div id="joystick"><div id="knob"></div></div>
    <script>
        var gateway = `ws://${window.location.hostname}:81/`;
        var websocket = new WebSocket(gateway);
        var throttle = document.getElementById('throttle');
        
        // Send data every 50ms
        setInterval(() => {
            if (websocket.readyState == 1) {
                let msg = "T:" + throttle.value + ",R:0,P:0"; // Simplified for now
                websocket.send(msg);
                document.getElementById('tVal').innerText = throttle.value;
            }
        }, 50);
    </script>
</body>
</html>
)=====";

WebServer server(80);
WebSocketsServer webSocket = WebSocketsServer(81);
DroneCommands currentCommands = {0, 0, 0};

void onPacket(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
    if(type == WStype_TEXT) {
        // Expecting a string like "T:50,R:0,P:10"
        sscanf((char*)payload, "T:%d,R:%d,P:%d", 
               &currentCommands.throttle, 
               &currentCommands.roll, 
               &currentCommands.pitch);
    }
}

void initWiFi() {
    WiFi.softAP("ESP32_Drone", "12345678"); // SSID and Password
    // Serve the HTML string when the phone visits 192.168.4.1
    server.on("/", []() {
        server.send(200, "text/html", html_control_page);
    });
    
    server.begin();
    webSocket.begin();
    webSocket.onEvent(onPacket);
    Serial.println("WiFi AP Started: ESP32_Drone");
}

void handleComm() {
    server.handleClient();
    webSocket.loop();
}

DroneCommands getRemoteCommands() {
    return currentCommands;
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


    


//-----------------------------------------------------------Sensor code-------------------------------------------------------------------

Adafruit_MPU6050 mpu;

bool initSensors(int sda, int scl) {
    Wire.begin(sda, scl);
    
    if (!mpu.begin()) return false;

    delay(100);
    return true; 
}

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

    static float lastroll = 0;
    static float lastpitch = 0;
    // Calculate Degrees (The "Angles and Stuff")
    // Roll: rotation around X-axis
    data.smoothedroll = 0.7 * lastroll + 0.3 * (atan2(data.accY, data.accZ) * 180 / PI + 180); // Add 180 to make roll back to 0 since the imu sensor will be flipped upside down on the drone. 
    data.smoothedpitch = 0.7 * lastpitch + 0.3 * (atan2(-data.accX, sqrt(data.accY * data.accY + data.accZ * data.accZ)) * 180 / PI);
    
<<<<<<< HEAD
    
=======
    // Store last values for smoothing
    lastpitch = data.smoothedpitch;
    lastroll = data.smoothedroll;

>>>>>>> PID-
    return data;
   
}
