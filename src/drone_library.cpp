#include "drone_library.h"

#define MAG_ADDR 0x0C


//-----------------------------PID Control code--------------------------------

// PID Gains 
float Kp = 18.0; 
float Kd = 2.0; 

MotorPowers CalcPID(DroneSensors s, float targetPitch, float targetRoll, int baseThrottle) {
    MotorPowers m;

    // 1. Calculate Error (Target - Current)
    // We use Accelerometer for position (angle)
    float pitchError = targetPitch - s.accY; 
    float rollError  = targetRoll - s.accX;

    // 2. Calculate Correction (P + D)
    // s.gyroX/Y gives us the "speed" of rotation to dampen the movement
    float pitchCorrection = (pitchError * Kp) - (s.gyroY * Kd);
    float rollCorrection  = (rollError * Kp) - (s.gyroX * Kd);

    // 3. Motor Mixing (X-Config)
    m.tl = baseThrottle + pitchCorrection - rollCorrection;
    m.tr = baseThrottle + pitchCorrection + rollCorrection;
    m.bl = baseThrottle - pitchCorrection - rollCorrection;
    m.br = baseThrottle - pitchCorrection + rollCorrection;

    // 4. Safety Constrain and mapping onto 0-100% for ESCs
    m.tl = map(constrain(m.tl, 0, 255), 0, 255, 0, 100);
    m.tr = map(constrain(m.tr, 0, 255), 0, 255, 0, 100);
    m.bl = map(constrain(m.bl, 0, 255), 0, 255, 0, 100);
    m.br = map(constrain(m.br, 0, 255), 0, 255, 0, 100);

    return m;
}
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
    <h2>ESP32 DRONE PROTOTYPE</h2>
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
    data.accX = a.acceleration.x;
    data.accY = a.acceleration.y;
    data.accZ = a.acceleration.z;
    data.gyroX = g.gyro.x;
    data.gyroY = g.gyro.y;
    data.gyroZ = g.gyro.z;
    
    return data;
}