#include "drone_library.h"

#define MAG_ADDR 0x0C


//-------------------------------------------------------------PID Controller code----------------------------------------------------------------

PIDAxis::PIDAxis(float p, float i, float d) : kp(p), ki(i), kd(d) {}

float PIDAxis::calculate(float target, float current, float dt) {
    float error = target - current;
    
    // Apply deadband ONLY to the Proportional output to stop jitter
    float pError = (abs(error) < 1.0) ? 0 : error; 

    // Always update Integral and Derivative to keep the math smooth
    integral += error * dt;
    integral = constrain(integral, -iLimit, iLimit);
    
    float derivative = (abs(error) < 1.0) ? 0 :constrain((error - lastError) / dt, -dlimit, dlimit); // Derivative kick prevention (if error is small, consider it as zero to prevent large derivative spikes)
    lastError = error;

    return (kp * pError) + (ki * integral) + (kd * derivative);
}
 
// Create the instances (Global to the library)
PIDAxis pitchPID(2.0, 0.0, 0.0);
PIDAxis rollPID(2.0, 0.00, 0.0);
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


// --- IMU Dashboard Page ---
// Served at http://192.168.4.1/imu
// Connects to ws://192.168.4.1:81/ and filters for "IMU:" prefixed messages
const char* html_imu_page = R"=====(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>Drone IMU Dashboard</title>
<style>
  @import url('https://fonts.googleapis.com/css2?family=Share+Tech+Mono&family=Exo+2:wght@300;600&display=swap');
 
  :root {
    --bg: #0a0c0f;
    --panel: #10141a;
    --border: #1e2d3d;
    --accent: #00d4ff;
    --accent2: #ff6b35;
    --accent3: #39ff14;
    --text: #c8d8e8;
    --muted: #4a6070;
    --warn: #ffcc00;
  }
 
  * { margin: 0; padding: 0; box-sizing: border-box; }
 
  body {
    background: var(--bg);
    color: var(--text);
    font-family: 'Exo 2', sans-serif;
    min-height: 100vh;
    overflow-x: hidden;
  }
 
  /* Scanline overlay */
  body::before {
    content: '';
    position: fixed;
    inset: 0;
    background: repeating-linear-gradient(
      0deg,
      transparent,
      transparent 2px,
      rgba(0,212,255,0.015) 2px,
      rgba(0,212,255,0.015) 4px
    );
    pointer-events: none;
    z-index: 1000;
  }
 
  header {
    display: flex;
    align-items: center;
    justify-content: space-between;
    padding: 16px 28px;
    border-bottom: 1px solid var(--border);
    background: var(--panel);
  }
 
  .logo {
    font-family: 'Share Tech Mono', monospace;
    font-size: 1.1rem;
    color: var(--accent);
    letter-spacing: 3px;
    text-transform: uppercase;
  }
 
  .status-dot {
    width: 10px; height: 10px;
    border-radius: 50%;
    background: var(--muted);
    display: inline-block;
    margin-right: 8px;
    transition: background 0.3s;
  }
  .status-dot.connected { background: var(--accent3); box-shadow: 0 0 8px var(--accent3); }
  .status-dot.disconnected { background: #ff3333; }
 
  #status-text {
    font-family: 'Share Tech Mono', monospace;
    font-size: 0.75rem;
    color: var(--muted);
    letter-spacing: 1px;
  }
 
  .timestamp {
    font-family: 'Share Tech Mono', monospace;
    font-size: 0.7rem;
    color: var(--muted);
  }
 
  /* ---- MAIN GRID ---- */
  .dashboard {
    display: grid;
    grid-template-columns: 1fr 1fr 1fr;
    grid-template-rows: auto auto auto;
    gap: 14px;
    padding: 20px 24px;
    max-width: 1200px;
    margin: 0 auto;
  }
 
  /* Panel base */
  .panel {
    background: var(--panel);
    border: 1px solid var(--border);
    border-radius: 4px;
    padding: 18px 20px;
    position: relative;
    overflow: hidden;
  }
 
  .panel::after {
    content: '';
    position: absolute;
    top: 0; left: 0;
    width: 3px; height: 100%;
  }
 
  .panel.accel::after { background: var(--accent); }
  .panel.gyro::after  { background: var(--accent2); }
  .panel.angles::after { background: var(--accent3); }
  .panel.attitude { grid-column: 1 / -1; }
  .panel.attitude::after { background: var(--accent); }
 
  .panel-label {
    font-family: 'Share Tech Mono', monospace;
    font-size: 0.6rem;
    letter-spacing: 3px;
    text-transform: uppercase;
    color: var(--muted);
    margin-bottom: 14px;
  }
 
  /* ---- VALUE ROWS ---- */
  .val-row {
    display: flex;
    align-items: center;
    justify-content: space-between;
    margin-bottom: 12px;
  }
  .val-row:last-child { margin-bottom: 0; }
 
  .val-label {
    font-family: 'Share Tech Mono', monospace;
    font-size: 0.7rem;
    color: var(--muted);
    width: 30px;
  }
 
  .val-bar-wrap {
    flex: 1;
    height: 4px;
    background: rgba(255,255,255,0.05);
    border-radius: 2px;
    margin: 0 12px;
    overflow: hidden;
  }
 
  .val-bar {
    height: 100%;
    border-radius: 2px;
    transition: width 0.12s ease;
    min-width: 2px;
  }
 
  .panel.accel .val-bar { background: var(--accent); box-shadow: 0 0 6px var(--accent); }
  .panel.gyro  .val-bar { background: var(--accent2); box-shadow: 0 0 6px var(--accent2); }
  .panel.angles .val-bar { background: var(--accent3); box-shadow: 0 0 6px var(--accent3); }
 
  .val-num {
    font-family: 'Share Tech Mono', monospace;
    font-size: 0.85rem;
    text-align: right;
    min-width: 70px;
    color: var(--text);
  }
 
  .val-unit {
    font-size: 0.6rem;
    color: var(--muted);
    margin-left: 3px;
  }
 
  /* ---- ATTITUDE VISUALIZER ---- */
  .attitude-inner {
    display: grid;
    grid-template-columns: 1fr 1fr;
    gap: 20px;
    align-items: center;
  }
 
  .horizon-wrap {
    display: flex;
    flex-direction: column;
    align-items: center;
    gap: 8px;
  }
 
  .horizon-label {
    font-family: 'Share Tech Mono', monospace;
    font-size: 0.6rem;
    letter-spacing: 2px;
    color: var(--muted);
  }
 
  /* Artificial Horizon */
  .horizon {
    width: 180px; height: 180px;
    border-radius: 50%;
    border: 2px solid var(--border);
    overflow: hidden;
    position: relative;
    box-shadow: 0 0 20px rgba(0,212,255,0.1);
  }
 
  .horizon-inner {
    position: absolute;
    width: 200%;
    height: 200%;
    left: -50%;
    top: -50%;
    transition: transform 0.12s ease;
  }
 
  .sky {
    position: absolute;
    top: 0; left: 0; right: 0;
    height: 50%;
    background: linear-gradient(180deg, #0a1628 0%, #112244 100%);
  }
 
  .ground {
    position: absolute;
    bottom: 0; left: 0; right: 0;
    height: 50%;
    background: linear-gradient(180deg, #3d1a00 0%, #1a0a00 100%);
  }
 
  .horizon-line {
    position: absolute;
    top: 50%; left: 0; right: 0;
    height: 1px;
    background: var(--accent);
    box-shadow: 0 0 4px var(--accent);
  }
 
  /* Overlay crosshair (static) */
  .horizon-overlay {
    position: absolute;
    inset: 0;
    pointer-events: none;
  }
 
  .horizon-overlay::before, .horizon-overlay::after {
    content: '';
    position: absolute;
    background: rgba(0,212,255,0.6);
  }
 
  .horizon-overlay::before {
    top: 50%; left: 20%; right: 20%;
    height: 1px;
    transform: translateY(-50%);
  }
 
  .horizon-overlay::after {
    left: 50%; top: 30%; bottom: 30%;
    width: 1px;
    transform: translateX(-50%);
  }
 
  /* Angle displays */
  .angle-readouts {
    display: flex;
    flex-direction: column;
    gap: 20px;
    justify-content: center;
  }
 
  .angle-card {
    background: rgba(0,0,0,0.3);
    border: 1px solid var(--border);
    border-radius: 4px;
    padding: 14px 18px;
    text-align: center;
  }
 
  .angle-title {
    font-family: 'Share Tech Mono', monospace;
    font-size: 0.55rem;
    letter-spacing: 3px;
    color: var(--muted);
    text-transform: uppercase;
    margin-bottom: 6px;
  }
 
  .angle-value {
    font-family: 'Share Tech Mono', monospace;
    font-size: 2rem;
    font-weight: bold;
    line-height: 1;
  }
 
  #roll-display  { color: var(--accent); }
  #pitch-display { color: var(--accent2); }
 
  .angle-deg {
    font-size: 0.8rem;
    color: var(--muted);
    margin-left: 2px;
  }
 
  /* Bottom stats row */
  .stats-row {
    grid-column: 1 / -1;
    display: grid;
    grid-template-columns: repeat(4, 1fr);
    gap: 14px;
  }
 
  .stat-card {
    background: var(--panel);
    border: 1px solid var(--border);
    border-radius: 4px;
    padding: 14px 18px;
    text-align: center;
  }
 
  .stat-label {
    font-family: 'Share Tech Mono', monospace;
    font-size: 0.55rem;
    letter-spacing: 2px;
    color: var(--muted);
    text-transform: uppercase;
    margin-bottom: 6px;
  }
 
  .stat-value {
    font-family: 'Share Tech Mono', monospace;
    font-size: 1.3rem;
    color: var(--warn);
  }
 
  /* Hz counter */
  #hz-value { color: var(--accent3); }
 
  /* Responsive */
  @media (max-width: 700px) {
    .dashboard { grid-template-columns: 1fr; }
    .stats-row { grid-template-columns: 1fr 1fr; }
    .attitude-inner { grid-template-columns: 1fr; }
    .panel.attitude { grid-column: 1; }
  }
 
  /* Animate in */
  @keyframes fadeUp {
    from { opacity: 0; transform: translateY(16px); }
    to   { opacity: 1; transform: translateY(0); }
  }
 
  .panel { animation: fadeUp 0.5s ease both; }
  .panel:nth-child(1) { animation-delay: 0.05s; }
  .panel:nth-child(2) { animation-delay: 0.10s; }
  .panel:nth-child(3) { animation-delay: 0.15s; }
  .panel:nth-child(4) { animation-delay: 0.20s; }
  .panel:nth-child(5) { animation-delay: 0.25s; }
</style>
</head>
<body>
<header>
  <div class="logo">ESP32 // IMU TELEMETRY</div>
  <div>
    <span class="status-dot" id="status-dot"></span>
    <span id="status-text">CONNECTING...</span>
  </div>
  <div class="timestamp" id="timestamp">--:--:--</div>
</header>
 
<div class="dashboard">
 
  <!-- Accelerometer Panel -->
  <div class="panel accel">
    <div class="panel-label">// ACCELEROMETER</div>
    <div class="val-row">
      <span class="val-label">AX</span>
      <div class="val-bar-wrap"><div class="val-bar" id="bar-ax" style="width:50%"></div></div>
      <span class="val-num"><span id="ax">0.000</span><span class="val-unit">m/s2</span></span>
    </div>
    <div class="val-row">
      <span class="val-label">AY</span>
      <div class="val-bar-wrap"><div class="val-bar" id="bar-ay" style="width:50%"></div></div>
      <span class="val-num"><span id="ay">0.000</span><span class="val-unit">m/s2</span></span>
    </div>
    <div class="val-row">
      <span class="val-label">AZ</span>
      <div class="val-bar-wrap"><div class="val-bar" id="bar-az" style="width:50%"></div></div>
      <span class="val-num"><span id="az">0.000</span><span class="val-unit">m/s2</span></span>
    </div>
  </div>
 
  <!-- Gyroscope Panel -->
  <div class="panel gyro">
    <div class="panel-label">// GYROSCOPE</div>
    <div class="val-row">
      <span class="val-label">GX</span>
      <div class="val-bar-wrap"><div class="val-bar" id="bar-gx" style="width:50%"></div></div>
      <span class="val-num"><span id="gx">0.000</span><span class="val-unit">r/s</span></span>
    </div>
    <div class="val-row">
      <span class="val-label">GY</span>
      <div class="val-bar-wrap"><div class="val-bar" id="bar-gy" style="width:50%"></div></div>
      <span class="val-num"><span id="gy">0.000</span><span class="val-unit">r/s</span></span>
    </div>
    <div class="val-row">
      <span class="val-label">GZ</span>
      <div class="val-bar-wrap"><div class="val-bar" id="bar-gz" style="width:50%"></div></div>
      <span class="val-num"><span id="gz">0.000</span><span class="val-unit">r/s</span></span>
    </div>
  </div>
 
  <!-- Angles Panel -->
  <div class="panel angles">
    <div class="panel-label">// COMPUTED ANGLES</div>
    <div class="val-row">
      <span class="val-label">ROL</span>
      <div class="val-bar-wrap"><div class="val-bar" id="bar-roll" style="width:50%"></div></div>
      <span class="val-num"><span id="roll-num">0.0</span><span class="val-unit">deg</span></span>
    </div>
    <div class="val-row">
      <span class="val-label">PIT</span>
      <div class="val-bar-wrap"><div class="val-bar" id="bar-pitch" style="width:50%"></div></div>
      <span class="val-num"><span id="pitch-num">0.0</span><span class="val-unit">deg</span></span>
    </div>
  </div>
 
  <!-- Attitude Visualizer -->
  <div class="panel attitude">
    <div class="panel-label">// ATTITUDE</div>
    <div class="attitude-inner">
      <div class="horizon-wrap">
        <div class="horizon">
          <div class="horizon-inner" id="horizon-inner">
            <div class="sky"></div>
            <div class="ground"></div>
            <div class="horizon-line"></div>
          </div>
          <div class="horizon-overlay"></div>
        </div>
        <div class="horizon-label">ARTIFICIAL HORIZON</div>
      </div>
      <div class="angle-readouts">
        <div class="angle-card">
          <div class="angle-title">Roll Angle</div>
          <div class="angle-value" id="roll-display">0.0<span class="angle-deg">deg</span></div>
        </div>
        <div class="angle-card">
          <div class="angle-title">Pitch Angle</div>
          <div class="angle-value" id="pitch-display">0.0<span class="angle-deg">deg</span></div>
        </div>
      </div>
    </div>
  </div>
 
  <!-- Bottom Stats Row -->
  <div class="stats-row">
    <div class="stat-card">
      <div class="stat-label">Update Rate</div>
      <div class="stat-value"><span id="hz-value">--</span><span style="font-size:0.7rem;color:var(--muted)"> Hz</span></div>
    </div>
    <div class="stat-card">
      <div class="stat-label">Packets Recv</div>
      <div class="stat-value" id="pkt-count">0</div>
    </div>
    <div class="stat-card">
      <div class="stat-label">Accel Magnitude</div>
      <div class="stat-value"><span id="accel-mag">0.00</span><span style="font-size:0.7rem;color:var(--muted)"> m/s2</span></div>
    </div>
    <div class="stat-card">
      <div class="stat-label">Gyro Magnitude</div>
      <div class="stat-value"><span id="gyro-mag">0.00</span><span style="font-size:0.7rem;color:var(--muted)"> r/s</span></div>
    </div>
  </div>
 
</div>
 
<script>
  var ws = new WebSocket('ws://' + window.location.hostname + ':81/');
  var dot = document.getElementById('status-dot');
  var statusText = document.getElementById('status-text');
  var packetCount = 0;
  var lastHz = 0, hzTimer = 0, hzCount = 0;
 
  ws.onopen = function() {
    dot.className = 'status-dot connected';
    statusText.textContent = 'LINK ESTABLISHED';
  };
 
  ws.onclose = function() {
    dot.className = 'status-dot disconnected';
    statusText.textContent = 'LINK LOST -- RECONNECTING';
    setTimeout(function() { location.reload(); }, 2000);
  };
 
  // Only process messages that start with "IMU:" prefix
  ws.onmessage = function(e) {
    if (typeof e.data !== 'string' || e.data.indexOf('IMU:') !== 0) return;
    var jsonStr = e.data.substring(4);
    var d;
    try { d = JSON.parse(jsonStr); } catch(err) { return; }
 
    packetCount++;
    hzCount++;
    document.getElementById('pkt-count').textContent = packetCount;
    document.getElementById('timestamp').textContent = new Date().toLocaleTimeString();
 
    // Update accelerometer
    document.getElementById('ax').textContent = d.ax.toFixed(3);
    document.getElementById('ay').textContent = d.ay.toFixed(3);
    document.getElementById('az').textContent = d.az.toFixed(3);
 
    // Update gyro
    document.getElementById('gx').textContent = d.gx.toFixed(3);
    document.getElementById('gy').textContent = d.gy.toFixed(3);
    document.getElementById('gz').textContent = d.gz.toFixed(3);
 
    // Update angles
    document.getElementById('roll-num').textContent   = d.roll.toFixed(1);
    document.getElementById('pitch-num').textContent  = d.pitch.toFixed(1);
    document.getElementById('roll-display').innerHTML  = d.roll.toFixed(1) + '<span class="angle-deg">deg</span>';
    document.getElementById('pitch-display').innerHTML = d.pitch.toFixed(1) + '<span class="angle-deg">deg</span>';
 
    // Progress bars: map accel -20..20 to 0..100%, gyro -10..10 to 0..100%
    function accelPct(v) { return Math.min(100, Math.max(0, ((v + 20) / 40) * 100)); }
    function gyroPct(v)  { return Math.min(100, Math.max(0, ((v + 10) / 20) * 100)); }
    function anglePct(v) { return Math.min(100, Math.max(0, ((v + 180) / 360) * 100)); }
 
    document.getElementById('bar-ax').style.width    = accelPct(d.ax) + '%';
    document.getElementById('bar-ay').style.width    = accelPct(d.ay) + '%';
    document.getElementById('bar-az').style.width    = accelPct(d.az) + '%';
    document.getElementById('bar-gx').style.width    = gyroPct(d.gx) + '%';
    document.getElementById('bar-gy').style.width    = gyroPct(d.gy) + '%';
    document.getElementById('bar-gz').style.width    = gyroPct(d.gz) + '%';
    document.getElementById('bar-roll').style.width  = anglePct(d.roll)  + '%';
    document.getElementById('bar-pitch').style.width = anglePct(d.pitch) + '%';
 
    // Magnitudes
    var am = Math.sqrt(d.ax*d.ax + d.ay*d.ay + d.az*d.az);
    var gm = Math.sqrt(d.gx*d.gx + d.gy*d.gy + d.gz*d.gz);
    document.getElementById('accel-mag').textContent = am.toFixed(2);
    document.getElementById('gyro-mag').textContent  = gm.toFixed(2);
 
    // Artificial horizon: roll rotates the inner div, pitch shifts it vertically
    // roll: direct CSS rotation; pitch: translateY scaled to viewport
    var pitchShift = d.pitch * 0.8; // px per degree
    document.getElementById('horizon-inner').style.transform =
      'rotate(' + (-d.roll) + 'deg) translateY(' + pitchShift + 'px)';
  };
 
  // Compute Hz every second
  setInterval(function() {
    document.getElementById('hz-value').textContent = hzCount;
    hzCount = 0;
  }, 1000);
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

    
    data.roll = atan2(data.accY, data.accZ) * 180 / PI + 180; // Add 180 to make roll back to 0 since the imu sensor will be flipped upside down on the drone. 
    data.pitch = atan2(-data.accX, sqrt(data.accY * data.accY + data.accZ * data.accZ)) * 180 / PI;
    
  

    return data;
   
}
