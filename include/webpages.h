// This folder will contain the webpages that will be served by the web server. 

#ifndef WEBPAGES_H
#define WEBPAGES_H


// Control page served at http://192.168.4.1

const char* html_control_page = R"=====(
<!DOCTYPE html>
<html>
<head>
    <meta name="viewport" content="width=device-width, initial-scale=1.0, maximum-scale=1.0, user-scalable=0">
    <style>
        body { text-align: center; font-family: sans-serif; background: #1a1a1a; color: white; overflow: hidden; touch-action: none; }
        .slider { width: 80%; height: 50px; margin: 20px; }
        #joystick { width: 200px; height: 200px; background: #333; border-radius: 50%; margin: 50px auto; position: relative; border: 2px solid #555; }
        #knob { width: 60px; height: 60px; background: #ff4444; border-radius: 50%; position: absolute; top: 70px; left: 70px; cursor: pointer; box-shadow: 0 4px 10px rgba(0,0,0,0.5); }
    </style>
</head>
<body>
    <h2>DRONE COMMANDER</h2>
    <p>Throttle: <span id="tVal">0</span>% | Pitch: <span id="pVal">0</span> | Roll: <span id="rVal">0</span></p>
    <input type="range" id="throttle" class="slider" min="0" max="100" value="0">
    
    <div id="joystick"><div id="knob"></div></div>

    <script>
        var gateway = `ws://${window.location.hostname}:81/`;
        var websocket = new WebSocket(gateway);
        
        const throttleInput = document.getElementById('throttle');
        const joystick = document.getElementById('joystick');
        const knob = document.getElementById('knob');
        
        let pitch = 0, roll = 0;
        let dragging = false;

        // Joystick Logic
        const handleInput = (e) => {
            if (!dragging) return;
            const rect = joystick.getBoundingClientRect();
            const centerX = rect.width / 2;
            const centerY = rect.height / 2;
            
            // Get touch or mouse position relative to joystick center
            let clientX = e.touches ? e.touches[0].clientX : e.clientX;
            let clientY = e.touches ? e.touches[0].clientY : e.clientY;
            
            let x = clientX - rect.left - centerX;
            let y = clientY - rect.top - centerY;

            // Limit knob movement to the circle radius (70px)
            const distance = Math.sqrt(x*x + y*y);
            const maxRadius = 70;
            if (distance > maxRadius) {
                x *= maxRadius / distance;
                y *= maxRadius / distance;
            }

            // Move the visual knob
            knob.style.transform = `translate(${x}px, ${y}px)`;

            // Map coordinates to degrees (-30 to 30)
            // Note: y is inverted because in screen coords, up is negative
            roll = Math.round((x / maxRadius) * 30);
            pitch = Math.round(-(y / maxRadius) * 30);
            
            document.getElementById('rVal').innerText = roll;
            document.getElementById('pVal').innerText = pitch;
        };

        const stopDragging = () => {
            dragging = false;
            pitch = 0; roll = 0; // Auto-center
            knob.style.transform = `translate(0px, 0px)`;
            document.getElementById('rVal').innerText = 0;
            document.getElementById('pVal').innerText = 0;
        };

        joystick.addEventListener('mousedown', () => dragging = true);
        joystick.addEventListener('touchstart', () => dragging = true);
        window.addEventListener('mousemove', handleInput);
        window.addEventListener('touchmove', handleInput);
        window.addEventListener('mouseup', stopDragging);
        window.addEventListener('touchend', stopDragging);

        // Send data string: "T:val,R:val,P:val"
        setInterval(() => {
            if (websocket.readyState == 1) {
                let msg = `T:${throttleInput.value},R:${roll},P:${pitch}`;
                websocket.send(msg);
                document.getElementById('tVal').innerText = throttleInput.value;
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

#endif 