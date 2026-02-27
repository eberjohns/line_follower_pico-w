import machine
import network
import socket
import time
import _thread
import ure

# ================= HARDWARE CONFIG =================
MOTOR_PINS = [5, 4, 3, 2] # AIN1, AIN2, BIN1, BIN2
SENSOR_PINS = [15, 14, 13, 12, 9, 8, 7, 6]

# ================= WIFI & DISCOVERY =================
SSID = "POCO M2 Pro"
PASSWORD = "123456789"

# ================= TUNING PARAMETERS =================
PWM_FREQ = 25000
LOOP_DELAY = 0.005 # 5ms loop

# Global variables controlled by Web UI
BASE_SPEED = 70.0
KP = 18.0
KD = 45.0
TURBO_BOOST = 25.0  # Added to base speed when error is ~0
WEIGHTS = [0.0, -5.0, -3.0, -0.2, 0.2, 3.0, 5.0, 0.0]

# ================= STATE VARIABLES =================
state_lock = _thread.allocate_lock()
robot_active = False
search_start = 0
search_dir = 1  # 1 for Left, -1 for Right

# ================= HARDWARE INIT =================
led = machine.Pin("LED", machine.Pin.OUT)
sensors = [machine.Pin(p, machine.Pin.IN) for p in SENSOR_PINS]
pwms = [machine.PWM(machine.Pin(p)) for p in MOTOR_PINS]
for p in pwms:
    p.freq(PWM_FREQ)
    p.duty_u16(0)

def set_motors(l, r):
    l = max(-100, min(100, l))
    r = max(-100, min(100, r))
    # Left Motor
    pwms[0].duty_u16(int(abs(l)/100 * 65535) if l >= 0 else 0)
    pwms[1].duty_u16(0 if l >= 0 else int(abs(l)/100 * 65535))
    # Right Motor
    pwms[2].duty_u16(int(abs(r)/100 * 65535) if r >= 0 else 0)
    pwms[3].duty_u16(0 if r >= 0 else int(abs(r)/100 * 65535))

# ================= WEB UI HTML =================
html = """<!DOCTYPE html><html><head><title>Pico Turbo</title>
<meta name="viewport" content="width=device-width, initial-scale=1">
<style>
body{{background:#111;color:#eee;text-align:center;font-family:sans-serif}}
.time{{font-size:3em;color:#0af;font-family:monospace}}
button{{padding:15px;width:40%;margin:5px;border-radius:8px;font-weight:bold;border:none}}
input{{width:80%}}
</style></head><body>
<div class="time">Time: <span id="t">0.00</span>s</div>
<button style="background:#2ecc71" onclick="fetch('/start')">START</button>
<button style="background:#e74c3c" onclick="fetch('/stop')">STOP</button>
<form id="f">
<p>Speed: <span id="sv">{s}</span>% <input type="range" id="s" min="0" max="100" value="{s}"></p>
<p>Kp: <span id="kpv">{kp}</span> <input type="number" id="kp" min="0" max="100" step="0.1" value="{kp}"></p>
<p>Kd: <span id="kdv">{kd}</span> <input type="number" id="kd" min="0" max="200" step="0.1" value="{kd}"></p>
<button type="submit" style="width:85%;background:#444;color:#fff">UPDATE GAINS</button>
</form>
<script>
['s','kp','kd'].forEach(id=>{{
    document.getElementById(id).oninput=()=>document.getElementById(id+'v').innerText=document.getElementById(id).value;
}});
document.getElementById('f').onsubmit=e=>{{
    e.preventDefault();
    fetch(`/?s=${{s.value}}&kp=${{kp.value}}&kd=${{kd.value}}`);
}};
</script></body></html>"""

# ================= WEB SERVER THREAD =================
def web_server():
    global BASE_SPEED, KP, KD, robot_active
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    wlan.connect(SSID, PASSWORD)
    
    while not wlan.isconnected():
        led.toggle()
        time.sleep(0.5)
    
    ip = wlan.ifconfig()[0]
    print("Robot IP:", ip)
    led.on()

    s = socket.socket()
    s.bind(('0.0.0.0', 80))
    s.listen(1)

    while True:
        try:
            cl, _ = s.accept()
            req = cl.recv(1024).decode()
            if 'GET /start' in req:
                with state_lock: robot_active = True
            elif 'GET /stop' in req:
                with state_lock: robot_active = False
            elif 'GET /?' in req:
                m = ure.search(r's=([\d.]+)&kp=([\d.]+)&kd=([\d.]+)', req)
                if m:
                    BASE_SPEED, KP, KD = float(m.group(1)), float(m.group(2)), float(m.group(3))
            
            cl.send("HTTP/1.0 200 OK\r\nContent-Type: text/html\r\n\r\n" + 
                    html.format(s=BASE_SPEED, kp=KP, kd=KD))
            cl.close()
        except: pass

# ================= MAIN DRIVE LOOP =================
def drive():
    global robot_active, search_start, search_dir
    last_err = 0
    
    while True:
        if robot_active:
            v = [s.value() for s in sensors]
            
            # 1. Check for Finish Box (All sensors see black)
            if sum(v) >= 7:
                with state_lock: robot_active = False
                set_motors(0,0)
                continue

            # 2. Line Detection and Error Calculation
            active_indices = [i for i, val in enumerate(v) if val]
            
            if active_indices:
                # LINE FOUND
                search_start = 0 
                err = sum(WEIGHTS[i] for i in active_indices) / len(active_indices)
            else:
                # LINE LOST: 180-Degree Search Sweep
                if search_start == 0:
                    search_start = time.ticks_ms()
                    search_dir = 1 if last_err >= 0 else -1
                
                elapsed = time.ticks_diff(time.ticks_ms(), search_start)
                
                # Search Pattern: Left (250ms) -> Right (750ms total) -> Left (1500ms total)
                if elapsed < 250:
                    err = 5.0 * search_dir   # Turn one way
                elif elapsed < 750:
                    err = -5.0 * search_dir  # Turn 180 degrees the other way
                elif elapsed < 1500:
                    err = 5.0 * search_dir   # Search even wider
                else:
                    err = 0 # Give up and move straight or stop
            
            # 3. PID Calculation
            # We don't divide by LOOP_DELAY here to keep KD tuning more intuitive
            derivative = err - last_err
            corr = (KP * err) + (KD * derivative)
            last_err = err

            # 4. Speed Logic (Turbo on straights)
            current_base = (BASE_SPEED + TURBO_BOOST) if abs(err) < 0.2 else BASE_SPEED
            
            # 5. Apply Motors
            set_motors(current_base + corr, current_base - corr)
        else:
            # Stopped state
            set_motors(0, 0)
            last_err = 0
            search_start = 0
            time.sleep(0.1)
            
        time.sleep(LOOP_DELAY)

# ================= EXECUTION =================
print("Starting System...")
_thread.start_new_thread(web_server, ())
drive()
