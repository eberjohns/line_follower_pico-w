import machine
import network
import socket
import time
import _thread
import ure

# ================= HARDWARE CONFIG =================
# Driver 1 (Left Motor): PWM=5, IN1=4, IN2=3
# Driver 2 (Right Motor): PWM=2, IN1=1, IN2=0
L_PINS = [5, 4, 3] 
R_PINS = [2, 1, 0]
SENSOR_PINS = [15, 14, 13, 12, 9, 8, 7, 6]

# ================= WIFI & DISCOVERY =================
SSID = "Sam"
PASSWORD = "samuel2006"

# ================= TUNING PARAMETERS =================
PWM_FREQ = 10000    # 10kHz helps TB6612FNG stay cool with 12V motors
LOOP_DELAY = 0.001  # Ultra-fast 1ms loop for 1000 RPM response

# Global variables (Adjustable via Web UI)
BASE_SPEED = 75.0   # Recommended start for 1000 RPM
KP = 30.0           # Proportional: Sharpness of correction
KD = 220.0          # Derivative: Dampening to prevent overshooting
TURBO_BOOST = 15.0  # Speed added on perfect straights

# High-resolution weights for straight-line sensor array
WEIGHTS = [-25.0, -15.0, -5.0, -1.0, 1.0, 5.0, 15.0, 25.0]

# ================= STATE VARIABLES =================
state_lock = _thread.allocate_lock()
robot_active = False
search_start = 0
search_dir = -1  
GAP_TIMEOUT_MS = 0 # Memory duration when line is lost in a turn

# ================= HARDWARE INIT =================
led = machine.Pin("LED", machine.Pin.OUT)
sensors = [machine.Pin(p, machine.Pin.IN) for p in SENSOR_PINS]

# Motor Initializations (Parallel Mode Logic)
l_pwm = machine.PWM(machine.Pin(L_PINS[0]))
l_in1 = machine.Pin(L_PINS[1], machine.Pin.OUT)
l_in2 = machine.Pin(L_PINS[2], machine.Pin.OUT)

r_pwm = machine.PWM(machine.Pin(R_PINS[0]))
r_in1 = machine.Pin(R_PINS[1], machine.Pin.OUT)
r_in2 = machine.Pin(R_PINS[2], machine.Pin.OUT)

l_pwm.freq(PWM_FREQ)
r_pwm.freq(PWM_FREQ)

def set_motors(l, r):
    # Safety cap: Protect drivers and battery
    l = max(-90, min(90, l))
    r = max(-90, min(90, r))
    
    # Left Motor
    l_in1.value(1 if l >= 0 else 0)
    l_in2.value(0 if l >= 0 else 1)
    l_pwm.duty_u16(int(abs(l)/100 * 65535))

    # Right Motor
    r_in1.value(1 if r >= 0 else 0)
    r_in2.value(0 if r >= 0 else 1)
    r_pwm.duty_u16(int(abs(r)/100 * 65535))

# ================= WEB UI HTML =================
html = """<!DOCTYPE html><html><head><title>Pico Turbo</title>
<meta name="viewport" content="width=device-width, initial-scale=1">
<style>
body{{background:#111;color:#eee;text-align:center;font-family:sans-serif}}
.val{{font-size:2em;color:#0af;font-family:monospace}}
button{{padding:15px;width:40%;margin:5px;border-radius:8px;font-weight:bold;border:none;cursor:pointer}}
input{{width:80%}}
</style></head><body>
<h1>Pico 1000RPM Controller</h1>
<button style="background:#2ecc71" onclick="fetch('/start')">START</button>
<button style="background:#e74c3c" onclick="fetch('/stop')">STOP</button>
<form id="f">
<p>Base Speed: <span id="sv" class="val">{s}</span>% <input type="range" id="s" min="0" max="100" value="{s}"></p>
<p>Kp: <span id="kpv" class="val">{kp}</span> <input type="number" id="kp" step="0.5" value="{kp}"></p>
<p>Kd: <span id="kdv" class="val">{kd}</span> <input type="number" id="kd" step="1" value="{kd}"></p>
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
    
    print("Robot IP:", wlan.ifconfig()[0])
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
    gap_start = 0 
    
    while True:
        if robot_active:
            v = [s.value() for s in sensors]
            active_indices = [i for i, val in enumerate(v) if val]
            
            # 1. Line Detection & Memory Logic
            if active_indices:
                # LINE FOUND
                gap_start = 0 
                search_start = 0 
                err = sum(WEIGHTS[i] for i in active_indices) / len(active_indices)
            else:
                # LINE LOST (Use Memory for Sharp Turns)
                if gap_start == 0: gap_start = time.ticks_ms()
                gap_elapsed = time.ticks_diff(time.ticks_ms(), gap_start)

                if gap_elapsed < GAP_TIMEOUT_MS:
                    # RECOVERY: Keep steering hard in the last known direction
                    err = 25.0 if last_err > 0 else -25.0
                else:
                    # SEARCH: Sweep pattern
                    if search_start == 0:
                        search_start = time.ticks_ms()
                        search_dir = 1 if last_err >= 0 else -1
                    
                    search_elapsed = time.ticks_diff(time.ticks_ms(), search_start)
                    if search_elapsed < 250: err = 20.0 * search_dir
                    elif search_elapsed < 750: err = -20.0 * search_dir
                    else: err = 0 
            
            # 2. PID Calculation
            derivative = err - last_err
            corr = (KP * err) + (KD * derivative)
            
            # 3. Dynamic Speed Logic (Pivot Braking)
            if abs(err) > 12.0:
                # Hard turn detected: slow down to let motors reverse (Pivot)
                current_base = BASE_SPEED * 0.25
            elif abs(err) < 0.5:
                current_base = BASE_SPEED + TURBO_BOOST
            else:
                current_base = BASE_SPEED
            
            # 4. Apply Motors
            set_motors(current_base + corr, current_base - corr)
            last_err = err 
            
        else:
            set_motors(0, 0)
            last_err = 0
            gap_start = 0
            search_start = 0
            time.sleep(0.1)
            
        time.sleep(LOOP_DELAY)

# ================= EXECUTION =================
print("System Initializing...")
_thread.start_new_thread(web_server, ())
drive()
