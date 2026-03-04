import machine, network, socket, time, _thread, ure

# ================= 🛡️ WIFI RESET (Soft Reboot Fix) =================
wlan = network.WLAN(network.STA_IF)
wlan.active(False)
time.sleep(0.1)
wlan.active(True)

SSID, PASSWORD = "POCO M2 Pro", "123456789"

# ================= ⚙️ HARDWARE CONFIG =================
# Dual Motor Driver Pins: [PWM, IN1, IN2]
L_PINS = [5, 4, 3] 
R_PINS = [2, 1, 0]
SENSOR_PINS = [15, 14, 13, 12, 9, 8, 7, 6]

# ================= ⚡ VOLTAGE SAFETY (12V to 6V) =================
SAFE_LIMIT = 0.5 

# ================= 🏎️ TUNING PARAMETERS =================
PWM_FREQ = 25000
LOOP_DELAY = 0.005 # 5ms

# Global variables controlled by Web UI
BASE_SPEED = 70.0
KP = 18.0
KD = 45.0
TURBO_BOOST = 25.0
# Weighting: Inner sensors have low weights, Outer have high weights
WEIGHTS = [0.0, -5.0, -3.0, -0.2, 0.2, 3.0, 5.0, 0.0]

# ================= 🔧 INIT HARDWARE =================
led = machine.Pin("LED", machine.Pin.OUT)
sensors = [machine.Pin(p, machine.Pin.IN) for p in SENSOR_PINS]

def init_motor(pins):
    pwm = machine.PWM(machine.Pin(pins[0]))
    pwm.freq(PWM_FREQ)
    in1 = machine.Pin(pins[1], machine.Pin.OUT)
    in2 = machine.Pin(pins[2], machine.Pin.OUT)
    return pwm, in1, in2

l_pwm, l_in1, l_in2 = init_motor(L_PINS)
r_pwm, r_in1, r_in2 = init_motor(R_PINS)

# State Variables
state_lock = _thread.allocate_lock()
robot_active = False
search_start = 0
search_dir = 1 

def set_motors(l, r):
    # Clamp and apply Safety Limit for 3S battery
    l_safe = max(-100, min(100, l)) * SAFE_LIMIT
    r_safe = max(-100, min(100, r)) * SAFE_LIMIT

    # Left Motor Control
    l_in1.value(1 if l_safe >= 0 else 0)
    l_in2.value(0 if l_safe >= 0 else 1)
    l_pwm.duty_u16(int(abs(l_safe) / 100 * 65535))

    # Right Motor Control
    r_in1.value(1 if r_safe >= 0 else 0)
    r_in2.value(0 if r_safe >= 0 else 1)
    r_pwm.duty_u16(int(abs(r_safe) / 100 * 65535))

# ================= 📱 WEB UI =================
html = """<!DOCTYPE html><html><head><title>Pico Turbo</title>
<meta name="viewport" content="width=device-width, initial-scale=1">
<style>
body{{background:#111;color:#eee;text-align:center;font-family:sans-serif}}
button{{padding:15px;width:45%;margin:5px;border-radius:8px;font-weight:bold;border:none}}
input{{width:80%}}
</style></head><body>
<h2>RACE CONTROL</h2>
<button style="background:#2ecc71" onclick="fetch('/start')">START</button>
<button style="background:#e74c3c" onclick="fetch('/stop')">STOP</button>
<form id="f">
<p>Base Speed: <span id="sv">{s}</span> <input type="range" id="s" min="0" max="100" value="{s}"></p>
<p>Kp: <span id="kpv">{kp}</span> <input type="number" id="kp" step="0.1" value="{kp}"></p>
<p>Kd: <span id="kdv">{kd}</span> <input type="number" id="kd" step="0.1" value="{kd}"></p>
<button type="submit" style="width:93%;background:#444;color:#fff">UPDATE GAINS</button>
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

def web_server():
    global BASE_SPEED, KP, KD, robot_active
    wlan.connect(SSID, PASSWORD)
    while not wlan.isconnected(): 
        led.toggle(); time.sleep(0.5)
    print("Robot IP:", wlan.ifconfig()[0])
    led.on()

    s = socket.socket()
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind(('0.0.0.0', 80))
    s.listen(1)

    while True:
        try:
            cl, _ = s.accept(); req = cl.recv(1024).decode()
            if 'GET /start' in req:
                with state_lock: robot_active = True
            elif 'GET /stop' in req:
                with state_lock: robot_active = False
            elif 'GET /?' in req:
                m = ure.search(r's=([\d.]+)&kp=([\d.]+)&kd=([\d.]+)', req)
                if m:
                    BASE_SPEED, KP, KD = float(m.group(1)), float(m.group(2)), float(m.group(3))
            cl.send("HTTP/1.0 200 OK\r\n\r\n" + html.format(s=BASE_SPEED, kp=KP, kd=KD))
            cl.close()
        except: pass

# ================= 🏎️ COMPETITION LOGIC DRIVE LOOP =================
def drive():
    global robot_active, search_start, search_dir
    last_err = 0
    
    while True:
        if robot_active:
            v = [s.value() for s in sensors]
            
            # 1. Finish Detection (All Black)
            if sum(v) >= 7:
                with state_lock: robot_active = False
                set_motors(0, 0)
                continue

            # 2. Line Detection & Weighted Error
            active_indices = [i for i, val in enumerate(v) if val]
            
            if active_indices:
                search_start = 0 
                # Original Weighted Average Logic
                err = sum(WEIGHTS[i] for i in active_indices) / len(active_indices)
            else:
                # LINE LOST: Sweep Search Logic
                if search_start == 0:
                    search_start = time.ticks_ms()
                    search_dir = 1 if last_err >= 0 else -1
                
                elapsed = time.ticks_diff(time.ticks_ms(), search_start)
                if elapsed < 250:   err = 5.0 * search_dir   # Quick turn
                elif elapsed < 750:  err = -5.0 * search_dir  # 180 sweep
                elif elapsed < 1500: err = 5.0 * search_dir   # Wide search
                else:                err = 0                  # Stop/Straight

            # 3. PID Calculation
            derivative = err - last_err
            corr = (KP * err) + (KD * derivative)
            last_err = err

            # 4. Turbo Logic (Boost on straights)
            current_base = (BASE_SPEED + TURBO_BOOST) if abs(err) < 0.2 else BASE_SPEED
            
            # 5. Apply to Motors
            set_motors(current_base + corr, current_base - corr)
        else:
            set_motors(0, 0)
            last_err = 0
            search_start = 0
            time.sleep(0.1)
            
        time.sleep(LOOP_DELAY)

# ================= EXECUTION =================
print("Starting System...")
_thread.start_new_thread(web_server, ())
drive()