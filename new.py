import machine
import network
import socket
import time
import _thread
import ure

# ================= HARDWARE CONFIG =================
L_PINS = [5, 4, 3]  
R_PINS = [2, 1, 0]
SENSOR_PINS = [15, 14, 13, 12, 9, 8, 7, 6]

# ================= TUNING PARAMETERS (Global) =================
SSID = "Sam"
PASSWORD = "samuel2006"
PWM_FREQ = 10000
LOOP_DELAY = 0.001 

# Values now updateable via Web UI
BASE_SPEED = 65.0       
KP = 35.0               
KD = 250.0              
HARD_THRESHOLD = 18.0   
WEIGHTS = [-25, -15, -8, -2, 2, 8, 15, 25]

# ================= STATE & HARDWARE INIT =================
robot_active = False
sensors = [machine.Pin(p, machine.Pin.IN) for p in SENSOR_PINS]

def init_motor(pins):
    pwm = machine.PWM(machine.Pin(pins[0]))
    pwm.freq(PWM_FREQ)
    return pwm, machine.Pin(pins[1], machine.Pin.OUT), machine.Pin(pins[2], machine.Pin.OUT)

l_pwm, l_in1, l_in2 = init_motor(L_PINS)
r_pwm, r_in1, r_in2 = init_motor(R_PINS)

def set_motors(l, r):
    l = max(-95, min(95, l))
    r = max(-95, min(95, r))
    l_in1.value(1 if l >= 0 else 0)
    l_in2.value(0 if l >= 0 else 1)
    l_pwm.duty_u16(int(abs(l) * 655.35))
    r_in1.value(1 if r >= 0 else 0)
    r_in2.value(0 if r >= 0 else 1)
    r_pwm.duty_u16(int(abs(r) * 655.35))

# ================= WEB UI HTML =================
def get_html():
    return f"""<!DOCTYPE html><html><head><title>Pico Tuning</title>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <style>
        body{{background:#111;color:#eee;text-align:center;font-family:sans-serif}}
        input{{width:60px;padding:8px;margin:5px;background:#222;color:#0af;border:1px solid #444;border-radius:5px}}
        button{{padding:15px;width:120px;margin:10px;border-radius:8px;border:none;font-weight:bold;cursor:pointer}}
        .save{{background:#0af;color:#fff;width:80%}}
    </style></head><body>
    <h1>Pico 1000RPM</h1>
    <button style="background:#2ecc71" onclick="fetch('/start')">START</button>
    <button style="background:#e74c3c" onclick="fetch('/stop')">STOP</button>
    <hr>
    <form action="/">
        Speed: <input name="s" value="{BASE_SPEED}"><br>
        Kp: <input name="kp" value="{KP}"><br>
        KD: <input name="kd" value="{KD}"><br>
        Pivot: <input name="pt" value="{HARD_THRESHOLD}"><br>
        <button type="submit" class="save">UPDATE PARAMETERS</button>
    </form>
    </body></html>"""

# ================= WEB SERVER THREAD =================
def web_server():
    global BASE_SPEED, KP, KD, HARD_THRESHOLD, robot_active
    wlan = network.WLAN(network.STA_IF); wlan.active(True); wlan.connect(SSID, PASSWORD)
    while not wlan.isconnected(): time.sleep(0.5)
    print("Robot IP:", wlan.ifconfig()[0])
    
    s = socket.socket(); s.bind(('0.0.0.0', 80)); s.listen(1)
    while True:
        try:
            cl, _ = s.accept()
            req = cl.recv(1024).decode()
            
            # Handle Actions
            if 'GET /start' in req: robot_active = True
            if 'GET /stop' in req: robot_active = False
            
            # Parse Parameters (Regex for s, kp, kd, pt)
            m = ure.search(r's=([\d.]+)&kp=([\d.]+)&kd=([\d.]+)&pt=([\d.]+)', req)
            if m:
                BASE_SPEED = float(m.group(1))
                KP = float(m.group(2))
                KD = float(m.group(3))
                HARD_THRESHOLD = float(m.group(4))
            
            cl.send("HTTP/1.1 200 OK\r\nContent-Type: text/html\r\n\r\n" + get_html())
            cl.close()
        except Exception as e:
            print("Web error:", e)

# ================= MAIN DRIVE LOOP =================
def drive():
    global robot_active
    last_err = 0
    search_dir = 1 
    
    while True:
        if not robot_active:
            set_motors(0, 0)
            time.sleep(0.1)
            continue

        v = [s.value() for s in sensors]
        active = [i for i, val in enumerate(v) if val]

        if active:
            err = sum(WEIGHTS[i] for i in active) / len(active)
            if err > 2: search_dir = 1
            elif err < -2: search_dir = -1
            
            derivative = err - last_err
            correction = (KP * err) + (KD * derivative)

            if abs(err) >= HARD_THRESHOLD:
                # AGGRESSIVE TURN: Opposite motor direction
                set_motors(correction, -correction)
            else:
                # SMOOTH TURN: Adjusting both speeds
                set_motors(BASE_SPEED + correction, BASE_SPEED - correction)
            
            last_err = err
        else:
            # PERSISTENT SEARCH: Turn until line is found
            search_speed = BASE_SPEED * 0.7
            if search_dir == 1:
                set_motors(search_speed, -search_speed)
            else:
                set_motors(-search_speed, search_speed)
        
        time.sleep(LOOP_DELAY)

# ================= EXECUTION =================
_thread.start_new_thread(web_server, ())
drive()
