import machine, network, socket, time, _thread, ure

# Force WiFi Reset on startup
wlan = network.WLAN(network.STA_IF)
wlan.active(False)
time.sleep(0.5)
wlan.active(True)

# ================= 🛠️ CONFIGURABLE PARAMETERS =================
cfg = {
    "fwd": 150, "klr": 220, "kb": 400,
    "s_map": 50.0,  
    "s_race": 50.0, 
    "t": 75.0,      
    "kp": 1.0
}

# ================= ⚙️ HARDWARE CONFIG =================
L_PINS = [5, 4, 3]; R_PINS = [2, 1, 0]
SENSOR_PINS = [15, 14, 13, 12, 9, 8, 7, 6]
LED_FINISH = machine.Pin(16, machine.Pin.OUT)

# ================= 📶 WIFI =================
SSID, PASSWORD = 'Bsnl Ftth Lonappan', '4885285968'

# ================= 🧠 MAZE MEMORY =================
raw_path = []      # The "dirty" path with dead ends
solved_path = []   # The optimized LSR string
mode = "IDLE"      # IDLE, MAPPING, RACING
race_index = 0

def solve_maze():
    global raw_path, solved_path
    path = "".join(raw_path)
    rules = {"LBL":"S", "LBS":"R", "RBL":"B", "SBL":"R", "SBS":"B", "LBR":"B"}
    
    # Repeatedly simplify until no 'B' remains
    while "B" in path:
        old_path = path
        for key, val in rules.items():
            path = path.replace(key, val)
        if path == old_path: break
    
    solved_path = list(path)

# ================= 🔧 MOTOR & SENSOR INIT =================
def init_motor(pins):
    pwm = machine.PWM(machine.Pin(pins[0])); pwm.freq(15000)
    return pwm, machine.Pin(pins[1], machine.Pin.OUT), machine.Pin(pins[2], machine.Pin.OUT)

l_pwm, l_in1, l_in2 = init_motor(L_PINS)
r_pwm, r_in1, r_in2 = init_motor(R_PINS)
sensors = [machine.Pin(p, machine.Pin.IN) for p in SENSOR_PINS]

SAFE_LIMIT = 0.5 

def set_motors(l, r):
    l, r = max(-100, min(100, l)), max(-100, min(100, r))
    l_safe, r_safe = l * SAFE_LIMIT, r * SAFE_LIMIT

    l_in1.value(1 if l_safe >= 0 else 0)
    l_in2.value(0 if l_safe >= 0 else 1)
    r_in1.value(1 if r_safe >= 0 else 0)
    r_in2.value(0 if r_safe >= 0 else 1)

    l_pwm.duty_u16(int(abs(l_safe) / 100 * 65535))
    r_pwm.duty_u16(int(abs(r_safe) / 100 * 65535))

# ================= 🧭 NAVIGATION HELPERS =================
def execute_turn(node, speed):
    # 1. Move forward to center
    set_motors(speed, speed)
    found_finish = False
    for _ in range(cfg["fwd"] // 10):
        time.sleep_ms(10)
        if sum([s.value() for s in sensors]) >= 8:
            found_finish = True; break
    if found_finish: return "FINISH"

    # 2. Execute Rotation
    if node != 'S':
        set_motors(0, 0); time.sleep_ms(50)
        if node == 'L': set_motors(-cfg["t"], cfg["t"])
        else: set_motors(cfg["t"], -cfg["t"])
        
        # Initial kick to move off current line
        time.sleep_ms(150 if node == 'B' else cfg["klr"])
        # Wait for center sensors to find the new line
        while sensors[3].value() == 0 and sensors[4].value() == 0: pass
        
        # Active Brake
        if node == 'L': set_motors(cfg["t"], -cfg["t"])
        else: set_motors(-cfg["t"], cfg["t"])
        time.sleep_ms(20)

    # 3. Clear Junction (The Gatekeeper)
    # Don't return until wing sensors are on white
    while sum([sensors[i].value() for i in [0, 1, 6, 7]]) > 0:
        set_motors(speed * 0.7, speed * 0.7)
        
    set_motors(0,0) # Brief pause to sync logic
    return "DONE"

def finish_sequence():
    global mode
    set_motors(0, 0)
    LED_FINISH.on()
    mode = "IDLE"

# ================= 📱 WEB DASHBOARD =================
html_template = """<!DOCTYPE html><html><head><title>MAZE MASTER</title>
<meta name="viewport" content="width=device-width, initial-scale=1">
<style>
    body{{background:#111;color:#eee;text-align:center;font-family:sans-serif}}
    .box{{background:#222;padding:12px;margin:8px;border-radius:10px;border:1px solid #444}}
    .path{{font-size:1.3em;color:#2ecc71;background:#000;padding:10px;word-break:break-all;border:1px solid #333}}
    button{{padding:15px;width:30%;margin:5px;border-radius:8px;font-weight:bold;border:none;cursor:pointer}}
</style></head><body>
    <h2>Mode: <span style="color:#0af">{mode}</span></h2>
    <div class="box">
        <button style="background:#2ecc71;color:#fff" onclick="fetch('/map')">DRY RUN</button>
        <button style="background:#f39c12;color:#fff" onclick="fetch('/solve')">SOLVE</button>
        <button style="background:#3498db;color:#fff" onclick="fetch('/race')">ACTUAL</button><br>
        <button style="background:#e74c3c;width:95%;margin-top:10px" onclick="fetch('/stop')">STOP</button>
    </div>
    <div class="box"><h3>RAW PATH</h3><div class="path" style="color:#e67e22">{raw}</div></div>
    <div class="box"><h3>SOLVED PATH</h3><div class="path">{sol}</div></div>
    <script>setInterval(()=>{{ if(!window.location.search) location.reload(); }}, 2000);</script>
</body></html>"""

def web_server():
    global mode, solved_path, raw_path, cfg, race_index
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    wlan.connect(SSID, PASSWORD)
    while not wlan.isconnected(): time.sleep(0.5)
    
    s = socket.socket()
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1) 
    s.bind(('0.0.0.0', 80))
    s.listen(1)
    
    while True:
        try:
            cl, _ = s.accept(); req = cl.recv(1024).decode()
            if 'GET /map' in req: 
                raw_path=[]; solved_path=[]; mode="MAPPING"
                LED_FINISH.off()
            elif 'GET /solve' in req: 
                solve_maze()
            elif 'GET /race' in req: 
                race_index=0; mode="RACING"
                LED_FINISH.off()
            elif 'GET /stop' in req: 
                mode="IDLE"
            
            r_str = " → ".join(raw_path) if raw_path else "Empty"
            s_str = " → ".join(solved_path) if solved_path else "Not Solved"
            cl.send("HTTP/1.0 200 OK\r\n\r\n" + html_template.format(mode=mode, raw=r_str, sol=s_str, **cfg, sm=cfg["s_map"], sr=cfg["s_race"]))
            cl.close()
        except: pass

# ================= 🏎️ THE MAIN ENGINE =================
def drive():
    global mode, solved_path, raw_path, race_index
    while True:
        v = [s.value() for s in sensors]
        s_sum = sum(v)

        if mode == "MAPPING":
            if s_sum >= 7: finish_sequence(); continue
            if v[0] == 1 or v[1] == 1: # Left Junction
                if execute_turn('L', cfg["s_map"]) == "FINISH": finish_sequence(); continue
                raw_path.append('L'); continue
            if s_sum == 0: # Dead End
                execute_turn('B', cfg["s_map"])
                raw_path.append('B'); continue
            if v[6] == 1 or v[7] == 1: # Right/Straight Junction
                node = 'S' if sum(v[2:6]) > 0 else 'R'
                if execute_turn(node, cfg["s_map"]) == "FINISH": finish_sequence(); continue
                raw_path.append(node); continue

            err = (v[5]*15 + v[4]*5) - (v[3]*5 + v[2]*15)
            set_motors(cfg["s_map"] + (err * cfg["kp"]), cfg["s_map"] - (err * cfg["kp"]))

        elif mode == "RACING":
            if s_sum >= 7: finish_sequence(); continue
            # Intersection detected by wings or total loss (Dead end in race = turn)
            if (v[0]==1 or v[1]==1 or v[6]==1 or v[7]==1) or (s_sum == 0):
                if race_index < len(solved_path):
                    execute_turn(solved_path[race_index], cfg["s_race"])
                    race_index += 1
                continue

            err = (v[5]*15 + v[4]*5) - (v[3]*5 + v[2]*15)
            set_motors(cfg["s_race"] + (err * cfg["kp"]), cfg["s_race"] - (err * cfg["kp"]))
        else:
            set_motors(0, 0); time.sleep(0.1)

_thread.start_new_thread(web_server, ())
drive()
