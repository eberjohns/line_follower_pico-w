import machine, network, socket, time, _thread, ure

# ================= DEFAULT SETTINGS =================
cfg = {
    "fwd": 220, "kick_lr": 280, "kick_b": 580,
    "speed": 40.0, "turn": 45.0, "kp": 15.0
}

# =================  HARDWARE CONFIG =================
L_PINS = [5, 4, 3]; R_PINS = [2, 1, 0]
SENSOR_PINS = [15, 14, 13, 12, 9, 8, 7, 6]
LED_FINISH = machine.Pin(16, machine.Pin.OUT)

# ================= WIFI (POCO M2 Pro) =================
SSID, PASSWORD = 'POCO M2 Pro', '123456789'

# ================= GLOBAL STATE =================
path_log = []
last_node = "" 
robot_active = False
state_lock = _thread.allocate_lock()

# ================= HARDWARE INIT =================
def init_motor(pins):
    pwm = machine.PWM(machine.Pin(pins[0]))
    pwm.freq(15000)
    return pwm, machine.Pin(pins[1], machine.Pin.OUT), machine.Pin(pins[2], machine.Pin.OUT)

l_pwm, l_in1, l_in2 = init_motor(L_PINS)
r_pwm, r_in1, r_in2 = init_motor(R_PINS)
sensors = [machine.Pin(p, machine.Pin.IN) for p in SENSOR_PINS]

def set_motors(l, r):
    l, r = max(-100, min(100, l)), max(-100, min(100, r))
    l_in1.value(1 if l >= 0 else 0); l_in2.value(0 if l >= 0 else 1)
    l_pwm.duty_u16(int(abs(l)/100 * 65535))
    r_in1.value(1 if r >= 0 else 0); r_in2.value(0 if r >= 0 else 1)
    r_pwm.duty_u16(int(abs(r)/100 * 65535))

# ================= HYBRID NAVIGATION =================
def execute_hybrid_turn(node):
    set_motors(cfg["speed"], cfg["speed"])
    time.sleep_ms(cfg["fwd"])
    set_motors(0, 0); time.sleep_ms(50)

    if node == 'L':
        set_motors(-cfg["turn"], cfg["turn"])
        time.sleep_ms(cfg["kick_lr"])
    elif node == 'R' or node == 'B':
        set_motors(cfg["turn"], -cfg["turn"])
        time.sleep_ms(cfg["kick_lr"] if node == 'R' else cfg["kick_b"])

    # Wait for center-line recovery (Sensors 3 or 4)
    while sensors[3].value() == 0 and sensors[4].value() == 0:
        pass 
    set_motors(0, 0); time.sleep_ms(50)

# ================= WEB DASHBOARD =================
html_template = """<!DOCTYPE html><html><head><title>MAZE CONFIG</title>
<meta name="viewport" content="width=device-width, initial-scale=1">
<style>
    body{{background:#111;color:#eee;text-align:center;font-family:sans-serif}}
    .box{{background:#222;padding:10px;margin:5px;border-radius:10px;border:1px solid #444}}
    input{{width:60px;padding:8px;background:#333;color:#fff;border:1px solid #555}}
    button{{padding:12px;width:90%;margin:5px;border-radius:5px;font-weight:bold;border:none}}
    .path{{font-size:1.2em;color:#0af;background:#000;padding:10px;word-break:break-all}}
</style></head><body>
    <h2>Live Control</h2>
    <div class="box">
        <button style="background:#2ecc71" onclick="fetch('/start')">START</button>
        <button style="background:#e74c3c" onclick="fetch('/stop')">STOP</button>
    </div>
    <form action="/">
    <div class="box">
        <h3>Timings (ms)</h3>
        Fwd: <input type="number" name="fwd" value="{fwd}"> 
        KickLR: <input type="number" name="klr" value="{klr}"><br>
        KickB: <input type="number" name="kb" value="{kb}">
    </div>
    <div class="box">
        <h3>Speeds & PID</h3>
        Base: <input type="number" name="s" value="{s}"> 
        Turn: <input type="number" name="t" value="{t}"><br>
        Kp: <input type="number" name="kp" value="{kp}">
        <button type="submit" style="background:#444;color:#fff;margin-top:10px">UPDATE PARAMS</button>
    </div>
    </form>
    <div class="box"><h3>Path:</h3><div class="path">{path}</div></div>
    <script>setInterval(()=>{{ if(!window.location.search) location.reload(); }}, 3000);</script>
</body></html>"""

def web_server():
    global robot_active, path_log, cfg
    wlan = network.WLAN(network.STA_IF); wlan.active(True); wlan.connect(SSID, PASSWORD)
    while not wlan.isconnected(): time.sleep(0.5)
    s = socket.socket(); s.bind(('0.0.0.0', 80)); s.listen(1)
    while True:
        try:
            cl, _ = s.accept(); req = cl.recv(1024).decode()
            if 'GET /start' in req:
                with state_lock: path_log=[]; robot_active=True
            elif 'GET /stop' in req:
                with state_lock: robot_active=False
            elif 'GET /?' in req:
                m = ure.search(r'fwd=(\d+)&klr=(\d+)&kb=(\d+)&s=([\d.]+)&t=([\d.]+)&kp=([\d.]+)', req)
                if m:
                    cfg["fwd"], cfg["kick_lr"], cfg["kick_b"] = int(m.group(1)), int(m.group(2)), int(m.group(3))
                    cfg["speed"], cfg["turn"], cfg["kp"] = float(m.group(4)), float(m.group(5)), float(m.group(6))
            
            p_str = "->".join(path_log) if path_log else "Ready"
            cl.send("HTTP/1.0 200 OK\r\n\r\n" + html_template.format(
                fwd=cfg["fwd"], klr=cfg["kick_lr"], kb=cfg["kick_b"], 
                s=cfg["speed"], t=cfg["turn"], kp=cfg["kp"], path=p_str))
            cl.close()
        except: pass

# =================  DRIVE LOOP =================
def drive():
    global robot_active, path_log, last_node
    while True:
        if robot_active:
            v = [s.value() for s in sensors]
            
            # 1. FINISH
            if sum(v) >= 7:
                set_motors(0,0); LED_FINISH.on(); robot_active=False
                continue

            # 2. LEFT NODE (Sensors 0 or 1)
            if v[0] == 1 or v[1] == 1:
                path_log.append('L'); last_node='L'
                execute_hybrid_turn('L')
                continue

            # 3. DEAD END (Total sensor loss)
            if sum(v) == 0:
                path_log.append('B'); last_node='B'
                execute_hybrid_turn('B')
                continue

            # 4. RIGHT TURN (Sensors 6 or 7 AND middle is clear)
            if (v[6] == 1 or v[7] == 1) and sum(v[2:6]) == 0:
                path_log.append('R'); last_node='R'
                execute_hybrid_turn('R')
                continue

            # 5. EXPANDED LINE FOLLOWING (Sensors 2, 3, 4, 5)
            # This fixes the "stalling on 3rd sensor" issue
            if sum(v[2:6]) > 0:
                if last_node != 'S':
                    path_log.append('S'); last_node='S'
                
                # Proportional steering using all 4 middle sensors
                # Weights: S2=-15, S3=-5, S4=5, S5=15
                err = (v[5]*15 + v[4]*5) - (v[3]*5 + v[2]*15)
                set_motors(cfg["speed"] + (err * cfg["kp"]), cfg["speed"] - (err * cfg["kp"]))
        else:
            set_motors(0,0); LED_FINISH.off(); time.sleep(0.1)

_thread.start_new_thread(web_server, ())
drive()
