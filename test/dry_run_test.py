import machine, network, socket, time, _thread, ure

# ================= 🛠️ CURRENT CALIBRATED SETTINGS =================
cfg = {
    "fwd": 220, "klr": 280, "kb": 580,
    "s": 40.0, "t": 45.0, "kp": 12.0
}

# ================= ⚙️ HARDWARE CONFIG =================
L_PINS = [5, 4, 3]; R_PINS = [2, 1, 0]
SENSOR_PINS = [15, 14, 13, 12, 9, 8, 7, 6]
LED_FINISH = machine.Pin(16, machine.Pin.OUT)

# ================= 📶 WIFI (POCO M2 Pro) =================
SSID, PASSWORD = 'POCO M2 Pro', '123456789'

# ================= 🧠 PATH STORAGE =================
path_log = []      # The array we are testing
last_node = ""
robot_active = False
state_lock = _thread.allocate_lock()

# ================= 🔧 HARDWARE INIT =================
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

# ================= 🧭 HYBRID NAVIGATION =================
def execute_hybrid_turn(node):
    set_motors(cfg["s"], cfg["s"]); time.sleep_ms(cfg["fwd"])
    set_motors(0, 0); time.sleep_ms(50)
    
    if node == 'L': set_motors(-cfg["t"], cfg["t"])
    else: set_motors(cfg["t"], -cfg["t"])
    
    time.sleep_ms(cfg["klr"] if node != 'B' else cfg["kb"])
    
    # Hunt until center sensors find the line
    while sensors[3].value() == 0 and sensors[4].value() == 0:
        pass 
    set_motors(0, 0); time.sleep_ms(50)

# ================= 📱 WEB DASHBOARD =================
html_template = """<!DOCTYPE html><html><head><title>PATH TESTER</title>
<meta name="viewport" content="width=device-width, initial-scale=1">
<style>
    body{{background:#111;color:#eee;text-align:center;font-family:sans-serif}}
    .box{{background:#222;padding:15px;margin:10px;border-radius:12px;border:1px solid #444}}
    .path-log{{font-size:1.5em;color:#0af;background:#000;padding:20px;word-break:break-all;border:2px solid #0af}}
    button{{padding:15px;width:90%;margin:10px;border-radius:8px;font-weight:bold;font-size:1.1em;border:none}}
    input{{width:60px;padding:8px;background:#333;color:#fff;border:1px solid #555}}
</style></head><body>
    <h1>Path Logger Test</h1>
    <div class="box">
        <button style="background:#2ecc71;color:#fff" onclick="fetch('/start')">START DRY RUN</button>
        <button style="background:#e74c3c;color:#fff" onclick="fetch('/stop')">STOP</button>
    </div>
    <div class="box">
        <h3>RECORDED PATH ARRAY</h3>
        <div class="path-log" id="p">[{path}]</div>
    </div>
    <form action="/">
    <div class="box">
        <h3>Live Tuning</h3>
        Fwd:<input name="fwd" value="{fwd}"> KLR:<input name="klr" value="{klr}"> KB:<input name="kb" value="{kb}"><br>
        Spd:<input name="s" value="{s}"> Kp:<input name="kp" value="{kp}">
        <button type="submit" style="background:#444;color:#fff;margin-top:10px">SAVE PARAMS</button>
    </div>
    </form>
    <script>setInterval(()=>{{ if(!window.location.search) location.reload(); }}, 2000);</script>
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
                m = ure.search(r'fwd=(\d+)&klr=(\d+)&kb=(\d+)&s=([\d.]+)&kp=([\d.]+)', req)
                if m:
                    cfg["fwd"], cfg["klr"], cfg["kb"] = int(m.group(1)), int(m.group(2)), int(m.group(3))
                    cfg["s"], cfg["kp"] = float(m.group(4)), float(m.group(5))
            
            p_str = ", ".join([f'"{i}"' for i in path_log]) if path_log else "EMPTY"
            cl.send("HTTP/1.0 200 OK\r\n\r\n" + html_template.format(
                path=p_str, fwd=cfg["fwd"], klr=cfg["klr"], kb=cfg["kb"], s=cfg["s"], kp=cfg["kp"]))
            cl.close()
        except: pass

# ================= 🏎️ DRIVE LOOP =================
def drive():
    global robot_active, path_log, last_node
    while True:
        if robot_active:
            v = [s.value() for s in sensors]
            
            # 1. FINISH BOX
            if sum(v) >= 7:
                set_motors(0, 0); LED_FINISH.on(); robot_active = False
                print("Finished! Final Path:", path_log)
                continue

            # 2. LEFT NODE (Priority)
            if v[0] == 1 or v[1] == 1:
                path_log.append('L'); last_node='L'
                execute_hybrid_turn('L')
                continue

            # 3. DEAD END
            if sum(v) == 0:
                path_log.append('B'); last_node='B'
                execute_hybrid_turn('B')
                continue

            # 4. RIGHT TURN (Check if no straight ahead)
            if (v[6] == 1 or v[7] == 1) and sum(v[2:6]) == 0:
                path_log.append('R'); last_node='R'
                execute_hybrid_turn('R')
                continue

            # 5. STRAIGHT PID
            if sum(v[2:6]) > 0:
                if last_node != 'S':
                    path_log.append('S'); last_node='S'
                err = (v[5]*15 + v[4]*5) - (v[3]*5 + v[2]*15)
                set_motors(cfg["s"] + (err * cfg["kp"]), cfg["s"] - (err * cfg["kp"]))
        else:
            set_motors(0, 0); LED_FINISH.off(); time.sleep(0.1)

_thread.start_new_thread(web_server, ())
drive()