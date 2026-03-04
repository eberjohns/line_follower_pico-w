import machine, network, socket, time, _thread, ure, struct

# ================= 🛡️ WIFI RESET (Soft Reboot Fix) =================
wlan = network.WLAN(network.STA_IF)
wlan.active(False)
time.sleep(0.1)
wlan.active(True)

# ================= 🛠️ CURRENT CALIBRATED SETTINGS =================
cfg = {
    "fwd": 220, "klr": 280, "kb": 580,
    "s": 40.0, "t": 45.0, "kp": 12.0
}

# ================= ⚙️ HARDWARE CONFIG =================
L_PINS = [5, 4, 3]; R_PINS = [2, 1, 0]
SENSOR_PINS = [15, 14, 13, 12, 9, 8, 7, 6]
LED_FINISH = machine.Pin(16, machine.Pin.OUT)
I2C_SCL, I2C_SDA = 18, 19 # I2C0 for MPU6050

# ================= 📶 WIFI CONFIG =================
SSID, PASSWORD = "Bsnl Ftth Lonappan", "4885285968"

# ================= 🧠 NAVIGATION & STATE =================
path_log = []
last_node = ""
robot_active = False
state_lock = _thread.allocate_lock()

current_yaw = 0.0      # Live angle
target_heading = 0.0   # Where we want to be (0, 90, 180, 270)
gyro_bias = 0.0        # Set during startup calibration

# ================= 🔧 MPU6050 & MOTOR INIT =================
i2c = machine.I2C(0, scl=machine.Pin(I2C_SCL), sda=machine.Pin(I2C_SDA))
MPU_ADDR = 0x68

def init_mpu():
    global gyro_bias
    try:
        # Check if device is actually on the bus before writing
        devices = i2c.scan()
        if MPU_ADDR not in devices:
            print("MPU6050 not found at 0x68. Check wiring!")
            return
            
        i2c.writeto_mem(MPU_ADDR, 0x6B, b'\x00') # Wake up
        print("Calibrating Gyro... Keep Bot Still")
        samples = 500
        total = 0
        for _ in range(samples):
            data = i2c.readfrom_mem(MPU_ADDR, 0x47, 2)
            total += struct.unpack('>h', data)[0]
            time.sleep_ms(2)
        gyro_bias = total / samples
        print("Gyro Calibrated. Bias:", gyro_bias)
    except Exception as e:
        print("MPU6050 Error:", e)

def get_gz():
    try:
        data = i2c.readfrom_mem(MPU_ADDR, 0x47, 2)
        raw = struct.unpack('>h', data)[0]
        # 131.0 is sensitivity for +/- 250 deg/s
        return (raw - gyro_bias) / 131.0
    except: return 0

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

# ================= 🧭 GYRO ASSISTED TURNS =================
def execute_gyro_turn(node):
    global current_yaw, target_heading
    
    # 1. Drive forward slightly to center the wheels on the node
    set_motors(cfg["s"], cfg["s"]); time.sleep_ms(cfg["fwd"])
    set_motors(0, 0); time.sleep_ms(100)
    
    # 2. Update target heading
    if node == 'L': target_heading += 90
    elif node == 'R': target_heading -= 90
    elif node == 'B': target_heading += 180
    
    # 3. Rotate using Gyro PID/Threshold
    last_t = time.ticks_ms()
    while True:
        # We must keep integrating yaw during the turn loop
        dt = time.ticks_diff(time.ticks_ms(), last_t) / 1000.0
        last_t = time.ticks_ms()
        current_yaw += get_gz() * dt
        
        error = target_heading - current_yaw
        if abs(error) < 4: break # Stop when close to 90/180
        
        # Turn speed
        p_speed = cfg["t"] if error > 0 else -cfg["t"]
        set_motors(-p_speed, p_speed)
        time.sleep_ms(5)

    # 4. Final Hunt: Fine-tune until IR sensors find the line
    while sensors[3].value() == 0 and sensors[4].value() == 0:
        hunt_speed = 35 # Slow crawl to catch the line
        if node == 'L': set_motors(-hunt_speed, hunt_speed)
        else: set_motors(hunt_speed, -hunt_speed)
    
    # 5. Reset Drift: Force yaw to exactly the target
    current_yaw = target_heading 
    set_motors(0, 0); time.sleep_ms(50)

# ================= 📱 WEB DASHBOARD =================
html_template = """<!DOCTYPE html><html><head><title>ROBO-DASH</title>
<meta name="viewport" content="width=device-width, initial-scale=1">
<style>
    body{{background:#111;color:#eee;text-align:center;font-family:sans-serif}}
    .box{{background:#222;padding:15px;margin:10px;border-radius:12px;border:1px solid #444}}
    .path-log{{font-size:1.2em;color:#0af;background:#000;padding:15px;word-break:break-all}}
    .gyro-val{{font-size:2em;color:#f1c40f;font-weight:bold}}
    button{{padding:15px;width:90%;margin:5px;border-radius:8px;font-weight:bold;border:none}}
    input{{width:50px;padding:5px;background:#333;color:#fff;border:1px solid #555}}
</style></head><body>
    <h2>Gyro Path Logger</h2>
    <div class="box">
        <div class="gyro-val">ANGLE: {yaw:.1f}°</div>
        <div style="color:#888">Target: {target:.0f}°</div>
    </div>
    <div class="box">
        <button style="background:#2ecc71;color:#fff" onclick="fetch('/start')">START BOT</button>
        <button style="background:#e74c3c;color:#fff" onclick="fetch('/stop')">STOP</button>
    </div>
    <div class="box">
        <div class="path-log">[{path}]</div>
    </div>
    <form action="/">
    <div class="box">
        Fwd:<input name="fwd" value="{fwd}"> Kp:<input name="kp" value="{kp}"><br><br>
        Spd:<input name="s" value="{s}"> Turn:<input name="t" value="{t}">
        <button type="submit" style="background:#444;color:#fff;margin-top:10px">UPDATE PARAMS</button>
    </div>
    </form>
    <script>setInterval(()=>{{ if(!window.location.search) location.reload(); }}, 1500);</script>
</body></html>"""

def web_server():
    global robot_active, path_log, cfg
    wlan = network.WLAN(network.STA_IF); wlan.active(True); wlan.connect(SSID, PASSWORD)
    while not wlan.isconnected(): time.sleep(0.5)
    print("Web Server at:", wlan.ifconfig()[0])
    
    s = socket.socket(); s.bind(('0.0.0.0', 80)); s.listen(1)
    while True:
        try:
            cl, _ = s.accept(); req = cl.recv(1024).decode()
            if 'GET /start' in req:
                with state_lock: path_log=[]; robot_active=True
            elif 'GET /stop' in req:
                with state_lock: robot_active=False
            elif 'GET /?' in req:
                m = ure.search(r'fwd=(\d+)&kp=([\d.]+)&s=([\d.]+)&t=([\d.]+)', req)
                if m:
                    cfg["fwd"] = int(m.group(1))
                    cfg["kp"], cfg["s"], cfg["t"] = float(m.group(2)), float(m.group(3)), float(m.group(4))
            
            p_str = ", ".join([f'"{i}"' for i in path_log]) if path_log else "EMPTY"
            cl.send("HTTP/1.0 200 OK\r\n\r\n" + html_template.format(
                yaw=current_yaw, target=target_heading, path=p_str, 
                fwd=cfg["fwd"], kp=cfg["kp"], s=cfg["s"], t=cfg["t"]))
            cl.close()
        except: pass

# ================= 🏎️ DRIVE LOOP (CORE 1) =================
def drive():
    global robot_active, path_log, last_node, current_yaw
    init_mpu()
    last_time = time.ticks_ms()
    
    while True:
        # Constant Gyro Integration
        now = time.ticks_ms()
        dt = time.ticks_diff(now, last_time) / 1000.0
        last_time = now
        current_yaw += get_gz() * dt

        if robot_active:
            v = [s.value() for s in sensors]
            
            # 1. FINISH
            if sum(v) >= 7:
                set_motors(0, 0); LED_FINISH.on(); robot_active = False
                print("Final Path:", path_log)
                continue

            # 2. LEFT INTERSECTION
            if v[0] == 1 or v[1] == 1:
                path_log.append('L'); last_node='L'
                execute_gyro_turn('L')
                continue

            # 3. DEAD END
            if sum(v) == 0:
                path_log.append('B'); last_node='B'
                execute_gyro_turn('B')
                continue

            # 4. RIGHT (Only if no straight path)
            if (v[6] == 1 or v[7] == 1) and sum(v[2:6]) == 0:
                path_log.append('R'); last_node='R'
                execute_gyro_turn('R')
                continue

            # 5. STRAIGHT PID
            if sum(v[2:6]) > 0:
                if last_node != 'S': path_log.append('S'); last_node='S'
                err = (v[5]*15 + v[4]*5) - (v[3]*5 + v[2]*15)
                set_motors(cfg["s"] + (err * cfg["kp"]), cfg["s"] - (err * cfg["kp"]))
        else:
            set_motors(0, 0); LED_FINISH.off(); time.sleep(0.05)

# Start drive logic on Core 1, Web server on Core 0
_thread.start_new_thread(drive, ())
web_server()
