import machine, network, socket, time, _thread, struct

# 1. ROBUST CORE CLEARANCE
# This ensures that if you hit 'Run' in Thonny, the Pico hard-resets
# to kill any "ghost" threads on Core 1.
try:
    if machine.reset_cause() != machine.PWRON_RESET:
        machine.reset()
except:
    pass

# ================= 🛡️ SAFETY & CONFIG =================
SAFE_LIMIT = 0.5 
SSID, PASSWORD = 'POCO M2 Pro', '123456789'
L_PINS = [5, 4, 3]; R_PINS = [2, 1, 0]

# Hardware Setup
i2c = machine.SoftI2C(scl=machine.Pin(18), sda=machine.Pin(19), freq=400000)
MPU_ADDR = 0x68

# ================= 📶 WIFI RESET =================
wlan = network.WLAN(network.STA_IF)
wlan.active(False)
time.sleep(0.5)
wlan.active(True)

# Global State
angle = 0.0
gyro_offset = 0
turn_speed = 70 

# ================= 🔧 MOTOR INIT =================
def init_motor(pins):
    pwm = machine.PWM(machine.Pin(pins[0]))
    pwm.freq(1000) # Lower frequency = more torque for BO motors
    return pwm, machine.Pin(pins[1], machine.Pin.OUT), machine.Pin(pins[2], machine.Pin.OUT)

l_pwm, l_in1, l_in2 = init_motor(L_PINS)
r_pwm, r_in1, r_in2 = init_motor(R_PINS)

def set_motors(l, r):
    # Apply the 6V safety cap for your 3S battery
    l_safe, r_safe = max(-100, min(100, l)) * SAFE_LIMIT, max(-100, min(100, r)) * SAFE_LIMIT
    l_in1.value(1 if l_safe >= 0 else 0); l_in2.value(0 if l_safe >= 0 else 1)
    r_in1.value(1 if r_safe >= 0 else 0); r_in2.value(0 if r_safe >= 0 else 1)
    l_pwm.duty_u16(int(abs(l_safe)/100 * 65535))
    r_pwm.duty_u16(int(abs(r_safe)/100 * 65535))

# ================= 📐 GYRO ENGINE (ON CORE 1) =================
def gyro_core():
    global angle, gyro_offset
    try: 
        i2c.writeto_mem(MPU_ADDR, 0x6B, b'\x00') # Wake MPU
    except: 
        print("MPU6050 NOT FOUND")
        return
    
    # Auto-Calibration (Bot must be still!)
    temp_offset = 0
    for _ in range(500):
        data = i2c.readfrom_mem(MPU_ADDR, 0x47, 2)
        temp_offset += struct.unpack('>h', data)[0]
        time.sleep_ms(2)
    gyro_offset = temp_offset / 500
    
    last_time = time.ticks_ms()
    while True:
        try:
            data = i2c.readfrom_mem(MPU_ADDR, 0x47, 2)
            raw = struct.unpack('>h', data)[0]
            gz = (raw - gyro_offset) / 131.0
            now = time.ticks_ms()
            dt = time.ticks_diff(now, last_time) / 1000.0
            last_time = now
            # Noise gate to prevent "creeping" angle
            if abs(gz) > 0.8: 
                angle += gz * dt
        except: 
            pass
        time.sleep_ms(10)

# ================= 🧭 AUTO-TURN LOGIC =================
def auto_turn(target_degrees):
    global angle
    start_angle = angle
    # Turn logic
    if target_degrees > 0: set_motors(turn_speed, -turn_speed) # Right
    else: set_motors(-turn_speed, turn_speed)                # Left
    
    # Wait until angle reached (with 4-degree buffer for momentum)
    while abs(angle - start_angle) < abs(target_degrees) - 4:
        time.sleep_ms(5)
    
    set_motors(0, 0)
    print("Turn Complete. Final Angle:", round(angle, 1))

# ================= 📱 WEB SERVER (MAIN CORE) =================
def start_server():
    global angle
    wlan.connect(SSID, PASSWORD)
    while not wlan.isconnected(): 
        time.sleep(0.5)
    
    s = socket.socket()
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind(('0.0.0.0', 80))
    s.listen(1)
    print("Dashboard Ready! IP:", wlan.ifconfig()[0])

    html_body = """<!DOCTYPE html><html><head><meta name="viewport" content="width=device-width, initial-scale=1">
    <style>body{{background:#111;color:#fff;text-align:center;font-family:sans-serif}}.gauge{{font-size:3.5em;color:#2ecc71;margin:15px}}.btn{{width:80px;height:80px;margin:5px;background:#333;color:white;border:none;border-radius:12px;font-weight:bold}}</style>
    </head><body><h1>HARMONY TEST</h1><div class="gauge" id="ang">0.0°</div>
    <button class="btn" onmousedown="f('/F')" onmouseup="f('/S')">FWD</button><br>
    <button class="btn" onclick="f('/L90')" style="background:#3498db">L90</button>
    <button class="btn" onclick="f('/R90')" style="background:#3498db">R90</button><br>
    <button class="btn" onclick="f('/B180')" style="background:#3498db">B180</button>
    <button class="btn" onclick="f('/S')" style="background:#e74c3c">STOP</button>
    <script>function f(u){{fetch(u)}} setInterval(()=>{{fetch('/data').then(r=>r.text()).then(t=>{{document.getElementById('ang').innerText=t+'°'}}) }},250)</script></body></html>"""

    while True:
        try:
            cl, _ = s.accept()
            req = cl.recv(1024).decode()
            if 'GET /data' in req: 
                cl.send("HTTP/1.0 200 OK\r\n\r\n" + str(round(angle, 1)))
            elif 'GET /L90' in req: 
                auto_turn(-90); cl.send("HTTP/1.0 200 OK\r\n\r\nOK")
            elif 'GET /R90' in req: 
                auto_turn(90); cl.send("HTTP/1.0 200 OK\r\n\r\nOK")
            elif 'GET /B180' in req: 
                auto_turn(180); cl.send("HTTP/1.0 200 OK\r\n\r\nOK")
            elif 'GET /F' in req: 
                set_motors(100, 100); cl.send("HTTP/1.0 200 OK\r\n\r\nOK")
            elif 'GET /S' in req: 
                set_motors(0, 0); cl.send("HTTP/1.0 200 OK\r\n\r\nOK")
            else: 
                cl.send("HTTP/1.0 200 OK\r\n\r\n" + html_body)
            cl.close()
        except: 
            pass

# Start Gyro Thread on Core 1
_thread.start_new_thread(gyro_core, ())

# Start Web Server on Main Core (Core 0)
start_server()