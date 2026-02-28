import machine, network, socket, time, _thread, ure

# ================= HARDWARE CONFIG =================
L_PINS = [5, 4, 3] # PWM, IN1, IN2
R_PINS = [2, 1, 0] # PWM, IN1, IN2
SENSOR_PINS = [15, 14, 13, 12, 9, 8, 7, 6]

# ================= WIFI CONFIG (POCO M2 Pro) =================
SSID = 'POCO M2 Pro'
PASSWORD = '123456789'

# ================= CALIBRATION STATE =================
forward_ms = 220  # New: Adjustment to center the wheels on the node
left_ms = 430
right_ms = 340
back_ms = 620
turn_speed = 45.0
fwd_speed = 40.0
test_type = "L" 
execute_test = False

# ================= MOTOR INIT =================
def init_motor(pins):
    pwm = machine.PWM(machine.Pin(pins[0]))
    pwm.freq(15000)
    return pwm, machine.Pin(pins[1], machine.Pin.OUT), machine.Pin(pins[2], machine.Pin.OUT)

l_pwm, l_in1, l_in2 = init_motor(L_PINS)
r_pwm, r_in1, r_in2 = init_motor(R_PINS)

def set_motors(l, r):
    l, r = max(-100, min(100, l)), max(-100, min(100, r))
    l_in1.value(1 if l >= 0 else 0); l_in2.value(0 if l >= 0 else 1)
    l_pwm.duty_u16(int(abs(l)/100 * 65535))
    r_in1.value(1 if r >= 0 else 0); r_in2.value(0 if r >= 0 else 1)
    r_pwm.duty_u16(int(abs(r)/100 * 65535))

# ================= WEB CALIBRATOR UI =================
html = """<!DOCTYPE html><html><head><title>MAZE TUNER</title>
<meta name="viewport" content="width=device-width, initial-scale=1">
<style>
    body{{background:#111;color:#eee;text-align:center;font-family:sans-serif}}
    .box{{background:#222;padding:15px;margin:10px;border-radius:12px;border:1px solid #444}}
    input[type=number]{{width:70px;padding:8px;background:#333;color:#fff;border:1px solid #555}}
    button{{padding:15px;width:90%;margin:8px;border-radius:8px;font-weight:bold;font-size:1.1em;border:none}}
    .fwd{{color:#0af;font-weight:bold}}
</style></head><body>
    <h1>Maze Tuner v2</h1>
    
    <form action="/">
        <div class="box">
            <label><input type="radio" name="t" value="L" {cL}> LEFT</label>
            <label><input type="radio" name="t" value="R" {cR}> RIGHT</label>
            <label><input type="radio" name="t" value="B" {cB}> BACK</label>
        </div>
        
        <div class="box">
            <p class="fwd">Forward Adjust: <input type="number" name="fms" value="{fms}"> ms</p>
            <hr style="border:0.5px solid #444">
            <p>Left Turn: <input type="number" name="lms" value="{lms}"> ms</p>
            <p>Right Turn: <input type="number" name="rms" value="{rms}"> ms</p>
            <p>Back Turn: <input type="number" name="bms" value="{bms}"> ms</p>
            <button type="submit" style="background:#444;color:#fff">SAVE ALL VALUES</button>
        </div>
    </form>

    <button style="background:#2ecc71;color:#fff" onclick="fetch('/test')">EXECUTE SEQUENCE</button>
    <button style="background:#e74c3c;color:#fff" onclick="fetch('/stop')">STOP</button>

</body></html>"""

def web_server():
    global forward_ms, left_ms, right_ms, back_ms, test_type, execute_test
    wlan = network.WLAN(network.STA_IF); wlan.active(True)
    wlan.connect(SSID, PASSWORD)
    while not wlan.isconnected(): time.sleep(0.5)
    
    s = socket.socket(); s.bind(('0.0.0.0', 80)); s.listen(1)
    while True:
        try:
            cl, _ = s.accept(); req = cl.recv(1024).decode()
            if 'GET /test' in req: execute_test = True
            elif 'GET /stop' in req: execute_test = False
            elif 'GET /?' in req:
                m = ure.search(r't=(L|R|B)&fms=(\d+)&lms=(\d+)&rms=(\d+)&bms=(\d+)', req)
                if m:
                    test_type = m.group(1)
                    forward_ms = int(m.group(2))
                    left_ms, right_ms, back_ms = int(m.group(3)), int(m.group(4)), int(m.group(5))
            
            cl.send("HTTP/1.0 200 OK\r\n\r\n" + html.format(
                fms=forward_ms, lms=left_ms, rms=right_ms, bms=back_ms,
                cL="checked" if test_type=="L" else "",
                cR="checked" if test_type=="R" else "",
                cB="checked" if test_type=="B" else ""))
            cl.close()
        except: pass

# ================= CALIBRATION LOOP =================
def drive():
    global execute_test
    while True:
        if execute_test:
            # STEP 1: MOVE FORWARD TO CENTER WHEELS
            print(f"Centering: {forward_ms}ms")
            set_motors(fwd_speed, fwd_speed)
            time.sleep_ms(forward_ms)
            
            # STEP 2: EXECUTE PIVOT
            print(f"Turning: {test_type}")
            if test_type == "L":
                set_motors(-turn_speed, turn_speed)
                time.sleep_ms(left_ms)
            elif test_type == "R":
                set_motors(turn_speed, -turn_speed)
                time.sleep_ms(right_ms)
            elif test_type == "B":
                set_motors(turn_speed, -turn_speed)
                time.sleep_ms(back_ms)
            
            set_motors(0, 0)
            execute_test = False 
        time.sleep(0.1)

_thread.start_new_thread(web_server, ())
drive()
