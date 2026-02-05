import machine
import network
import socket
import time
import _thread

# ================= HARDWARE CONFIG =================
MOTOR_AIN1_PIN, MOTOR_AIN2_PIN = 5, 4
MOTOR_BIN1_PIN, MOTOR_BIN2_PIN = 3, 2
SENSOR_PINS = [15, 14, 13, 12, 9, 8, 7, 6]
BUTTON_PIN = 20

# ================= WIFI =================
SSID = "POCO M2 Pro"
PASSWORD = "123456789"

# ================= CONTROL =================
PWM_FREQ = 20000
LOOP_DELAY = 0.005

BASE_SPEED = 95.0
KP, KD = 30.0, 2.5

WEIGHTS = [-5.0, -2.5, -0.5, -0.1, 0.1, 0.5, 2.5, 5.0]

# ================= STATE =================
state_lock = _thread.allocate_lock()
robot_active = False
start_time_ms = 0

# ================= HARDWARE INIT =================
btn = machine.Pin(BUTTON_PIN, machine.Pin.IN, machine.Pin.PULL_UP)
led = machine.Pin("LED", machine.Pin.OUT)
sensors = [machine.Pin(p, machine.Pin.IN) for p in SENSOR_PINS]

pwms = [machine.PWM(machine.Pin(p)) for p in [5,4,3,2]]
for p in pwms:
    p.freq(PWM_FREQ)
    p.duty_u16(0)

# ================= MOTOR =================
def set_motors(l, r):
    l = max(-100, min(100, l))
    r = max(-100, min(100, r))

    pwms[0].duty_u16(int(abs(l)/100 * 65535) if l >= 0 else 0)
    pwms[1].duty_u16(0 if l >= 0 else int(abs(l)/100 * 65535))
    pwms[2].duty_u16(int(abs(r)/100 * 65535) if r >= 0 else 0)
    pwms[3].duty_u16(0 if r >= 0 else int(abs(r)/100 * 65535))

# ================= WEB UI =================
html = """<!DOCTYPE html><html><head>
<title>Pico Turbo</title>
<meta name="viewport" content="width=device-width, initial-scale=1">
<style>
body{{background:#111;color:#eee;text-align:center;font-family:sans-serif}}
.time{{font-size:3em;color:#0af;font-family:monospace}}
button{{padding:15px;width:40%;margin:5px;border-radius:8px;font-weight:bold}}
input{{width:80%}}
</style></head>
<body>
<div class="time">Time: <span id="t">0.00</span>s</div>
<button style="background:#2ecc71" onclick="start()">START</button>
<button style="background:#e74c3c" onclick="stop()">STOP</button>

<form id="f">
<label>Speed: <span id="sv">{s}</span>%<br>
<input type="range" id="s" min="0" max="100" value="{s}"></label><br>

<label>Kp: <span id="kpv">{kp}</span><br>
<input type="range" id="kp" min="0" max="100" step="0.5" value="{kp}"></label><br>

<label>Kd: <span id="kdv">{kd}</span><br>
<input type="range" id="kd" min="0" max="100" step="0.5" value="{kd}"></label><br>

<button type="submit" style="width:85%;background:#444;color:#fff">UPDATE</button>
</form>

<script>
let st=null, timer=null;

function start(){{
 if(st!==null)return;
 st=Date.now();
 fetch('/start');
 timer=setInterval(()=>{{
  document.getElementById('t').innerText=((Date.now()-st)/1000).toFixed(2);
 }},50);
}}

function stop(){{
 fetch('/stop');
 st=null;
 if(timer){{clearInterval(timer);timer=null;}}
}}

['s','kp','kd'].forEach(id=>{{
 document.getElementById(id).oninput=()=>document.getElementById(id+'v').innerText=document.getElementById(id).value;
}});

document.getElementById('f').onsubmit=e=>{{
 e.preventDefault();
 fetch('/?s='+s.value+'&kp='+kp.value+'&kd='+kd.value);
}};
</script>
</body></html>
"""

# ================= WEB SERVER =================
def web_server():
    global BASE_SPEED, KP, KD, robot_active, start_time_ms

    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    wlan.connect(SSID, PASSWORD)

    for _ in range(20):
        if wlan.isconnected():
            break
        led.toggle()
        time.sleep(0.5)
    else:
        print("WiFi failed")
        return

    print("URL:", wlan.ifconfig()[0])

    s = socket.socket()
    s.bind(('0.0.0.0', 80))
    s.listen(1)

    while True:
        try:
            cl, _ = s.accept()
            req = cl.recv(2048).decode()

            if 'GET /start' in req:
                with state_lock:
                    robot_active = True
                    start_time_ms = time.ticks_ms()

            elif 'GET /stop' in req:
                with state_lock:
                    robot_active = False

            elif 'GET /?' in req:
                import ure
                m = ure.search(r's=(\d+)&kp=([\d.]+)&kd=([\d.]+)', req)
                if m:
                    BASE_SPEED = float(m.group(1))
                    KP = float(m.group(2))
                    KD = float(m.group(3))

            cl.send("HTTP/1.0 200 OK\r\nContent-Type: text/html\r\n\r\n" +
                    html.format(s=BASE_SPEED, kp=KP, kd=KD))
            cl.close()
        except:
            pass

# ================= DRIVE LOOP =================
def drive():
    global robot_active
    last_err = 0

    while True:
        if btn.value() == 0:
            time.sleep(0.05)
            if btn.value() == 0:
                with state_lock:
                    robot_active = not robot_active
                while btn.value() == 0:
                    time.sleep(0.1)

        if robot_active:
            led.on()
            v = [s.value() for s in sensors]

            if sum(v) >= 7:
                with state_lock:
                    robot_active = False
                set_motors(0, 0)
                continue

            mid = v[2:6]
            if sum(mid):
                err = sum(WEIGHTS[i+2] for i,val in enumerate(mid) if val) / sum(mid)
            else:
                act = [i for i,val in enumerate(v) if val]
                err = sum(WEIGHTS[i] for i in act)/len(act) if act else last_err

            corr = KP*err + KD*(err-last_err)/LOOP_DELAY
            last_err = err

            set_motors(BASE_SPEED+corr, BASE_SPEED-corr)
        else:
            led.off()
            set_motors(0,0)
            last_err = 0
            time.sleep(0.1)

        time.sleep(LOOP_DELAY)

# ================= START =================
_thread.start_new_thread(web_server, ())
drive()
