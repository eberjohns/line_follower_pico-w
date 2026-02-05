# Complete MicroPython Code for Pico W Line Follower Robot
# Version: Pure PID Competition with Web Start/Stop (v5)
# Description: Pure PID follower with web tuning and remote Start/Stop.

import machine
import network
import socket
import time
import _thread
import ure # Using regular expressions for parsing

# --- Configuration ---
# Pins
MOTOR_AIN1_PIN = 5
MOTOR_AIN2_PIN = 4
MOTOR_BIN1_PIN = 3
MOTOR_BIN2_PIN = 2
SENSOR_PINS = [15, 14, 13, 12, 9, 8, 7, 6] # Example: GP15(S1)..GP6(S8) - Verify!

# WiFi Credentials
SSID = "POCO M2 Pro" # <<< --- UPDATE
PASSWORD = "123456789" # <<< --- UPDATE

# Motor & PID Defaults
PWM_FREQUENCY = 15000
MAX_DUTY = 65535
BASE_SPEED_PERCENT = 75.0 # (Tune %)

# Initial PID Gains (Tune!)
PID_KP = 15
PID_KI = 0
PID_KD = 0

# PID State
integral = 0.0
last_error = 0.0
integral_limit = 200.0

# Sensor Weights (Tune!)
SENSOR_WEIGHTS = [-0, -2, -1, -0.5, 0.5, 1, 2, 0]
NUM_SENSORS = len(SENSOR_PINS)

# Timing
LOOP_DELAY_S = 0.01

# --- Global Variables ---
# For Tuning
pid_lock = _thread.allocate_lock()
current_pid_gains = {'kp': PID_KP, 'ki': PID_KI, 'kd': PID_KD}

# For Robot State (Start/Stop)
state_lock = _thread.allocate_lock()
robot_active = False # Start in stopped state

# --- Motor Setup ---
motor_ain1 = machine.PWM(machine.Pin(MOTOR_AIN1_PIN))
motor_ain2 = machine.PWM(machine.Pin(MOTOR_AIN2_PIN))
motor_bin1 = machine.PWM(machine.Pin(MOTOR_BIN1_PIN))
motor_bin2 = machine.PWM(machine.Pin(MOTOR_BIN2_PIN))

for pwm in [motor_ain1, motor_ain2, motor_bin1, motor_bin2]:
    pwm.freq(PWM_FREQUENCY)
    pwm.duty_u16(0) # Start stopped

def set_motors(left_speed_percent, right_speed_percent):
    """ Sets motor speeds using percentages (-100 to 100). """
    left_speed_percent = max(-100, min(100, left_speed_percent))
    right_speed_percent = max(-100, min(100, right_speed_percent))
    left_duty = int(abs(left_speed_percent / 100.0) * MAX_DUTY)
    right_duty = int(abs(right_speed_percent / 100.0) * MAX_DUTY)

    if left_speed_percent >= 0:
        motor_ain1.duty_u16(left_duty)
        motor_ain2.duty_u16(0)
    else:
        motor_ain1.duty_u16(0)
        motor_ain2.duty_u16(left_duty)
    if right_speed_percent >= 0:
        motor_bin1.duty_u16(right_duty)
        motor_bin2.duty_u16(0)
    else:
        motor_bin1.duty_u16(0)
        motor_bin2.duty_u16(right_duty)

def stop_motors():
    set_motors(0, 0)

# --- Sensor Setup ---
# 1=Black (LED OFF), 0=White (LED ON) - Verify!
LINE_VALUE = 1
NO_LINE_VALUE = 0
sensors = [machine.Pin(pin, machine.Pin.IN) for pin in SENSOR_PINS]

def read_sensors():
    return [s.value() for s in sensors]

# --- WiFi Connection ---
def connect_wifi(ssid, password):
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    if not wlan.isconnected():
        print(f"Connecting to WiFi network '{ssid}'...")
        wlan.connect(ssid, password)
        max_wait = 15
        while max_wait > 0 and not wlan.isconnected():
             if wlan.status() < 0 or wlan.status() >= 3: break
             max_wait -= 1 ; print('.', end='') ; time.sleep(1)
    if wlan.isconnected():
        config = wlan.ifconfig()
        print("\nConnected to WiFi. IP Info:", config)
        return config[0]
    else:
        print(f"\nWiFi connection failed. Status: {wlan.status()}")
        return None

# --- Web Server for Tuning & Control ---
# Modified HTML to include Start/Stop buttons and status display
html_template = """<!DOCTYPE html><html><head><title>Pico PID Tuner & Control</title>
<meta name="viewport" content="width=device-width, initial-scale=1">
<style>body{{font-family:sans-serif; background:#222; color:#eee;}} label{{display:block; margin:5px 0;}} span{{font-weight:bold; color:lightblue;}} input[type=range]{{width: 70%;}} button{{padding:10px 15px; margin:5px; font-size:16px;}} #status{{color:{status_color}; font-weight:bold;}}</style>
</head><body>
<h2>Robot Control</h2>
<p>Status: <span id="status">{robot_status}</span></p>
<button onclick="fetch('/start')">Start Robot</button>
<button onclick="fetch('/stop')">Stop Robot</button>
<hr>
<h2>PID Tuning</h2>
<form id="pidForm">
<label for="speed">Base Speed: <input type="range" id="speed" min="0" max="100" step="1" value="{speed}"> <span id="speed_val">{speed}</span>%</label>
<hr>
<label for="kp">Kp: <input type="range" id="kp" name="kp" min="0" max="100" step="0.1" value="{kp}"> <span id="kp_val">{kp}</span></label>
<label for="ki">Ki: <input type="range" id="ki" name="ki" min="0" max="10" step="0.01" value="{ki}"> <span id="ki_val">{ki}</span></label>
<label for="kd">Kd: <input type="range" id="kd" name="kd" min="0" max="70" step="0.1" value="{kd}"> <span id="kd_val">{kd}</span></label>
<br><button type="submit">Update Gains</button>
</form>
<script>
  const ids = ['kp', 'ki', 'kd', 'speed'];
  ids.forEach(id => {{
      const slider = document.getElementById(id);
      const display = document.getElementById(id + '_val');
      slider.oninput = () => display.textContent = slider.value;
  }});
  document.getElementById('pidForm').onsubmit = (e) => {{
      e.preventDefault();
      const params = ids.map(id => id + '=' + document.getElementById(id).value).join('&');
      fetch('/?' + params);
  }};
</script></body></html>"""

def web_server_thread():
    global current_pid_gains, robot_active,BASE_SPEED_PERCENT # Add robot_active
    ip_address = connect_wifi(SSID, PASSWORD)
    if not ip_address:
        print("Web server cannot start without WiFi connection.")
        return

    addr = socket.getaddrinfo('0.0.0.0', 80)[0][-1]
    s = socket.socket()
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind(addr)
    s.listen(1)
    print(f"Web server running -> http://{ip_address}")

    while True:
        try:
            cl, addr = s.accept()
            request_bytes = cl.recv(1024)
            if not request_bytes: # Handle empty request
                cl.close()
                continue
            request = request_bytes.decode('utf-8')
            # print("Request:", request[:60]) # Print start of request for debugging

            # --- Handle Control Commands ---
            command_processed = False
            if 'GET /start' in request:
                with state_lock:
                    robot_active = True
                print(">>> Robot STARTED via Web <<<")
                command_processed = True
            elif 'GET /stop' in request:
                with state_lock:
                    robot_active = False
                # Stop motors immediately from web thread for responsiveness
                stop_motors()
                print(">>> Robot STOPPED via Web <<<")
                command_processed = True

            # --- Handle PID Tuning ---
            pid_updated = False
            if 'GET /?' in request:
                 params = {}
                 try:
                     query_part=request.split('?')[1].split(' ')[0]
                     pairs=query_part.split('&')
                     for pair in pairs:
                         if '=' in pair:
                             key,val=pair.split('=')
                             params[key]=val
                 except Exception as e:
                    print("Parsing error", e)
                     
                 
                 
                 
                 
                 
                 '''matches = ure.findall(r'([\w_]+)=([\d\.]+)', request)
                 for key, value in matches:
                     params[key] = value'''

                 try:
                     kp = float(params.get('kp', current_pid_gains['kp']))
                     ki = float(params.get('ki', current_pid_gains['ki']))
                     kd = float(params.get('kd', current_pid_gains['kd']))
                     new_speed = float(params.get('speed', BASE_SPEED_PERCENT))
                     with pid_lock:
                         current_pid_gains['kp'] = kp
                         current_pid_gains['ki'] = ki
                         current_pid_gains['kd'] = kd
                         BASE_SPEED_PERCENT = new_speed
                     pid_updated = True
                 except (ValueError, KeyError, TypeError): pass # Ignore parsing errors

                 if pid_updated: print(f"PID Updated -> Kp:{kp:.2f}, Ki:{ki:.2f}, Kd:{kd:.2f}")
                 command_processed = True # A tuning update also counts as processing

            # --- Send Response ---
            # Send status even if no command was processed (page refresh)
            with pid_lock, state_lock: # Read current values safely
                status_text = "RUNNING" if robot_active else "STOPPED"
                color = "lightgreen" if robot_active else "orange"
                html_resp = html_template.format(
                    kp=current_pid_gains['kp'], ki=current_pid_gains['ki'], kd=current_pid_gains['kd'],speed=BASE_SPEED_PERCENT,
                    robot_status=status_text, status_color=color
                )
            cl.send('HTTP/1.0 200 OK\r\nContent-type: text/html\r\n\r\n')
            cl.send(html_resp)
            cl.close()

        except OSError as e:
            cl.close()
            # print('Web server connection error:', e)
        except Exception as e:
            print(f"Web server critical error: {e}")
            # Consider stopping robot on server error?
            # with state_lock: robot_active = False
            # stop_motors()

# --- Line Position Logic (Simplified) ---
def get_line_position(sensor_values):
    weighted_sum = 0.0
    active_count = 0
    for i in range(NUM_SENSORS):
        if sensor_values[i] == LINE_VALUE:
            weighted_sum += SENSOR_WEIGHTS[i]
            active_count += 1
    if active_count == 0: return 0.0, False # Error, Line Found Flag
    else: return weighted_sum / active_count, True

# --- Main Line Following Loop (Checks robot_active flag) ---
def line_follow_main():
    global integral, last_error, current_pid_gains, robot_active
    print("Main loop started. Waiting for 'Start' command via web interface...")
    # Indicate stopped state visually? (e.g., solid LED)
    led = machine.Pin("LED", machine.Pin.OUT)
    led.off()

    while True:
        with state_lock: # Check if robot should be active
            is_active = robot_active

        if is_active:
            print("Base speed:",BASE_SPEED_PERCENT)
            led.on() # LED on when running
            # --- Read Current PID Values ---
            with pid_lock:
                kp = current_pid_gains['kp']
                ki = current_pid_gains['ki']
                kd = current_pid_gains['kd']

            # --- Read Sensors & Get Error ---
            sensor_values = read_sensors()
            error, line_found = get_line_position(sensor_values)

            # --- Handle Line Loss ---
            if not line_found:
                error = last_error # Use last known error to keep turning
                integral = 0.0 # Reset integral

            # --- PID Calculation ---
            integral += error
            integral = max(min(integral, integral_limit), -integral_limit) # Clamp
            derivative = error - last_error
            correction = (kp * error) + (ki * integral * LOOP_DELAY_S) + (kd * derivative / LOOP_DELAY_S)
            last_error = error

            # --- Motor Speed Calculation ---
            target_left = BASE_SPEED_PERCENT + correction
            target_right = BASE_SPEED_PERCENT - correction
            final_left = max(-100, min(100, target_left))
            final_right = max(-100, min(100, target_right))

            set_motors(final_left, final_right)

        else: # Robot is stopped
            led.off() # LED off when stopped
            stop_motors()
            # Reset PID state when stopped? Optional, but can prevent jumps on restart
            integral = 0.0
            last_error = 0.0
            # Sleep longer when stopped to reduce CPU usage
            time.sleep(0.1)
            continue # Skip the rest of the loop and the final sleep

        # Wait for the next loop cycle ONLY if active
        time.sleep(LOOP_DELAY_S)

# --- Start Threads ---
try:
    print("Starting web server thread...")
    _thread.start_new_thread(web_server_thread, ())
    time.sleep(5) # Wait for WiFi/Server
    print("Starting main line following loop (initially stopped)...")
    line_follow_main() # Main blocking loop

except Exception as e:
    print(f"FATAL ERROR in main execution: {e}")
    stop_motors()
    # Flash LED on error
    try: led = machine.Pin("LED", machine.Pin.OUT)
    except: led = None # Handle case where LED pin is unavailable
    if led:
        while True: led.toggle(); time.sleep(0.1)

finally:
    print("Exiting program...")
    stop_motors()

