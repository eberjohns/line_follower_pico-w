# ================= 🛠️ SYSTEM CONFIGURATION =================
import machine
import time

# ================= 🏎️ FLOW-STATE SETTINGS =================
base_speed = 85.0    # High speed for straights
kp = 0.8             # Lowered because we are squaring the error now
kd = 12.0            # Damping to stop the "shaking"
sampling_ms = 10     # 10ms is often smoother for N20 gearboxes
alpha = 0.8          # Stronger filter for cleaner motor signals

# Global PID states
last_error = 0
filtered_d = 0

# ================= ⚙️ HARDWARE INIT =================
# Tuning Buttons
btn_down = machine.Pin(16, machine.Pin.IN, machine.Pin.PULL_UP)
btn_up = machine.Pin(17, machine.Pin.IN, machine.Pin.PULL_UP)

# Motors
l_pwm = machine.PWM(machine.Pin(27)); l_pwm.freq(15000)
l_in1, l_in2 = machine.Pin(26, machine.Pin.OUT), machine.Pin(22, machine.Pin.OUT)
r_pwm = machine.PWM(machine.Pin(11)); r_pwm.freq(15000)
r_in1, r_in2 = machine.Pin(10, machine.Pin.OUT), machine.Pin(12, machine.Pin.OUT)

# Sensors (Left to Right)
sensors = [machine.Pin(p, machine.Pin.IN) for p in [9, 8, 7, 6, 3, 2, 1, 0]]

def set_motors(l, r):
    # Clamping for safety
    l, r = max(-100, min(100, l)), max(-100, min(100, r))
    l_in1.value(1 if l >= 0 else 0); l_in2.value(0 if l >= 0 else 1)
    l_pwm.duty_u16(int(abs(l)/100 * 65535))
    r_in1.value(1 if r >= 0 else 0); r_in2.value(0 if r >= 0 else 1)
    r_pwm.duty_u16(int(abs(r)/100 * 65535))

def get_weighted_error(v):
    # Normalized weights (-1.0 to 1.0) for easier math
    weights = [-1.5, -1.0, -0.5, -0.2, 0.2, 0.5, 1.0, 1.5]
    num = sum(v[i] * weights[i] for i in range(8))
    den = sum(v)
    return num / den if den > 0 else None

# ================= 🏁 MISSION CONTROL =================
while True:
    sensor_values = [s.value() for s in sensors]
    current_error = get_weighted_error(sensor_values)
    
    if current_error is None:
        # Emergency search (Spin)
        speed = 45
        if last_error < 0: set_motors(-speed, speed)
        else: set_motors(speed, -speed)
        continue

    # --- 1. NON-LINEAR ERROR (The Secret Sauce) ---
    # We square the error: 0.1 becomes 0.01 (tiny), but 1.0 stays 1.0 (big).
    # This removes jitter on straights.
    shaped_error = (current_error ** 2) * (1 if current_error > 0 else -1)

    # --- 2. PD CALCULATIONS ---
    p_term = shaped_error * kp * 100 # Multiplier to bring it back to scale
    
    raw_d = current_error - last_error
    filtered_d = (alpha * filtered_d) + ((1 - alpha) * raw_d)
    d_term = filtered_d * kd * 10
    
    correction = p_term + d_term

    # --- 3. DYNAMIC SPEED STEERING ---
    # On straights (error ~0), both motors run at base_speed.
    # On curves, we keep the outer wheel fast and slow down the inner wheel.
    if current_error > 0: # Turning Right
        l_speed = base_speed
        r_speed = base_speed - correction
    else: # Turning Left
        l_speed = base_speed + correction # Correction is negative here
        r_speed = base_speed

    set_motors(l_speed, r_speed)
    
    last_error = current_error
    time.sleep_ms(sampling_ms)
