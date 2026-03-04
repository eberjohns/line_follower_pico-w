import machine, time, struct

# 1. SETUP I2C (Your "Safe" Way)
scl_pin = machine.Pin(18, machine.Pin.IN)
sda_pin = machine.Pin(19, machine.Pin.IN)
i2c = machine.SoftI2C(scl=scl_pin, sda=sda_pin, freq=400000)

MPU_ADDR = 0x68

def init_mpu():
    # Wake up the MPU6050
    i2c.writeto_mem(MPU_ADDR, 0x6B, b'\x00')
    print("MPU6050 Woken Up")

def get_gyro_z_raw():
    # Read 2 bytes from the Gyro Z-axis register (0x47)
    data = i2c.readfrom_mem(MPU_ADDR, 0x47, 2)
    # '>h' means big-endian signed short (16-bit)
    return struct.unpack('>h', data)[0]

# --- CALIBRATION & TRACKING ---
init_mpu()
print("CALIBRATING... Keep the bot perfectly still!")

# Calculate the average noise (bias) over 200 readings
bias = 0
for _ in range(200):
    bias += get_gyro_z_raw()
    time.sleep_ms(5)
gyro_offset = bias / 200

print(f"Calibration Done. Offset: {gyro_offset:.2f}")
print("Starting Angle Tracking... (Ctrl+C to stop)")

angle = 0.0
last_time = time.ticks_ms()

while True:
    # 1. Get raw data and subtract bias
    raw = get_gyro_z_raw()
    gz_velocity = (raw - gyro_offset) / 131.0  # Convert to degrees per second
    
    # 2. Calculate time elapsed (dt)
    now = time.ticks_ms()
    dt = time.ticks_diff(now, last_time) / 1000.0  # convert ms to seconds
    last_time = now
    
    # 3. Integrate: New Angle = Old Angle + (Velocity * Time)
    # Only add if motion is above a tiny noise threshold
    if abs(gz_velocity) > 0.1:
        angle += gz_velocity * dt
    
    # 4. Print to shell
    print(f"Current Angle: {angle:6.1f}° | Raw: {raw:5d}", end='\r')
    
    time.sleep_ms(10) # 100Hz sampling rate