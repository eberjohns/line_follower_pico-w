import machine
import time

# Use your exact pin configuration
SENSOR_PINS = [15, 14, 13, 12, 9, 8, 7, 6]
sensors = [machine.Pin(p, machine.Pin.IN) for p in SENSOR_PINS]

print("--- Pico Sensor Debug Mode ---")
print("Move the robot over the line slowly.")
print("Format: [ S1 S2 S3 S4 S5 S6 S7 S8 ]")
print("------------------------------")

try:
    while True:
        # Read all sensor values (1 for line, 0 for floor)
        vals = [s.value() for s in sensors]
        
        # Create a visual string: '#' for detected, '.' for nothing
        visual = ""
        for v in vals:
            visual += " # " if v else " . "
            
        # Print on one line using \r to refresh
        print(f"[{visual}] Pins: {vals}", end="\r")
        
        time.sleep(0.1)
except KeyboardInterrupt:
    print("\nTesting stopped.")
