# Requires time of flight sensor driver: https://github.com/drakxtwo/vl53l1x_pico

from machine import Pin, I2C
from vl53l1x import VL53L1X
import time
import sys

# TODO: Add delay variable to allow person to settle before stillness detection
# TODO: Add sensitivity variable to adjust distance threshold for person detection
# TODO: Add logging functionality to record stillness events with timestamps
# TODO: Add a better LED indicator
# TODO: Add a blinking status light to show the system is running
# TODO: Add power-saving features for battery operation

version = "1.0"
print("Stillness Detector - Version:", version)

try:
    i2c = I2C(0)
    time_of_flight = VL53L1X(i2c)
except Exception as e:
    print("Failed to initialize VL53L1X:", e)
    sys.exit(1)

motion = Pin(3, Pin.IN)
LED = Pin("LED", Pin.OUT)
LED.value(0)

def read_distance():
    # Trigger measurement and read distance
    try:
        distance = time_of_flight.read()
        return distance
    except Exception as e:
        print("Error reading distance:", e)
        return None

# Main loop to read distance every second
while True:
    distance = read_distance()
    print("Distance (mm):", distance, "PIR State:", motion.value())
    if distance is not None and distance < 1300:
        print("person detected")
        if motion.value() == 1:
            print("motion detected")
            LED.value(0)
        else:
            print("STILLNESS DETECTED!")
            LED.value(1)
    else:
        print("No person detected")
        LED.value(0)
    time.sleep(0.5)