# Requires time of flight sensor driver: https://github.com/drakxtwo/vl53l1x_pico

from machine import Pin, I2C
from vl53l1x import VL53L1X
import time
import sys

time.sleep(2) # allow usb connection on startup

# TODO: Add logging functionality to record stillness events with timestamps
# TODO: Add power-saving features for battery operation

version = "1.0.9"
print("Stillness Detector - Version:", version)

PIR_RESET_TIME = 1.5  # seconds to wait after motion detected
MAX_DISTANCE = 2000
MIN_DISTANCE = 300
SETTLING_DELAY = 3
DAMPING_INTERVAL = 5
ALLOWABLE_DISTANCE_CHANGE = 300  # mm

MODE = "normal"  # options: "normal", "relaxed", "strict"

if MODE == "relaxed":
    SETTLING_DELAY = 3
    DAMPING_INTERVAL = 2
    ALLOWABLE_DISTANCE_CHANGE = 1000 
elif MODE == "normal":
    SETTLING_DELAY = 5
    DAMPING_INTERVAL = 7
    ALLOWABLE_DISTANCE_CHANGE = 300
elif MODE == "strict":
    SETTLING_DELAY = 10
    DAMPING_INTERVAL = 20
    ALLOWABLE_DISTANCE_CHANGE = 200

# Initialize I2C and Time-of-Flight sensor with retries
attempts = 0
while attempts < 3:
    try:
        i2c = I2C(0)
        time_of_flight = VL53L1X(i2c)
        break
    except Exception as e:
        attempts += 1
        print("Attempt", attempts, "to initialize VL53L1X failed:", e)
        time.sleep(1)
    if attempts == 3:    
        print("Failed to initialize VL53L1X:")
        sys.exit(1)

motion_sensor = Pin(3, Pin.IN)
STATUS_LED = Pin("LED", Pin.OUT)
STATUS_LED.value(0)
RED = Pin(18, Pin.OUT)
GREEN = Pin(19, Pin.OUT)
BLUE = Pin(20, Pin.OUT)
RED.value(0)
GREEN.value(0)
BLUE.value(0)

class StillnessDetector:
    def __init__(self, max_distance=1500, min_distance=50, delay=2, damping_interval=10):
        self.max_distance = max_distance
        self.min_distance = min_distance
        self.delay = delay
        self.presence_start_time = None
        self.elapsed_time = None
        self.stillness_detected = False
        self.movement_dampening_interval = damping_interval
        self.last_motion_time = time.time()
        self.start_distance = None

    def is_person_detected(self, distance):
        return distance is not None and self.min_distance < distance < self.max_distance
    
    def get_status(self):
        if self.stillness_detected:
            return "still"
        elif (self.presence_start_time is not None):
            return "present"
        else:
            return "absent"
    
    def update(self, distance, motion):
        if self.is_person_detected(distance):
            if self.presence_start_time is None: # update with info on initial detection
                self.presence_start_time = time.time()
                self.last_motion_time = time.time()
                self.start_distance = distance
                print("Distance threshold crossed, starting timer")
            else:
                self.elapsed_time = time.time() - self.presence_start_time # otherwise add to elapsed time
        else:
            print("Presence not detected")
            self.reset()
        if self.elapsed_time is not None:
            if abs(distance - self.start_distance) > ALLOWABLE_DISTANCE_CHANGE:
                print("Significant distance change detected, resetting timer")
                self.reset()
            if self.elapsed_time is not None and self.elapsed_time > self.delay:
                if motion == 0:
                    self.stillness_detected = True
                    print("Person has been still for", self.elapsed_time, "seconds")
                else:
                    if (time.time() - self.last_motion_time > self.movement_dampening_interval 
                        or time.time() - self.last_motion_time < PIR_RESET_TIME):
                        self.last_motion_time = time.time()
                        print("Ignoring brief motion")
                    else:
                        print("Person is in motion")
                        self.reset()
            else:
                if motion == 1:
                    self.last_motion_time = time.time()
                    print("Motion detected, resetting stillness timer")
                    self.reset()

    def reset(self):
        self.presence_start_time = None
        self.elapsed_time = None
        self.start_distance = None
        self.stillness_detected = False
    
    def set_max_distance(self, threshold):
        self.max_distance = threshold
        print("Max distance set to:", threshold)

    def set_min_distance(self, threshold):
        self.min_distance = threshold
        print("Min distance set to:", threshold)

    def set_delay(self, delay):
        self.delay = delay
        print("Delay set to:", delay)

    def get_distance(self):
        return self.max_distance
    
    def get_motion(self):
        return self.min_distance

def read_distance():
    # Trigger measurement and read distance
    try:
        distance = time_of_flight.read()
        return distance
    except Exception as e:
        print("Error reading distance:", e)
        return None
    
def set_led_color(r, g, b):
    RED.value(r)
    GREEN.value(g)
    BLUE.value(b)

def blink_led(pin, times, interval=0.2):
    for _ in range(times):
        pin.on()
        time.sleep(interval)
        pin.off()
        time.sleep(interval)

detector = StillnessDetector(max_distance=MAX_DISTANCE, min_distance=MIN_DISTANCE, delay=SETTLING_DELAY, damping_interval=DAMPING_INTERVAL)

# Main loop to read distance every second
def main():
    while True:
        if time.time() - detector.last_motion_time < DAMPING_INTERVAL:
            blink_led(STATUS_LED, 2, interval=0.1)
        else:
            blink_led(STATUS_LED, 1, interval=0.2)
        distance = read_distance()
        motion_state =  motion_sensor.value()
        print("Distance (mm):", distance, "PIR State:", motion_state)
        detector.update(distance, motion_state)

        if detector.get_status() == "still":
            set_led_color(0, 1, 0)  # Green for stillness detected
        elif detector.get_status() == "present":
            set_led_color(0, 0, 1)  # Blue for presence detected
        else:
            set_led_color(1, 0, 0)  # Red for no presence detected
        time.sleep(0.5)

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("Program interrupted by user")
        STATUS_LED.value(0)
        RED.value(0)
        GREEN.value(0)
        BLUE.value(0)
        sys.exit(0)