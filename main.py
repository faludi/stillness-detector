# Requires time of flight sensor driver: https://github.com/drakxtwo/vl53l1x_pico

from machine import Pin, I2C
from vl53l1x import VL53L1X
import time
import sys
import settings

time.sleep(2) # allow usb connection on startup

# TODO: Add optional power-saving features for battery operation

version = "1.0.13"
print("Stillness Detector - Version:", version)

PIR_RESET_TIME = settings.PIR_RESET_TIME
MAX_DISTANCE = settings.MAX_DISTANCE
MIN_DISTANCE = settings.MIN_DISTANCE

MODE = settings.MODE

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
elif MODE == "custom":
    SETTLING_DELAY =  settings.SETTLING_DELAY
    DAMPING_INTERVAL = settings.DAMPING_INTERVAL
    ALLOWABLE_DISTANCE_CHANGE = settings.ALLOWABLE_DISTANCE_CHANGE
else:
    print("Invalid MODE in settings.py")
    sys.exit(1)

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

# Set up other inputs and outputs
motion_sensor = Pin(3, Pin.IN, Pin.PULL_DOWN)
STATUS_LED = Pin("LED", Pin.OUT)
STATUS_LED.value(0)
RED = Pin(18, Pin.OUT)
GREEN = Pin(19, Pin.OUT)
BLUE = Pin(20, Pin.OUT)
RED.value(0)
GREEN.value(0)
BLUE.value(0)
ABSENT_OUTPUT = Pin(7, Pin.OUT)
PRESENT_OUTPUT = Pin(8, Pin.OUT)
STILL_OUTPUT = Pin(9, Pin.OUT)
ABSENT_OUTPUT.value(0)
PRESENT_OUTPUT.value(0)
STILL_OUTPUT.value(0)

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
        # Check if distance reading is valid and within thresholds
        return distance is not None and self.min_distance < distance < self.max_distance
    
    def get_status(self):
        # Returns "still" if stillness detected, "present" if presence detected but not still, and "absent" if no presence detected
        if self.stillness_detected:
            return "still"
        elif (self.presence_start_time is not None):
            return "present"
        else:
            return "absent"
    
    def update(self, distance, motion):
        # Main logic to update stillness status based on distance and motion readings
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
                    if time.time() - self.last_motion_time > self.movement_dampening_interval:
                        self.last_motion_time = time.time()
                        print(self.last_motion_time)
                        print("Ignoring brief motion")
                    elif time.time() - self.last_motion_time < PIR_RESET_TIME:
                        print("PIR reset time not yet elapsed, ignoring motion")
                    else:
                        print("Person is in motion")
                        self.reset()
            else:
                if motion == 1:
                    self.last_motion_time = time.time()
                    print("Motion detected, resetting stillness timer")
                    self.reset()

    def reset(self):
        # Reset all timers and flags
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
        # Set the required stillness duration before detection is triggered
        self.delay = delay
        print("Delay set to:", delay)

    def get_distance(self):
        # Get current distance reading from time-of-flight sensor
        return self.max_distance
    
    def get_motion(self):
        # Get current motion state from PIR sensor
        return motion_sensor.value()

def read_distance():
    # Trigger measurement and read distance
    try:
        distance = time_of_flight.read()
        if distance == 0: # impossible reading
            print("Error: Invalid distance reading (0 mm)")
            sys.exit(1)
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

# Initialize the stillness detector with configured parameters
detector = StillnessDetector(max_distance=MAX_DISTANCE, min_distance=MIN_DISTANCE, delay=SETTLING_DELAY, damping_interval=DAMPING_INTERVAL)

# Main loop to read distance every second
def main():
    while True:
        # Provide visual feedback on status LED based on recent motion activity
        if time.time() - detector.last_motion_time < DAMPING_INTERVAL:
            blink_led(STATUS_LED, 2, interval=0.1)
        else:
            blink_led(STATUS_LED, 1, interval=0.2)
        # Read distance and motion sensor values, update stillness detector
        distance = read_distance()
        motion_state =  motion_sensor.value()
        print("Distance (mm):", distance, "PIR State:", motion_state)
        detector.update(distance, motion_state)
        # Set LED and output pins based on stillness status
        if detector.get_status() == "still":
            set_led_color(0, 1, 0)  # Green for stillness detected
            STILL_OUTPUT.value(1)
            PRESENT_OUTPUT.value(0)
            ABSENT_OUTPUT.value(0)
        elif detector.get_status() == "present":
            set_led_color(0, 0, 1)  # Blue for presence detected
            STILL_OUTPUT.value(0)
            PRESENT_OUTPUT.value(1)
            ABSENT_OUTPUT.value(0)
        else:
            set_led_color(1, 0, 0)  # Red for no presence detected
            STILL_OUTPUT.value(0)
            PRESENT_OUTPUT.value(0)
            ABSENT_OUTPUT.value(1)
        time.sleep(0.5) # Short delay to prevent excessive sensor reads and allow for visual feedback on status LED

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        # Gracefully handle Ctrl+C to exit the program
        print("Program interrupted by user")
        STATUS_LED.value(0)
        RED.value(0)
        GREEN.value(0)
        BLUE.value(0)
        sys.exit(0)