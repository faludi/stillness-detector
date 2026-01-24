# Requires time of flight sensor driver: https://github.com/drakxtwo/vl53l1x_pico

from machine import Pin, I2C
from vl53l1x import VL53L1X
import time
import sys

time.sleep(2) # allow usb connection on startup

# TODO: Add logging functionality to record stillness events with timestamps
# TODO: Add power-saving features for battery operation

version = "1.0.4"
print("Stillness Detector - Version:", version)

PIR_RESET_TIME = 1.5  # seconds to wait after motion detected
DISTANCE_THRESHOLD = 1300
SETTLING_DELAY = 3
DAMPING_INTERVAL = 10

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
    def __init__(self, distance_threshold=1300, delay=2, damping_interval=10):
        self.distance_threshold = distance_threshold
        self.delay = delay
        self.presence_start_time = None
        self.elapsed_time = None
        self.stillness_detected = False
        self.movement_dampening_interval = damping_interval
        self.last_motion_time = time.time()

    def is_person_detected(self, distance):
        return distance is not None and distance < self.distance_threshold
    
    def get_status(self):
        if self.stillness_detected:
            return "still"
        elif (self.presence_start_time is not None):
            return "present"
        else:
            return "absent"
    
    def update(self, distance, motion):
        if self.is_person_detected(distance):
            if self.presence_start_time is None:
                self.presence_start_time = time.time()
                self.last_motion_time = time.time()
                print("Min distance threshold crossed, starting timer")
            else:
                self.elapsed_time = time.time() - self.presence_start_time
        else:
            self.presence_start_time = None
            self.elapsed_time = None
            self.stillness_detected = False
            print("Presence ended")

        if self.elapsed_time is not None and self.elapsed_time > self.delay:
            if motion == 0:
                self.stillness_detected = True
                print("Person has been still for", self.elapsed_time, "seconds")
            else:
                if time.time() - self.last_motion_time > self.movement_dampening_interval:
                    print("Ignoring brief motion")
                    time.sleep(PIR_RESET_TIME) # brief pause to avoid immediate re-trigger
                    self.last_motion_time = time.time()
                else:
                    print("Person is in motion")
                    self.presence_start_time = None
                    self.elapsed_time = None
                    self.stillness_detected = False
    
    def set_distance_threshold(self, threshold):
        self.distance_threshold = threshold
        print("Distance threshold set to:", threshold)

    def set_delay(self, delay):
        self.delay = delay
        print("Delay set to:", delay)

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

detector = StillnessDetector(distance_threshold=DISTANCE_THRESHOLD, delay=SETTLING_DELAY, damping_interval=DAMPING_INTERVAL)

# Main loop to read distance every second
def main():
    while True:
        blink_led(STATUS_LED, 1, interval=0.1)
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