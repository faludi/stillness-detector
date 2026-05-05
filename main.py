# Requires time of flight sensor driver: https://github.com/drakxtwo/vl53l1x_pico

from machine import Pin, I2C, PWM
from vl53l1x import VL53L1X
import time
import sys
import settings

time.sleep(2) # allow usb connection on startup

# TODO: Add optional power-saving features for battery operation

version = "1.0.15"
print("Stillness Detector - Version:", version)

# Pin Assignments
PIR_PIN = 3
STATUS_LED_PIN = "LED"
RED_LED_PIN = 18
GREEN_LED_PIN = 19
YELLOW_LED_PIN = 20
ABSENT_OUTPUT_PIN = 7
PRESENT_OUTPUT_PIN = 8
STILL_OUTPUT_PIN = 9

# LED States (for clarity)
LED_ON = 0  # Active low
LED_OFF = 1

PIR_RESET_TIME = settings.PIR_RESET_TIME
MAX_DISTANCE = settings.MAX_DISTANCE
MIN_DISTANCE = settings.MIN_DISTANCE

MODE = settings.MODE

MODE_PRESETS = {
    "relaxed": {"settling_delay": 3, "damping_interval": 3, "allowable_distance_change": 1000},
    "normal": {"settling_delay": 5, "damping_interval": 7, "allowable_distance_change": 300},
    "strict": {"settling_delay": 10, "damping_interval": 20, "allowable_distance_change": 200},
}

if MODE in MODE_PRESETS:
    config = MODE_PRESETS[MODE]
    SETTLING_DELAY = config["settling_delay"]
    DAMPING_INTERVAL = config["damping_interval"]
    ALLOWABLE_DISTANCE_CHANGE = config["allowable_distance_change"]
elif MODE == "custom":
    SETTLING_DELAY = settings.SETTLING_DELAY
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

class HardwareInterface:
    """
    Manages all hardware components for the stillness detector.
    
    This class encapsulates the setup and control of:
    - Motion sensor (PIR)
    - Status LED (onboard)
    - RGB LED indicator
    - Digital output pins for external devices
    
    All LEDs are configured as active-low (0=on, 1=off).
    """
    def __init__(self):
        self.motion_sensor = Pin(PIR_PIN, Pin.IN, Pin.PULL_DOWN)
        self.status_led = Pin(STATUS_LED_PIN, Pin.OUT)
        self.rgb_leds = {
            "red": Pin(RED_LED_PIN, Pin.OUT),
            "green": Pin(GREEN_LED_PIN, Pin.OUT),
            "yellow": Pin(YELLOW_LED_PIN, Pin.OUT),
        }
        self.outputs = {
            "absent": Pin(ABSENT_OUTPUT_PIN, Pin.OUT),
            "present": Pin(PRESENT_OUTPUT_PIN, Pin.OUT),
            "still": Pin(STILL_OUTPUT_PIN, Pin.OUT),
        }
        self._initialize_all()
    
    def _initialize_all(self):
        self.status_led.value(LED_OFF)
        for led in self.rgb_leds.values():
            led.value(LED_OFF)  # LEDs are active-low, so 1=off
        for output in self.outputs.values():
            output.value(0)
    
    def set_rgy_led(self, red, green, yellow):
        self.rgb_leds["red"].value(red)
        self.rgb_leds["green"].value(green)
        self.rgb_leds["yellow"].value(yellow)
    
    def get_motion(self):
        return self.motion_sensor.value()

# Initialize hardware
hardware = HardwareInterface()

class StillnessDetector:
    """
    Detects if a person is present and still using distance and motion sensors.
    
    States:
    - "absent": No person detected (distance out of range)
    - "present": Person detected but moving
    - "still": Person detected and still for the settling delay duration
    """
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
        if not self.is_person_detected(distance):
            self.reset()
            return
        
        # Handle initial detection
        if self.presence_start_time is None:
            self._handle_initial_detection(distance)
            return
        
        # Check for significant movement
        self._check_distance_change(distance)
        
        # Update elapsed time and check stillness
        self.elapsed_time = time.time() - self.presence_start_time
        self._update_stillness_status(motion)
    
    def _handle_initial_detection(self, distance):
        self.presence_start_time = time.time()
        self.last_motion_time = time.time()
        self.start_distance = distance
        print("Distance threshold crossed, starting timer")
    
    def _check_distance_change(self, distance):
        if abs(distance - self.start_distance) > ALLOWABLE_DISTANCE_CHANGE:
            print("Significant distance change detected, resetting timer")
            self.reset()
    
    def _update_stillness_status(self, motion):
        # Clearer logic for motion/stillness determination
        if self.elapsed_time <= self.delay:
            print("Waiting for settling delay...")
        elif motion == 0:  # No motion
            self.stillness_detected = True
            print(f"Person has been still for {self.elapsed_time:.1f} seconds")
        else:  # Motion detected
            self._handle_motion_detected()
    
    def _handle_motion_detected(self):
        if time.time() - self.last_motion_time > self.movement_dampening_interval:
            self.last_motion_time = time.time()
            print("Ignoring brief motion")
        elif time.time() - self.last_motion_time < PIR_RESET_TIME:
            print("PIR reset time not yet elapsed, ignoring motion")
        else:
            print("Person is in motion")
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
        return hardware.get_motion()

def read_distance():
    try:
        distance = time_of_flight.read()
        if distance == 0:
            print("Error: Invalid distance reading (0 mm)")
            return None
        return distance
    except Exception as e:
        print(f"Error reading distance: {e}")
        return None
    
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
            blink_led(hardware.status_led, 2, interval=0.1)
        else:
            blink_led(hardware.status_led, 1, interval=0.2)
        # Read distance and motion sensor values, update stillness detector
        distance = read_distance()
        motion_state = hardware.get_motion()
        print("Distance (mm):", distance, "PIR State:", motion_state)
        detector.update(distance, motion_state)
        # Set LED and output pins based on stillness status
        if detector.get_status() == "still":
            hardware.set_rgy_led(0, 1, 0)  # Green for stillness detected
            hardware.outputs["still"].value(1)
            hardware.outputs["present"].value(0)
            hardware.outputs["absent"].value(0)
        elif detector.get_status() == "present":
            hardware.set_rgy_led(0, 0, 1)  # Yellow for presence detected
            hardware.outputs["still"].value(0)
            hardware.outputs["present"].value(1)
            hardware.outputs["absent"].value(0)
        else:
            hardware.set_rgy_led(1, 0, 0)  # Red for no presence detected
            hardware.outputs["still"].value(0)
            hardware.outputs["present"].value(0)
            hardware.outputs["absent"].value(1)
        time.sleep(0.5) # Short delay to prevent excessive sensor reads and allow for visual feedback on status LED

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        # Gracefully handle Ctrl+C to exit the program
        print("Program interrupted by user")
        hardware.status_led.value(0)
        for led in hardware.rgb_leds.values():
            led.value(1)
        sys.exit(0)