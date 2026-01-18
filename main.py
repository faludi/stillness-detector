from machine import Pin
import time
import qwiic_ultrasonic
import sys

# Create instance for Qwiic Ultrasonic sensor
my_ultrasonic = qwiic_ultrasonic.QwiicUltrasonic()
PIR = Pin(3, Pin.IN)
LED = Pin("LED", Pin.OUT)
LED.value(0)

# Check if sensor is connected
if my_ultrasonic.is_connected() == False:
    print("The device isn't connected to the system. Please check your connection")
    sys.exit(1)

# Initialize ultrasonic sensor
my_ultrasonic.begin()

def read_distance():
    # Trigger measurement and read distance
    try:
        distance = my_ultrasonic.trigger_and_read()
        return distance
    except Exception as e:
        print("Error reading distance:", e)
        return None

# Main loop to read distance every second
while True:
    distance = read_distance()
    print("Distance (mm):", distance, "PIR State:", PIR.value())
    if distance is not None and distance < 300:
        print("Person detected")
        if PIR.value() == 1:
            print("PIR detected motion")
            LED.value(0)
        else:
            print("STILLNESS DETECTED!")
            LED.value(1)
    else:
        print("No person detected")
        LED.value(0)
    time.sleep(0.5)