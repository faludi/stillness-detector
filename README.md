The Stillness Detector senses presence and motion to determine if there is a person standing in front of it and whether that person is currently standing still. It is intended to provide a trigger to interactive art pieces that require stillness rather than action from the viewer. 

### Stillness Detector breadboard diagram<br>
![Stillness Detector breadboard diagram](images/Stillness_Detector_bb.png)

### Stillness Detector schematic<br>
![Stillness Detector schematic](images/Stillness_Detector_schem.png)



### Stillness Detector BOM

| Quantity | Manufacturer | Part | Description |
| ----------- | ----------- | ----------- | ----------- |
| 1 | Raspberry Pi | Pico 2 | Microcontroller, RP2350 |
| 2 | Adafruit | 4090 | USB Type C Breakout Board, Downstream Connection |
| 1 | Sparkfun | COM-09264 | LED - RGB Diffused Common Cathode |
| 1 | Littelfuse | HE3351A0500 | Reed Relay, SPST-NO, 5 VDC |
| 1 | Adafruit | 3967 | Adafruit VL53L1X Time of Flight Distance Sensor |
| 1 | Adafruit | 189 | PIR Motion Sensor |
| 3 | Adafruit | 2780 | Through-Hole Resistors - 220 ohm |

### Settings

The `settings.py` file contains the user settings values for the stillness detector.

| Name | Description |
| ----------- | ----------- |
| MODE | Defines how forgiving detector will be using: "normal", "relaxed", "strict", or "custom" |
| PIR_RESET_TIME | duration of a Passive Infrared Trigger, from sensor's data sheet or empirically |
| MAX_DISTANCE  farthest a person can be detected in millimeters |
| MIN_DISTANCE | nearest a person can be detected in millimeters |
| SETTLING_DELAY | delay in seconds after a person is sensed before stillness mode is activated |
| DAMPING_INTERVAL | ignore individual movement events if they are at least this far apart, in seconds |
| ALLOWABLE_DISTANCE_CHANGE | maximum total change forward or back from initial detection, before stillness is cancelled, in millimeters |

### Pinouts

The kinetic artwork's connections will be to the first three OUTPUT pins. The simplest connection would be to only the STILL_OUTPUT, to trigger actions once the person is motionless. If the artwork has a behavior when someone is present but in motion, that would be triggered by the ABSENT_OUTPUT. Finally, for behaviors that only happen when nobody is around, use the ABSENT_OUTPUT for a trigger.

| Pin | Name | Description |
| ----------- | ----------- | ----------- |
| GP7 | ABSENT_OUTPUT | digital output, high if no presence is currently detected |
| GP8 | PRESENT_OUTPUT | digital output, high if presence but not yet stillness is detected |
| GP9 | STILL_OUTPUT | digital output, high if stillness is detected |
| GP3 | motion_sensor | digital input from PIR sensor |
| GP4 | time_of_flight SDA | I2C data for time of flight sensor |
| GP5 | time_of_flight SCL | I2C clock for time of flight sensor |
| GP18 | Red LED | absent indicator, for debugging |
| GP19 | Green LED | stillness indicator, for debugging |
| GP20 | Blue LED | presence indicator, for debugging |


*\* images created with Fritzing.*