MODE = 'normal' # options: "normal", "relaxed", "strict", "custom"
PIR_RESET_TIME = 1.5  # duration of a PIR trigger, from sensor's data sheet or empirically
MAX_DISTANCE = 2000  # farthest a person can be detected
MIN_DISTANCE = 300  # nearest a person can be detected

# Only used in "custom" mode!
SETTLING_DELAY = 5
DAMPING_INTERVAL = 7
ALLOWABLE_DISTANCE_CHANGE = 300