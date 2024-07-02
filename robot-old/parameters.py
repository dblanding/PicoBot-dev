# parameters.py

# car geometry
TRACK_WIDTH = 0.1817  # meters
WHEEL_CIRC = 0.214  # meters

# encoder / gearbox
TICKS_PER_REV = 2464
METERS_PER_TICK = WHEEL_CIRC / TICKS_PER_REV
TICKS_PER_METER = TICKS_PER_REV / WHEEL_CIRC

# nominal motor speed for driving straight (ticks per sec)
TARGET_TICK_RATE = 3000

# max motor speed for driving sraight ahead or back (PWM value)
FULL_SPD = 60_000

# max motor speed while turning in place (PWM value)
TURN_SPD = 30_000

# half width of "good enough" zone when turning to angle
ANGLE_TOL = 0.035  # radians (2 degrees)

# ratio of spd to joystick value
SPD_GAIN = 2
