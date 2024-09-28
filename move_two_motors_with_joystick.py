import board
import time
import pwmio
import digitalio
from adafruit_motor import servo
from adafruit_simplemath import map_range
from circuitpython_gizmo import Gizmo

# the Gizmo object provides access to the data that is held by the field
# management system and the gizmo system processor
gizmo = Gizmo()

# Configure the motors for the ports they are connected to
motor_left = servo.ContinuousServo(pwmio.PWMOut(gizmo.MOTOR_1, frequency=50))
motor_right = servo.ContinuousServo(pwmio.PWMOut(gizmo.MOTOR_2, frequency=50))

# Configure the built-in LED pin as an output
builtin_led = digitalio.DigitalInOut(board.GP25)
builtin_led.direction = digitalio.Direction.OUTPUT

# Keep running forever
while True:
    # Toggle the built-in LED each time through the loop so we can see
    # that the program really is running.
    builtin_led.value = not builtin_led.value

    # Refreshes the information about axis and button states
    gizmo.refresh()

    # Convert gamepad axis positions (0 - 255) to motor speeds (-1.0 - 1.0)
    throttle_left = map_range(gizmo.axes.left_y, 0, 255, -1.0, 1.0)
    throttle_right = map_range(gizmo.axes.right_y, 0, 255, -1.0, 1.0)

    # Send motor speeds to motors
    motor_left.throttle = throttle_left
    motor_right.throttle = throttle_right

    # Sleep for 20ms, which means this loop will run at ~50Hz.  This is
    # because in this sample code all we do is get control inputs and
    # map them to motors to drive around, so we don't need to go any
    # faster.  You should probably remove this in your code so that
    # your main loop runs faster when polling sensors or animating
    # lights.
    time.sleep(0.02)
