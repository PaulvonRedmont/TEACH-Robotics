import board
import time
import pwmio
import digitalio
from adafruit_motor import servo
from adafruit_simplemath import map_range
from circuitpython_gizmo import Gizmo

gizmo = Gizmo()

motor_left = servo.ContinuousServo(pwmio.PWMOut(gizmo.MOTOR_1, frequency=50))
motor_right = servo.ContinuousServo(pwmio.PWMOut(gizmo.MOTOR_2, frequency=50))

builtin_led = digitalio.DigitalInOut(board.GP25)
builtin_led.direction = digitalio.Direction.OUTPUT

# Keep running forever
while True:
    #Toggle the built-in LED each time through the loop so we can see that the program really is running.
    builtin_led.value = not builtin_led.value
    # Refreshes the information about axis and button states
    gizmo.refresh()
    # Convert gamepad axis positions (0 - 255) to motor speeds (-1.0 - 1.0)
    throttle_left = map_range(gizmo.axes.left_y, 0, 255, -1.0, 1.0)
    throttle_right = map_range(gizmo.axes.right_y, 0, 255, -1.0, 1.0)
    # Send motor speeds to motors
    motor_left.throttle = throttle_left
    motor_right.throttle = throttle_right
    time.sleep(0.02)
