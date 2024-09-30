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

pwm_freq = 50 # Hertz
min_pulse = 1000 # milliseconds
max_pulse = 2000 # milliseconds
servo_range = 90  # degrees

# Configure the motors & servos for the ports they are connected to
motor_left = servo.ContinuousServo(
    pwmio.PWMOut(gizmo.MOTOR_1, frequency=pwm_freq),
    min_pulse=min_pulse,
    max_pulse=max_pulse
)
motor_right = servo.ContinuousServo(
    pwmio.PWMOut(gizmo.MOTOR_3, frequency=pwm_freq),
    min_pulse=min_pulse,
    max_pulse=max_pulse
)
motor_task = servo.ContinuousServo(
    pwmio.PWMOut(gizmo.MOTOR_4, frequency=pwm_freq),
    min_pulse=min_pulse,
    max_pulse=max_pulse
)
servo_task = servo.Servo(
    pwmio.PWMOut(gizmo.SERVO_1, frequency=pwm_freq),
    actuation_range=servo_range,
    min_pulse=min_pulse,
    max_pulse=max_pulse
)

# Configure the built-in LED pin as an output
builtin_led = digitalio.DigitalInOut(board.GP25)
builtin_led.direction = digitalio.Direction.OUTPUT

# Keep running forever
while True:
    loop_start_time = time.monotonic()  # Start timing the loop

    builtin_led.value = not builtin_led.value

    refresh_start_time = time.monotonic()  # Start timing refresh
    gizmo.refresh()
    refresh_elapsed = time.monotonic() - refresh_start_time

    control_start_time = time.monotonic()  # Start timing control logic

    motor_left.throttle = map_range(gizmo.axes.left_y, 0, 255, -1.0, 1.0)
    motor_right.throttle = map_range(gizmo.axes.right_y, 0, 255, -1.0, 1.0)

    if gizmo.buttons.right_trigger:
        motor_task.throttle = 1.0
    elif gizmo.buttons.right_shoulder:
        motor_task.throttle = -1.0
    else:
        motor_task.throttle = 0.0

    if gizmo.buttons.left_trigger:
        servo_task.angle = 0
    elif gizmo.buttons.left_shoulder:
        servo_task.angle = 90
    else:
        servo_task.angle = 45

    control_elapsed = time.monotonic() - control_start_time
    loop_elapsed = time.monotonic() - loop_start_time

    print(f"Refresh Time: {refresh_elapsed:.6f}s, Control Time: {control_elapsed:.6f}s, Total Loop Time: {loop_elapsed:.6f}s")
