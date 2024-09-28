"""
This is the default BEST Robotics program for the Gizmo.
This program offers remote control of simple robots using 3 motors and a servo.
This may serve as a useful starting point for your team's competition code. You
will almost certainly need to edit or extend this code to meet your needs.

This code has two control modes: 'Tank Mode' and 'Arcade Mode'. The Start
button on your gamepad switches the robot between the two modes.

Here are the controls for Tank Mode:
Left Joystick Up/Down    - Motor 1 Fwd/Rev
Right Joystick Up/Down   - Motor 3 Fwd/Rev

Here are the controls for Arcade Mode:
Left Joystick Up/Down    - Robot Fwd/Rev
Left Joystick Left/Right - Robot Turn Left/Right

These controls work in both modes:
Right Trigger            - Motor 4 Forward
Right Shoulder Button    - Motor 4 Reverse
Left Trigger             - Servo 1 to 0 degrees
Left Shoulder Button     - Servo 1 to 90 degrees

When neither the left trigger nor shoulder button are pressed, the servo will
go to 45 degrees.
"""

import board
import time
import pwmio
import digitalio    
from adafruit_motor import servo
from adafruit_simplemath import map_range, constrain
from circuitpython_gizmo import Gizmo

gizmo = Gizmo()

pwm_freq = 50 # Hertz
min_pulse = 1000 # milliseconds
max_pulse = 2000 # milliseconds
servo_range = 180  # degrees

# Configure the motors & servos for the ports they are connected to

motor_left = servo.ContinuousServo(
# Left Motor
    pwmio.PWMOut(gizmo.MOTOR_1, frequency=pwm_freq),
    min_pulse=min_pulse,
    max_pulse=max_pulse
)
motor_right = servo.ContinuousServo(
# Right Motor
    pwmio.PWMOut(gizmo.MOTOR_3, frequency=pwm_freq),
    min_pulse=min_pulse,
    max_pulse=max_pulse
)


#Motor arm stuff
#raise arm, extend it
motor_task_raise_arm = servo.ContinuousServo(
    pwmio.PWMOut(gizmo.MOTOR_4, frequency=pwm_freq),
    min_pulse=min_pulse,
    max_pulse=max_pulse
)
motor_task_arm_extension_retraction = servo.ContinuousServo(
    pwmio.PWMOut(gizmo.MOTOR_4, frequency=pwm_freq),
    min_pulse=min_pulse,
    max_pulse=max_pulse
)

#Servos, which has:
#Habitat Modules thingy
#Claw open and close
#Servo on the back of the robot for dropping habitat modules thingy
servo_task_habitat_modules_dropper = servo.Servo(
    pwmio.PWMOut(gizmo.SERVO_1, frequency=pwm_freq),
    actuation_range=servo_range,
    min_pulse=min_pulse,
    max_pulse=max_pulse
)

servo_task_claw_open_and_close = servo.Servo(
    pwmio.PWMOut(gizmo.SERVO_1, frequency=pwm_freq),
    actuation_range=servo_range,
    min_pulse=min_pulse,
    max_pulse=max_pulse
)


# Configure the built-in LED pin as an output
builtin_led = digitalio.DigitalInOut(board.GP25)
builtin_led.direction = digitalio.Direction.OUTPUT


#Modes stuff
TANK_MODE = 0
ARCADE_MODE = 1
PAWL_MODE = 2
#starting mode
mode = TANK_MODE
#start button pressed/not pressed
prev_start_button = False
#back button pressed (perhaps to switch to Pawl mode?)
prev_back_button = False

# Keep running forever
while True:
    loop_start_time = time.monotonic()  # Start timing the loop

    builtin_led.value = not builtin_led.value

    refresh_start_time = time.monotonic()  # Start timing refresh
    gizmo.refresh()
    refresh_elapsed = time.monotonic() - refresh_start_time

    if gizmo.buttons.start and not prev_start_button:
        mode = ARCADE_MODE if mode == TANK_MODE else TANK_MODE
    prev_start_button = gizmo.buttons.start

    control_start_time = time.monotonic()  # Start timing control logic

    if mode == TANK_MODE:
        motor_left.throttle = map_range(gizmo.axes.left_y, 0, 255, -1.0, 1.0)
        motor_right.throttle = map_range(gizmo.axes.right_y, 0, 255, -1.0, 1.0)
    elif mode == ARCADE_MODE:
        speed = map_range(gizmo.axes.left_y, 0, 255, -1.0, 1.0)
        steering = map_range(gizmo.axes.left_x, 0, 255, -1.0, 1.0)
        motor_left.throttle = constrain(speed - steering, -1.0, 1.0)
        motor_right.throttle = constrain(speed + steering, -1.0, 1.0)

    if gizmo.buttons.right_trigger:
        motor_task.throttle = 1.0
    elif gizmo.buttons.right_shoulder:
        motor_task.throttle = -1.0
    else:
        motor_task.throttle = 0.0

    if gizmo.buttons.left_trigger:
        servo_task_habitat_modules_dropper.angle = 0
    elif gizmo.buttons.left_shoulder:
        servo_task_habitat_modules_dropper.angle = 90
    else:
        servo_task_habitat_modules_dropper.angle = 45

    control_elapsed = time.monotonic() - control_start_time
    loop_elapsed = time.monotonic() - loop_start_time

    print(f"Refresh Time: {refresh_elapsed:.6f}s, Control Time: {control_elapsed:.6f}s, Total Loop Time: {loop_elapsed:.6f}s")
