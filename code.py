"""
This is the main controller program for our robot developed by TEACH Robotics 2024
All of this code was written by Paul Estrada (Programming)

This code has three control modes: 'Tank Mode', 'Arcade Mode', and 'Pawl Mode'. The Start
button on the gamepad switches the robot between the first two modes, and the Back 
button switches the robot to 'Pawl Mode'. The robot will start in 'Tank Mode'.



Here are the controls for Tank Mode:
Left Joystick Up/Down    - Motor 1 Fwd/Rev
Right Joystick Up/Down   - Motor 2 Fwd/Rev

Here are the controls for Arcade Mode:
Left Joystick Up/Down    - Robot Fwd/Rev
Left Joystick Left/Right - Robot Turn Left/Right

Here are the controls for Pawl Mode:
Right Joystick Up/Down    - Both motors Fwd/Rev
Left Joystick Left/Right  - Turn Left/Right



These controls work in both modes:
Right Trigger            - Motor 3 Forward
Right Shoulder Button    - Motor 4 Reverse
Left Trigger             - Servo 1 to 0 degrees
Left Shoulder Button     - Servo 1 to 90 degrees

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
# Left Motor
motor_left = servo.ContinuousServo(
    pwmio.PWMOut(gizmo.MOTOR_1, frequency=pwm_freq),
    min_pulse=min_pulse,
    max_pulse=max_pulse
)
# Right Motor
motor_right = servo.ContinuousServo(
    pwmio.PWMOut(gizmo.MOTOR_2, frequency=pwm_freq),
    min_pulse=min_pulse,
    max_pulse=max_pulse
)


#Motor arm stuff
#raise arm
motor_task_raise_arm = servo.ContinuousServo(
    pwmio.PWMOut(gizmo.MOTOR_3, frequency=pwm_freq),
    min_pulse=min_pulse,
    max_pulse=max_pulse
)
#extend/retract arm
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
    pwmio.PWMOut(gizmo.SERVO_2, frequency=pwm_freq),
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

#keep running forever and refresh and check the contoller inputs
while True:

    builtin_led.value = not builtin_led.value

    gizmo.refresh()

    if gizmo.buttons.start and not prev_start_button:
        mode = ARCADE_MODE if mode == TANK_MODE else TANK_MODE
    if gizmo.buttons.back and not prev_back_button:
        mode = PAWL_MODE if mode == TANK_MODE else TANK_MODE 
    #pawl mode switch trigger 
    prev_start_button = gizmo.buttons.start

    if mode == TANK_MODE:
        print("Now in tank mode")
        motor_left.throttle = map_range(gizmo.axes.left_y, 0, 255, -1.0, 1.0)
        motor_right.throttle = map_range(gizmo.axes.right_y, 0, 255, -1.0, 1.0)

    elif mode == ARCADE_MODE:
        print ("Now in arcade mode")
        speed = map_range(gizmo.axes.left_y, 0, 255, -1.0, 1.0)
        steering = map_range(gizmo.axes.left_x, 0, 255, -1.0, 1.0)
        motor_left.throttle = constrain(speed - steering, -1.0, 1.0)
        motor_right.throttle = constrain(speed + steering, -1.0, 1.0)
    
    elif mode == PAWL_MODE:
        print("Now in pawl mode")
        #left joystick controls both motors to go forward/backwards
        speed = map_range(gizmo.axes.left_y, 0, 255, -1.0, 1.0)
        #right joystick horizontal axis controls the turning (basically slows down one motor to half speed while continuing the other motor at full speed)
        turn = map_range(gizmo.axes.right_x, 0, 255, -1.0, 1.0)

        # Calculate the throttle for each motor
        if turn > 0:  # Turning right
            motor_left.throttle = speed
            motor_right.throttle = speed * (1 - turn / 2)  # Slow down the right motor
        elif turn < 0:  # Turning left
            motor_left.throttle = speed * (1 + turn / 2)  # Slow down the left motor
            motor_right.throttle = speed
        else:  # No turning
            motor_left.throttle = speed
            motor_right.throttle = speed
        #A wee little side note here: I'm unable to find the technical term for this sort of controller layout, so I'm just going to call it "Pawl Mode" for now.
        #Was initially thinking of calling it "Better/Modern Arcade" because when I first heard of the term "Arcade Mode" 
        #I thought it was going to be like this (like regular video games), but it wasn't :(
        #But this controller layout is basically the same as popular games such a Fortnite, Minecraft, and Roblox, so it's a pretty modern controller layout and the standard in at least the shooter game industry


    #Raise and lower arm motor
    if gizmo.buttons.left_trigger:
        print("Left trigger pressed")
        motor_task_raise_arm.motor.throttle = 1.0
    elif gizmo.buttons.left_shoulder:
        print("Left shoulder pressed")
        motor_task_raise_arm.motor.throttle = -1.0
    #arm extension and retration motor
    if gizmo.buttons.y:
        motor_task_arm_extension_retraction.motor.throttle = 1.0
    elif gizmo.buttons.y:
        motor_task_arm_extension_retraction.motor.throttle = -1.0

    #SERVOS CODE
    #Claw open and close servo
    if gizmo.buttons.y:
        servo_task_claw_open_and_close.angle = 90
    elif gizmo.buttons.y:
        servo_task_claw_open_and_close.angle = 0
    #Habitat modules dropper servo
    if gizmo.buttons.right_trigger:
        servo_task_habitat_modules_dropper.angle = 90
    elif gizmo.buttons.right_shoulder:
        servo_task_habitat_modules_dropper.angle = 0

    time.sleep(0.001)  #tiny delay to make sure we dont crash the gizmo board and overheat it and leave another set of fingerprints on the melted plastic case
