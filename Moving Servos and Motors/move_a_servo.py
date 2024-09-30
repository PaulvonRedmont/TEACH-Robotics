import time
import circuitpython_gizmo
import pwmio
import adafruit_motor.servo

gizmo = circuitpython_gizmo.Gizmo()

# Initialize servos
servo1 = adafruit_motor.servo.Servo(
    pwmio.PWMOut(gizmo.SERVO_1, frequency=50),
    actuation_range=90,
    min_pulse=1000,
    max_pulse=2000
)

servo4 = adafruit_motor.servo.Servo(
    pwmio.PWMOut(gizmo.SERVO_4, frequency=50),
    actuation_range=90,
    min_pulse=1000,
    max_pulse=2000
)

while True:
    gizmo.refresh()
    if gizmo.buttons.a:
        print("Button A pressed")
        servo1.angle = 90
        servo4.angle = 90
    else:
        print("Waiting for driver input....")
        servo1.angle = 0
        servo4.angle = 0
    time.sleep(0.01)
