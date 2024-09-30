import time
import circuitpython_gizmo
import pwmio
import adafruit_motor

gizmo = circuitpython_gizmo.Gizmo()

motor = adafruit_motor.servo.ContinuousServo(
    pwmio.PWMOut(gizmo.MOTOR_1, frequency=50),
    min_pulse=1000,
    max_pulse=2000
)

while True:
    gizmo.refresh()
    if gizmo.buttons.a:
        motor.throttle = 1.0
    else:
        motor.throttle = 0.0
    time.sleep(0.01)
