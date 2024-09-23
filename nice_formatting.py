import time, sys
import circuitpython_gizmo

# Paul Estrada, 9/23/24, 16:52:04 PM, CST

header = "|   A   |   B   |    X   |    Y   |   Axes Left X,Y   |   "
values = f"{}"
gizmo = circuitpython_gizmo.Gizmo()

print(header)
while True:
    for i in range(10):
        values[i] += 1
        print("\r" + " ".join(map(str, values)), end="")
        time.sleep(0.001)

while True:
    gizmo.refresh()
    #print(gizmo.buttons.a)
    print(gizmo.axes.left_x, gizmo.axes.left_y)
    #print(gizmo.axes.left_y)
    time.sleep(0.001)
