import math
import pose
import robot

r = robot.Robot()
start = pose.Pose(0.0, 0.0, 0.0)
target = pose.Pose(10.0, 0.0, math.pi / 6.0)

print(r.power_for_target(start, target))

for _ in range(30):
    r.set_power_for_target(target)
    r.step(noise = 0.0)
    print(r)
    if r.is_stopped():
        break


