import matplotlib.pyplot as plt
import math
import robot
import pose


plt.xlim(5,35)
plt.ylim(0, 30)

r = robot.Robot()

r.set_pose(pose.Pose(20.0, 20.0, math.pi / 2.0))
r.plot()

r.set_power(robot.FRONT_LEFT,  1.0)
r.set_power(robot.FRONT_RIGHT, -1.0)
r.set_power(robot.BACK_LEFT, 0.0)
r.set_power(robot.BACK_RIGHT, 0.0)


for _ in range(300):
    r.step(t_sec = 0.01)
    print(r)
    r.plot()

plt.show()

