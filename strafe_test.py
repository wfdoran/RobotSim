import matplotlib.pyplot as plt
import math
import robot
import pose



r = robot.Robot()
r.set_pose(pose.Pose(0.0, 0.0, math.pi / 2.0))
r.plot()


s_base, r_base, l_base = r.read_odometry()

target_in = 48.0
allowed_err_in = 0.5
l_total = 0.0
r_total = 0.0
delta_time = 0.01

kp = 0.1
ki = 0.0

for _ in range(300):
    s_curr, r_curr, l_curr = r.read_odometry()

    if s_curr - s_base > target_in - allowed_err_in:
        break

    power = 0.5

    l_err = l_curr - l_base
    r_err = r_curr - r_base

    l_total += delta_time * l_err
    r_total += delta_time * r_err

    l_correction = kp * l_err + ki * l_total
    r_correction = kp * r_err + ki * r_total
    
    r.set_power(robot.FRONT_LEFT,  power - l_correction)
    r.set_power(robot.FRONT_RIGHT, -power - r_correction)
    r.set_power(robot.BACK_LEFT, -power - l_correction)
    r.set_power(robot.BACK_RIGHT, power - r_correction)

    r.step(t_sec = delta_time, noise = 0.1)
    print(r)
    r.plot()

#plt.figure(figsize=(50, 2))    
plt.show()

    
           
