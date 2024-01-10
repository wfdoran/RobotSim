import matplotlib.pyplot as plt
import math
import random
import robot
import pose


random.seed(1)
r = robot.Robot()
r.set_pose(pose.Pose(0.0, 0.0, math.pi / 2.0))
r.plot()


s_base, l_base, r_base = r.read_odometry()

target_in = 48.0
allowed_err_in = 0.5
total_error = 0.0
delta_time = 0.01

kp = 0.1
ki = 1000.0

for i in range(300):
    s_curr, l_curr, r_curr = r.read_odometry()

    if s_curr - s_base > target_in - allowed_err_in:
        break

    l_err = l_curr - l_base
    r_err = r_curr - r_base
    error = l_err - r_err
    total_error += delta_time * error

    correction = kp * error + ki * total_error

    speed = 0.5
    front_left  =  speed - correction
    front_right = -speed + correction
    back_left   = -speed - correction
    back_right  =  speed + correction

    norm = max(abs(front_left), abs(front_right), abs(back_left), abs(back_right))
    if norm > 1.0:
        front_left /= norm
        front_right /= norm
        back_left /= norm
        back_right /= norm
    
    r.set_power(robot.FRONT_LEFT,  front_left)
    r.set_power(robot.FRONT_RIGHT, front_right)
    r.set_power(robot.BACK_LEFT, back_left)
    r.set_power(robot.BACK_RIGHT, back_right)

    r.step(t_sec = delta_time, noise = 0.3)
    print(i, r, "%8.4f %8.4f %8.4f" % (error, r.pose.theta_rads - math.pi / 2.0, total_error))
    r.plot()

#plt.figure(figsize=(50, 2))    
plt.show()

    
           
