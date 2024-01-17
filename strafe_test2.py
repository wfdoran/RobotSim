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
l_total = 0.0
r_total = 0.0
l_prev = 0.0
r_prev = 0.0
delta_time = 0.01

kp = 0.1
ki = 1.0
kd = 0.0

for i in range(300):
    s_curr, l_curr, r_curr = r.read_odometry()

    if s_curr - s_base > target_in - allowed_err_in:
        break

    l_error = l_curr - l_base
    r_error = r_curr - r_base
    l_der = (l_error - l_prev) / delta_time
    r_der = (r_error - r_prev) / delta_time
    l_total += delta_time * l_error
    r_total += delta_time * r_error

    l_prev = l_error
    r_prev = r_error

    l_corr = kp * l_error + ki * l_total + kd * l_der
    r_corr = kp * r_error + ki * r_total + kd * l_der
    
    speed = 0.5
    front_left  =  speed - l_corr 
    front_right = -speed - r_corr
    back_left   = -speed - l_corr
    back_right  =  speed - r_corr

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

    r.step(t_sec = delta_time, noise = 0.2)
    print(i, r)
    r.plot()

#plt.figure(figsize=(50, 2))    
plt.show()

    
           
