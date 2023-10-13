import math
from typing import List
import pose

FRONT_LEFT = 0
FRONT_RIGHT = 1
BACK_LEFT = 2
BACK_RIGHT = 3

class Robot:
    width_in : float                 # distance between wheels
    length_in : float                # distance between wheels
    wheel_radius_in : float
    max_rpm : float

    pose : pose.Pose

    # wheel motor powers
    __curr_power : List[float]
    __set_power : List[float]

    def __init__(self, width_in = 13.0, length_in = 13.0, wheel_radius_in = 1.9, max_rpm = 312.0):
        self.width_in = width_in
        self.length_in = length_in
        self.wheel_radius_in = wheel_radius_in
        self.max_rpm = max_rpm
        self.__curr_power = [0.0, 0.0, 0.0, 0.0]
        self.__set_power = [0.0, 0.0, 0.0, 0.0]

        self.pose = pose.Pose()

    def __str__(self):
        return "%s %s" % (str(self.pose), str(self.__curr_power))

    def set_pose(self, pose : pose.Pose):
        self.pose = pose

    def get_pose(self):
        return self.pose

    def set_power(self, motor : int, power : float):
        if power > 1.0:
            power = 1.0
        if power < -1.0:
            power = -1.0

        self.__set_power[motor] = power

    def dist_center_to_wheel_in(self):
        a = self.width_in * self.width_in
        a += self.length_in * self.length_in
        return math.sqrt(a) / 2.0

    def step(self, t_sec = 0.1):
        # assume for now that the motors are instantly responsive
        for i in range(4):
            self.__curr_power[i] = self.__set_power[i]

        # convert power to inches moved
        dist_in = [power_to_inches(power, self.wheel_radius_in, self.max_rpm, t_sec) for power in self.__curr_power]

        forward_in      = ( dist_in[FRONT_LEFT] + dist_in[FRONT_RIGHT] + dist_in[BACK_LEFT] + dist_in[BACK_RIGHT]) / (4.0 * math.sqrt(2.0)) 
        strafe_right_in = ( dist_in[FRONT_LEFT] - dist_in[FRONT_RIGHT] - dist_in[BACK_LEFT] + dist_in[BACK_RIGHT]) / (4.0 * math.sqrt(2.0))
        rot_dist_counter_clockwise_in = \
                          (-dist_in[FRONT_LEFT] + dist_in[FRONT_RIGHT] - dist_in[BACK_LEFT] + dist_in[BACK_RIGHT]) / 4.0

        rotate_counter_clockwise_rads = rot_dist_counter_clockwise_in / self.dist_center_to_wheel_in()

        self.pose.apply_movement(forward_in, strafe_right_in, rotate_counter_clockwise_rads)
                                                                                            

def power_to_inches(power : float, wheel_radius_in : float, max_rpm : float, t_sec : float):
    min_per_sec = 1.0 / 60.0
    wheel_rots = power * max_rpm * min_per_sec * t_sec
    return wheel_rots * 2.0 * math.pi * wheel_radius_in


if __name__ == "__main__":
    robot = Robot()

    robot.set_pose(pose.Pose(20.0, 20.0, math.pi / 4.0))
    print(robot)
    robot.set_power(FRONT_LEFT, 1.0)
    robot.set_power(FRONT_RIGHT, 1.0)
    robot.set_power(BACK_LEFT, 1.0)
    robot.set_power(BACK_RIGHT, 1.0)
    robot.step()
    print(robot)
    
    
    
