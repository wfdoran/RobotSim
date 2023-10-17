import math
from typing import List
import pose

FRONT_LEFT = 0
FRONT_RIGHT = 1
BACK_LEFT = 2
BACK_RIGHT = 3

class Robot:
    """This class models a classic FTC Mecanum Wheel Robot.  

       https://gm0.org/en/latest/docs/software/tutorials/mecanum-drive.html


       We will assume the wheels are in the X-arrangement at 45
       degrees.
       
         
                  
                 |<----  width_in  ---->|

                 \                      /
                 \                      /  -----
                 \                      /    ^
                                             |

                                         length_in

                                             |
                 /                      \    v
                 /                      \  -----
                 /                      \

    """

    # constant robot parameters.  We assume all four wheels
    # and motors are the same.  
    width_in : float                 # distance between wheels
    length_in : float                # distance between wheels
    wheel_radius_in : float          # radius of the wheels
    max_rpm : float                  # max rpm of the mo

    # the robots current pose.  For now we assume the programmer can
    # see this.  Eventually, this will have be inferred from odometry
    # and April tags.
    pose : pose.Pose

    # wheel motor powers
    __curr_power : List[float]
    __set_power : List[float]

    def __init__(self, width_in = 13.0, length_in = 13.0, wheel_radius_in = 1.9, max_rpm = 312.0):
        """ When creating a new robot, the constant robot parameters must be set.
            Defaults are given.  The robot starts off in the corner, not moving.  """
        self.width_in = width_in
        self.length_in = length_in
        self.wheel_radius_in = wheel_radius_in
        self.max_rpm = max_rpm
        self.__curr_power = [0.0, 0.0, 0.0, 0.0]
        self.__set_power = [0.0, 0.0, 0.0, 0.0]

        self.pose = pose.Pose()

    def __str__(self):
        return "%s [%8.4f, %8.4f, %8.4f, %8.4f]" % \
            (str(self.pose), self.__curr_power[0], self.__curr_power[1], self.__curr_power[2], self.__curr_power[3])

    def set_pose(self, pose : pose.Pose):
        self.pose = pose

    def get_pose(self):
        return self.pose

    def set_power(self, motor : int, power : float):
        """ Sets the power for one of the motors. """
        power = clamp(power, -1.0, 1.0)
        self.__set_power[motor] = power

    def dist_center_to_wheel_in(self):
        a = self.width_in * self.width_in
        a += self.length_in * self.length_in
        return math.sqrt(a) / 2.0

    def eff_dist_center_to_wheel_in(self):
        """When the four Mecanum wheels do not form a square.  There
            is an adjustment needed since the rotational force is at a
            45 degree angle.  See

            https://onlinelibrary.wiley.com/doi/pdfdirect/10.1002/zamm.201900173

            for details.

            Note: when width == length, this returns the same value as
              dist_center_to_wheel

        """
        return (self.width_in + self.length_in) / (2.0 * math.sqrt(2.0))
        

    def step(self, t_sec = 0.1):
        """This is a first shot (very naive) modeling of the movement
            of the robot given powers of the 4 motors.  First we
            convert the power on each wheel to distance travelled for
            that wheel.  Then those are converted into distance
            forward, distance strafed, and rotation for the overall
            robot.  Here the model is overly simple, but we are going
            with it for now.  Finally, the pose of the robot is
            updated given these three values.

            In Zeidis and Zimmermann's paper referenced above, this
            is the "approximate" method, equation 11 on page 4.

            This model does not take into account distribution of
            weight on the wheels, rolling resistances, slipping, ...
        """ 
        # assume for now that the motors are instantly responsive
        for i in range(4):
            self.__curr_power[i] = self.__set_power[i]

        # convert power to inches moved
        dist_in = [power_to_inches(power, self.wheel_radius_in, self.max_rpm, t_sec) for power in self.__curr_power]

        forward_in      = ( dist_in[FRONT_LEFT] + dist_in[FRONT_RIGHT] + dist_in[BACK_LEFT] + dist_in[BACK_RIGHT]) / (4.0 * math.sqrt(2.0)) 
        strafe_right_in = ( dist_in[FRONT_LEFT] - dist_in[FRONT_RIGHT] - dist_in[BACK_LEFT] + dist_in[BACK_RIGHT]) / (4.0 * math.sqrt(2.0))
        rot_dist_counter_clockwise_in = \
                          (-dist_in[FRONT_LEFT] + dist_in[FRONT_RIGHT] - dist_in[BACK_LEFT] + dist_in[BACK_RIGHT]) / 4.0

        rotate_counter_clockwise_rads = rot_dist_counter_clockwise_in / self.eff_dist_center_to_wheel_in()

        # update the pose
        self.pose.apply_movement(forward_in, strafe_right_in, rotate_counter_clockwise_rads)
    

    def set_power_for_target(self, target : pose.Pose, allowed_error_in = 1.0, start_slowdown_in = 10.0, min_power = .2):
        """Now we want to do the opposite of step(): what power settings 
           of the four motors will take us from our current pose to a
           target pose.  The key is the equations for the stepping have 
           a pseudo-inverse.  Writing this in matrix form, the formula
           for wheels turning distance to robot moving distance is

            ( forward )    ( 1/(4 sqrt(2))        0         0  )  (  1  1  1  1 ) ( front_left  )           
            ( strafe  ) =  (      0         1/(4 sqrt(2))   0  )  (  1 -1 -1  1 ) ( front_right )
            ( rotate  )    (      0               0        1/4 )  ( -1  1 -1  1 ) ( back_left   )       (eqn 1)
                                                                                  ( back_right  )

           Notice that 
            
              (  1  1  1  1 )  ( 1  1 -1)      (1 0 0)
              (  1 -1 -1  1 )  ( 1 -1  1)  = 4 (0 1 0) 
              ( -1  1 -1  1 )  ( 1 -1 -1)      (0 0 1)  
                               ( 1  1  1) 

           This the pseudo-inverse. It an inverse in one direction
           only, but only need that direction.  Given how far we want
           move forward, strafe, and rotate, we select powers using.
 
           ( front_left  )    ( 1  1 -1) ( sqrt(2)    0     0 ) ( forward )      
           ( front_right ) =  ( 1 -1  1) (   0     sqrt(2)  0 ) ( strafe  )                             (eqn 2)
           ( back_left   )    ( 1 -1 -1) (   0        0     1 ) ( rotate  )
           ( back_right  )    ( 1  1  1)
            

           When you matrix multiply out (eqn 1) * (eqn 2), everything cancels and you get

           ( forward )   ( 1 0 0 ) ( forward )
           ( strafe  ) = ( 0 1 0 ) ( strafe  )
           ( rotate  )   ( 0 0 1 ) ( rotate  )

           This means if you start with the "desired" (forward, strafe, rotate) and use eqn 2 to get motor
           powers, and then use eqn 1 to determine how far you actually move with those powers.  The actual
           values will equal the desired values.

           Note: This method for getting from A to B works best if you are already roughly facing B.
        """

        # How far do we need to move get to the target.  Also how
        # close are we to the target (the error).  As we get close, we
        # will scale back the speed.  Eventually, we will put a PID
        # controller in here.  For now we just linearly ramp down.
        forward_in, strafe_right_in, rotate_counter_clockwise_rads = self.pose.movement_to(target)
        rot_dist_counter_clockwise_in = rotate_counter_clockwise_rads * self.eff_dist_center_to_wheel_in()
        error_in = abs(forward_in) + abs(strafe_right_in) + 10.0 * abs(rotate_counter_clockwise_rads)

        # If we are within the allowed range of the target, stop.
        if error_in < allowed_error_in:
            self.set_power(FRONT_RIGHT, 0.0)
            self.set_power(FRONT_LEFT, 0.0)
            self.set_power(BACK_RIGHT, 0.0)
            self.set_power(BACK_LEFT, 0.0)
            return 

        # perform the two matrix multiplication to get wheel
        # distances.  This first one just scales forward and right by
        # sqrt(2).
        forward_in *= math.sqrt(2.0)
        strafe_right_in *= math.sqrt(2.0)

        dist_in = [0,0,0,0]
        dist_in[FRONT_LEFT]  = forward_in + strafe_right_in - rot_dist_counter_clockwise_in
        dist_in[FRONT_RIGHT] = forward_in - strafe_right_in + rot_dist_counter_clockwise_in
        dist_in[BACK_LEFT]   = forward_in - strafe_right_in - rot_dist_counter_clockwise_in
        dist_in[BACK_RIGHT]  = forward_in + strafe_right_in + rot_dist_counter_clockwise_in

        # We want to normalize these distances so that the largest
        # power is 1 in absolute value
        norm = max(abs(x) for x in dist_in)

        # if we are within the slowdown range, scale the powers down.
        # again this needs to be a PID controller.
        if error_in < start_slowdown_in:
            scale = min_power + (error_in - allowed_error_in) * (1.0 - min_power) / (start_slowdown_in - allowed_error_in)
            norm /= scale

        power = [x / norm for x in dist_in]

        # set the motors
        self.set_power(FRONT_RIGHT, power[FRONT_RIGHT])
        self.set_power(FRONT_LEFT, power[FRONT_LEFT])
        self.set_power(BACK_RIGHT, power[BACK_RIGHT])
        self.set_power(BACK_LEFT, power[BACK_LEFT])
        

    def is_stopped(self):
        return sum(abs(x) for x in self.__curr_power) < 0.0001
    

        

def power_to_inches(power : float, wheel_radius_in : float, max_rpm : float, t_sec : float):
    """ utility function to convert motor power and time run to inches moved.  """
    min_per_sec = 1.0 / 60.0
    wheel_rots = power * max_rpm * min_per_sec * t_sec
    return wheel_rots * 2.0 * math.pi * wheel_radius_in

def clamp(value, lower_bound, upper_bound):
    """ utility function to clamp values to range. """
    return max(min(value, upper_bound), lower_bound)


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
    
    
    print("===============================")

    r2 = Robot()
    r2.set_pose(pose.Pose(0.0, 0.0, 0.0))
    target = pose.Pose(20.0, 30.0, math.pi / 2.0)

    for _ in range(30):
        r2.set_power_for_target(target)
        r2.step()
        print(r2)
    
    
    print("===============================")
    r3 = Robot()
    r3.set_pose(pose.Pose.random())
    target = pose.Pose.random()

    print(r3.get_pose())
    while True:
        r3.set_power_for_target(target)
        r3.step()
        print(r3)
        if r3.is_stopped():
            break
    print(target)
