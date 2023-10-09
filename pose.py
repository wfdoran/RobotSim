import math
import random

class Pose:
    """ Give the position and orientiation of the robot. 
        
      y  ^       /  
         |      /  theta (counter-clockwise <-> increasing)
         |     /
         |    *---------
         |
         +------------->
                      x
    """

    x_in : float
    y_in : float
    theta_rads : float

    def __init__ (self, x_in : float = 0.0, y_in : float = 0.0, theta_rads : float = 0.0):
        self.x_in = x_in
        self.y_in = y_in
        self.theta_rads = theta_rads

    def __str__(self):
        return "(%8.4f %8.4f %8.4f)" % (self.x_in, self.y_in, normalize_angle(self.theta_rads))

    def __eq__(self, other):
        if not isinstance(other, Pose):
            return False
        epsilon_distance_in = 0.1      # allow a .1 inch error
        epsilon_distance_rads = 0.035  # allow a 2 degree error
        
        if math.fabs(self.x_in - other.x_in) > epsilon_distance_in:
            return False
        if math.fabs(self.y_in - other.y_in) > epsilon_distance_in:
            return False
        delta_theta_rads = normalize_angle((self.theta_rads - other.theta_rads) % (2.0 * math.pi))
        if math.fabs(delta_theta_rads) > epsilon_distance_rads:
            return False
        return True

    def random():
        """ return a random Pose on an FTC field. """
        field_size_in = 144.0

        x_in = field_size_in * random.random()
        y_in = field_size_in * random.random()
        theta_rads = normalize_angle(2.0 * math.pi * random.random())

        return Pose(x_in, y_in, theta_rads)

    THETA_SMALL_THRESH = 0.05
    
    def __cos_avg(self, theta0_rads : float, theta1_rads : float):
        """ If the change in theta is small, we can just use the cosine of
            average of the two theta values.  But if it is larger we
            need to do the 'right' thing and integrate

                                        theta1
                              1.0          /
              average = -----------------  |    cos(w) dw
                        theta1 - theta0    /
                                        theta0

                         sin(theta1) - sin(theta0)
                      = ---------------------------
                           theta1 - theta0

             NB: when theta1 - theta0 is small, this is approximately
                the derivative of the sin which is of course the
                cosine!
        """

        if theta1_rads < theta0_rads:
            theta1_rads, theta0_rads = theta0_rads, theta1_rads

        if theta1_rads - theta0_rads > self.THETA_SMALL_THRESH:
            return (math.sin(theta1_rads) - math.sin(theta0_rads)) / (theta1_rads - theta0_rads);
        return math.cos((theta1_rads + theta0_rads) / 2.0)

    def __sin_avg(self, theta0_rads : float, theta1_rads : float):
        if theta1_rads < theta0_rads:
            theta1_rads, theta0_rads = theta0_rads, theta1_rads

        if theta1_rads - theta0_rads > self.THETA_SMALL_THRESH:
            return (-math.cos(theta1_rads) + math.cos(theta0_rads)) / (theta1_rads - theta0_rads);
        return math.sin((theta1_rads + theta0_rads) / 2.0)
        

    def apply_movement(self,
                       forward_in : float = 0.0,
                       strafe_right_in : float = 0.0,
                       rotate_counter_clockwise_rads : float = 0.0):
        """  Movement  
                forward 
                   x -> x + cos(theta) * forward
                   y -> y + sin(theta) * forward
                   theta -> theta

                strafe_right
                   x -> x + sin(theta) * strafe
                   y -> y - cos(theta) * strafe
                   theta -> theta

                rotate:
                   x -> x
                   y -> y
                   theta -> theta + rotate

             When combining all three at once, we assume that rotation
             was smoothly applied throughout and use the average value 
             for sin and cos in these formulas. 

        """
        cos_avg = self.__cos_avg(self.theta_rads, self.theta_rads + rotate_counter_clockwise_rads)
        sin_avg = self.__sin_avg(self.theta_rads, self.theta_rads + rotate_counter_clockwise_rads)
        
        self.x_in += cos_avg * forward_in + sin_avg * strafe_right_in
        self.y_in += sin_avg * forward_in - cos_avg * strafe_right_in
        self.theta_rads += rotate_counter_clockwise_rads

    def movement_to(self, target):
        """Determine the forward, strafe, and rotate needed to move to 
            a target pose.  This is the inverse of apply_movement.  The 
            rotation is easy.

              rotate = target_theta - theta

            For forward and strafe

              x_target = x + forward * cos_avg(rotate) + strafe * sin_avg(rotate)
              y_target = y + forward * sin_avg(rotate) - strafe * cos_avg(rotate)

            In matrix form

              [ x_target - x ]   [ cos_avg(rotate)  sin_avg(rotate) ] [ forward ]
              [              ] = [                                  ] [         ]
              [ y_target - y ]   [ sin_avg(rotate) -cos_avg(rotate) ] [ strafe  ]

            To get the inverse of the rotation matrix, notice 

              [ a  b ] [ a  b ]   [ a*a + b*b      0     ]
              [      ] [      ] = [                      ]
              [ b -a ] [ b -a ]   [     0      a*a + b*b ]

            It is its own inverse upto scale factor.  Note: 
                  
                  sin_avg^2 + cos_avg^2 

            does not necessary equal 1.

               [ cos_avg(rotate)  sin_avg(rotate) ] [ x_target - x ]   [ forward ]
            C  [                                  ] [              ] = [         ]
               [ sin_avg(rotate) -cos_avg(rotate) ] [ y_target - y ]   [ strafe  ]
 
            where C = 1.0 / sin_avg * sin_avg + cos_avg * cos_avg
    
            Here we expect rotate to be big in many cases and it is important 
            to use cos_avg and sin_avg.

        """              

        rotate_counter_clockwise_rads = normalize_angle(target.theta_rads - self.theta_rads)

        cos_avg = self.__cos_avg(self.theta_rads, self.theta_rads + rotate_counter_clockwise_rads)
        sin_avg = self.__sin_avg(self.theta_rads, self.theta_rads + rotate_counter_clockwise_rads)

        C = 1.0 / (cos_avg * cos_avg + sin_avg * sin_avg) 

        delta_x_in = target.x_in - self.x_in
        delta_y_in = target.y_in - self.y_in
        
        forward_in = C * (cos_avg * delta_x_in + sin_avg * delta_y_in)
        strafe_in  = C * (sin_avg * delta_x_in - cos_avg * delta_y_in)

        return (forward_in, strafe_in, rotate_counter_clockwise_rads)
        

def normalize_angle(theta_rads : float):
    """ normalizes the angle theta to be in the range (-pi, pi]. """
    a = theta_rads % (2.0 * math.pi)
    if a > math.pi:
        a -= 2.0 * math.pi
    return a
    
        
if __name__ == "__main__":
    p = Pose()
    print(p)
    p.apply_movement(forward_in = 4.0)
    print(p)
    p.apply_movement(rotate_counter_clockwise_rads = math.pi / 2.0)
    p.apply_movement(forward_in = 3.0)
    print(p)
    p.apply_movement(rotate_counter_clockwise_rads = 2.2143)
    p.apply_movement(forward_in = 5.0)
    print(p)

    print("==============")

    p = Pose()
    print(p)
    p.apply_movement(strafe_right_in = 10.0, rotate_counter_clockwise_rads = math.pi)
    print(p)
    
    print("==============")

    p = Pose(1.0, 1.0, 0.0)
    t = Pose(2.0, 3.0, math.pi / 2.0)

    forward, strafe, rotate = p.movement_to(t)
    print("%8.4f %8.4f %8.4f" % (forward, strafe, rotate))
    p.apply_movement(forward, strafe, rotate)
    print(p)
    assert(p == t)
        
    print("==============")
    for _ in range(10):
        p = Pose.random()
        t = Pose.random()
        forward, strafe, rotate = p.movement_to(t)
        p.apply_movement(forward, strafe, rotate)
        assert(p == t)
    
