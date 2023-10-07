import math

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
        if self.theta_rads >= 0:
            theta_norm_rads = self.theta_rads % (2 * math.pi)
        else:
            theta_norm_rads = (self.theta_rads % (2 * math.pi)) - 2 * math.pi
        return "(%8.4f %8.4f %8.4f)" % (self.x_in, self.y_in, theta_norm_rads)

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
    
    
