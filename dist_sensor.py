import math
import pose
import robot


def power_for_distance_sensor(r : robot.Robot,  d_left_in, d_right_in, sensor_span_in, target_dist_in):
    """ Compute the power to set the four motors to drive to backdrop given
        values of the two distance sensors.

          r                 robot
          d_left_in         reading of the left distance sensor
          d_right_in        reading of the right distance sensor
          sensor_span_in    distance between two distance sensor (TODO: make part of robot class)
          target_dist_in    desired distance from the backdrop

    """

    # compute our current angle to the backdrop
    delta_d_in = d_right_in - d_left_in
    theta_rads = math.atan(delta_d_in / sensor_span_in)

    # compute how far forward we need to go
    q = target_dist_in * math.sqrt(delta_d_in * delta_d_in + sensor_span_in * sensor_span_in) / sensor_span_in
    forward_in = (d_left_in + d_right_in) / 2.0 - q

    # pretend we are in the default pose and make the target pose the change we want
    r = robot.Robot()
    start = pose.Pose(0.0, 0.0, 0.0)
    target = pose.Pose(forward_in, 0.0, theta_rads)

    # call power_for_target to get the needed motor powers.  TODO: set start_slowdown_in and min_power
    power = r.power_for_target(start, target)
    assert(math.fabs(power[robot.FRONT_LEFT] - power[robot.BACK_LEFT]) < .0001)
    assert(math.fabs(power[robot.FRONT_RIGHT] - power[robot.BACK_RIGHT]) < .0001)
    return (power[robot.FRONT_LEFT], power[robot.FRONT_RIGHT])

r = robot.Robot()
print(power_for_distance_sensor(r, 13.0, 10.0, 8.0, 3.0))
print(power_for_distance_sensor(r, 8.0, 10.0, 8.0, 3.0))
print(power_for_distance_sensor(r, 12.0, 12.0, 8.0, 3.0))



