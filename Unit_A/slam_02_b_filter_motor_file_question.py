# Implement the second move model for the Lego robot.
# The difference to the first implementation is:
# - added a scanner displacement
# - added a different start pose (measured in the real world)
# - result is now output to a file, as "F" ("filtered") records.
#
# 02_b_filter_motor_file
# Claus Brenner, 09 NOV 2012
from math import sin, cos, pi
from lego_robot import *

# This function takes the old (x, y, heading) pose and the motor ticks
# (ticks_left, ticks_right) and returns the new (x, y, heading).
def filter_step(old_pose, motor_ticks, ticks_to_mm, robot_width,
                scanner_displacement):
    
    # motor_ticks to real distance
    real_distance = [motor_ticks[0] * float(ticks_to_mm), motor_ticks[1] * float(ticks_to_mm)]
    
    # Find out if there is a turn at all.
    if motor_ticks[0] == motor_ticks[1]:
        # No turn. Just drive straight.

        # --->>> Use your previous implementation.
        # Think about if you need to modify your old code due to the
        # scanner displacement? no 
        x = old_pose[0] + real_distance[0]*cos(old_pose[2])
        y = old_pose[1] + real_distance[0]*sin(old_pose[2])
        theta = old_pose[2]
        
        return (x, y, theta)

    else:
        # Turn. Compute alpha, R, etc.

        # --->>> Modify your previous implementation.
        # First modify the the old pose to get the center (because the
        #   old pose is the LiDAR's pose, not the robot's center pose).
        # Second, execute your old code, which implements the motion model
        #   for the center of the robot.
        # Third, modify the result to get back the LiDAR pose from
        #   your computed center. This is the value you have to return.
        x_displacement = old_pose[0] - scanner_displacement*cos(old_pose[2])
        y_displacement = old_pose[1] - scanner_displacement*sin(old_pose[2])
        
        alpha = (real_distance[1] - real_distance[0]) / float(robot_width)
        R = real_distance[0]/float(alpha)
        cx = float(x_displacement) - (R+robot_width/2.0)*sin(old_pose[2])
        cy = float(y_displacement) - (R+robot_width/2.0)*(-cos(old_pose[2]))
        theta = (old_pose[2] + alpha)%(2*pi)
        x_temp = cx + (R+robot_width/2.0)*sin(theta)
        y_temp = cy + (R+robot_width/2.0)*(-cos(theta))
        
        x = x_temp + scanner_displacement*cos(theta)
        y = y_temp + scanner_displacement*sin(theta)

        return (x, y, theta)

if __name__ == '__main__':
    # Empirically derived distance between scanner and assumed
    # center of robot.
    scanner_displacement = 30.0

    # Empirically derived conversion from ticks to mm.
    ticks_to_mm = 0.349

    # Measured width of the robot (wheel gauge), in mm.
    #robot_width = 150.0
    robot_width = 171.0
    # Measured start position.
    pose = (1850.0, 1897.0, 213.0 / 180.0 * pi)

    # Read data.
    logfile = LegoLogfile()
    logfile.read("robot4_motors.txt")

    # Loop over all motor tick records generate filtered position list.
    filtered = []
    for ticks in logfile.motor_ticks:
        pose = filter_step(pose, ticks, ticks_to_mm, robot_width,
                           scanner_displacement)
        filtered.append(pose)

    # Write all filtered positions to file.
    f = open("poses_from_ticks.txt", "w")
    for pose in filtered:
        print >> f, "F %f %f %f" % pose
    f.close()
