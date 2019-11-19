# Subsample the scan. For each point, find a closest point on the
# wall of the arena.
# From those point pairs, estimate a transform and apply this to the pose.
# Repeat the closest point - estimate transform loop.
# This is an ICP algorithm.
# 05_c_icp_wall_transform
# Claus Brenner, 17 NOV 2012
from lego_robot import *
from slam_b_library import filter_step, concatenate_transform,\
    compute_cartesian_coordinates, write_cylinders
from math import sqrt, atan2

# Given a point list, return the center of mass.
def compute_center(point_list):
    # Safeguard against empty list.
    if not point_list:
        return (0.0, 0.0)
    # If not empty, sum up and divide.
    sx = sum([p[0] for p in point_list])
    sy = sum([p[1] for p in point_list])
    return (float(sx) / len(point_list), float(sy) / len(point_list))


# Given a left_list of points and a right_list of points, compute
# the parameters of a similarity transform: scale, rotation, translation.
# If fix_scale is True, use the fixed scale of 1.0.
# The returned value is a tuple of:
# (scale, cos(angle), sin(angle), x_translation, y_translation)
# i.e., the rotation angle is not given in radians, but rather in terms
# of the cosine and sine.
def estimate_transform(left_list, right_list, fix_scale = False):
    # Compute left and right center.
    lc = compute_center(left_list)
    rc = compute_center(right_list)

    # --->>> Insert your previous solution here.
    # Compute the new left and right list
    left_new = []
    right_new = []
#    print len(left_list), len(right_list)
    for i in range(len(left_list)):
        left_new.append([left_list[i][0]-lc[0], left_list[i][1]-lc[1]])
        right_new.append([right_list[i][0]-rc[0], right_list[i][1]-rc[1]])
#        print left_list[i], right_list[i], left_new[i], right_new[i]
    
    # Compute sum of cos, sin, length of vector r, l 
    cs = 0.0
    ss = 0.0
    rr = 0.0
    ll = 0.0
    
    for i in range(len(left_list)):
        cs += right_new[i][0]*left_new[i][0] + right_new[i][1]*left_new[i][1]
        ss += -right_new[i][0]*left_new[i][1] + right_new[i][1]*left_new[i][0]
        rr += right_new[i][0]*right_new[i][0] + right_new[i][1]*right_new[i][1]
        ll += left_new[i][0]*left_new[i][0] + left_new[i][1]*left_new[i][1]
    
    # 4 parameter need 4 observation 
    if rr == 0 and ll == 0:     
        return None
    
    # Compute the lambda, scala variable
    if fix_scale == False:
        la = sqrt(rr/ll)
    elif fix_scale == True:
        la = 1.0
    
    # Compute the rotation angle in cos and sin term
    if abs(sqrt(pow(cs,2) + pow(ss,2)) - 0.0) !=  0:
        c = cs / sqrt(pow(cs,2) + pow(ss,2))
        s = ss / sqrt(pow(cs,2) + pow(ss,2))
    else:
        return None
    
    # compute the translation
    tx = rc[0] - la*(c*lc[0] - s*lc[1])
    ty = rc[1] - la*(s*lc[0] + c*lc[1])

    return la, c, s, tx, ty

# Given a similarity transformation:
# trafo = (scale, cos(angle), sin(angle), x_translation, y_translation)
# and a point p = (x, y), return the transformed point.
def apply_transform(trafo, p):
    la, c, s, tx, ty = trafo
    lac = la * c
    las = la * s
    x = lac * p[0] - las * p[1] + tx
    y = las * p[0] + lac * p[1] + ty
    return (x, y)


# Correct the pose = (x, y, heading) of the robot using the given
# similarity transform. Note this changes the position as well as
# the heading.
def correct_pose(pose, trafo):
    
    # --->>> Insert your previous solution here.
    la = trafo[0]
    c = trafo[1]
    s = trafo[2]
    tx = trafo[3]
    ty = trafo[4]
    
    if trafo:
        x_new = la*(c*pose[0]-s*pose[1])+tx
        y_new = la*(s*pose[0]+c*pose[1])+ty
        alpha = atan2(s,c)  
        theta_new = pose[2] + alpha
      
        return (x_new, y_new, theta_new)
      
    elif not trafo:
        return (pose[0], pose[1], pose[2])  # Replace this by the corrected pose.


# Takes one scan and subsamples the measurements, so that every sampling'th
# point is taken. Returns a list of (x, y) points in the scanner's
# coordinate system.
def get_subsampled_points(scan, sampling = 10):
    # Subsample from scan
    index_range_tuples = []
    for i in xrange(0, len(scan), sampling):
        index_range_tuples.append( (i, scan[i]) )
    return compute_cartesian_coordinates(index_range_tuples, 0.0)


# Given a set of points, checks for every point p if it is closer than
# eps to the left, right, upper or lower wall of the arena. If so,
# adds the point to left_list, and the closest point on the wall to
# right_list.
def get_corresponding_points_on_wall(points,
                                     arena_left = 0.0, arena_right = 2000.0,
                                     arena_bottom = 0.0, arena_top = 2000.0,
                                     eps = 150.0):
    left_list = []
    right_list = []

    # ---> Insert your previous solution here.
    for point in points:
        if abs(point[0]-arena_left) < eps:
            left_list.append(point)
            right_list.append((arena_left, point[1]))
        elif abs(point[0]-arena_right) < eps:
            left_list.append(point)
            right_list.append((arena_right, point[1]))
        elif abs(point[1]-arena_bottom) < eps:
            left_list.append(point)
            right_list.append((point[0], arena_bottom))
        elif abs(point[1]-arena_top) < eps:
            left_list.append(point)
            right_list.append((point[0], arena_top))
    

    return left_list, right_list


# ICP: Iterate the steps of transforming the points, selecting point pairs, and
# estimating the transform. Returns the final transformation.
def get_icp_transform(world_points, iterations):

    # Iterate assignment and estimation of trafo a few times.

    # --->>> Implement your code here.
    
    # You may use the following strategy:
    # Start with the identity transform:
    #   overall_trafo = (1.0, 1.0, 0.0, 0.0, 0.0)
    # Then loop for j in xrange(iterations):
    #   Transform the world_points using the curent overall_trafo
    #     (see 05_b on how to do this)
    #   Call get_correspoinding_points_on_wall(...)
    #   Determine transformation which is needed "on top of" the current
    #     overall_trafo: trafo = estimate_transform(...)
    #   Concatenate the found transformation with the current overall_trafo
    #     to obtain a new, 'combined' transformation. You may use the function
    #     overall_trafo = concatenate_transform(trafo, overall_trafo)
    #     to concatenate two similarities.
    #   Note also that estimate_transform may return None.
    # 

    # Return the final transformation.
    
    # Init the trafo
    overall_trafo = (1.0, 1.0, 0.0, 0.0, 0.0)
    
    # Iterative Closest Points
    for j in xrange(iterations):
        # transform the points from LiDAR on the wall to the new estimation
        world_points_new = [apply_transform(overall_trafo, p) for p in world_points]
        # find the corresponding points on the wall
        left, right = get_corresponding_points_on_wall(world_points_new)
        # find the trafo for the next iteration
        trafo = estimate_transform(left, right, fix_scale = True)
        # concatenate the trafo ,like trafo chain
        if trafo:
            overall_trafo = concatenate_transform(trafo, overall_trafo)
            
        else:
            break
    
    return overall_trafo

if __name__ == '__main__':
    # The constants we used for the filter_step.
    scanner_displacement = 30.0
    ticks_to_mm = 0.349
    robot_width = 150.0

    # The start pose we obtained miraculously.
    pose = (1850.0, 1897.0, 3.717551306747922)

    # Read the logfile which contains all scans.
    logfile = LegoLogfile()
    logfile.read("robot4_motors.txt")
    logfile.read("robot4_scan.txt")

    # Iterate over all positions.
    out_file = file("icp_wall_transform.txt", "w")
    for i in xrange(len(logfile.scan_data)):
        # Compute the new pose.
        pose = filter_step(pose, logfile.motor_ticks[i],
                           ticks_to_mm, robot_width,
                           scanner_displacement)

        # Subsample points.
        subsampled_points = get_subsampled_points(logfile.scan_data[i])
        world_points = [LegoLogfile.scanner_to_world(pose, c)
                        for c in subsampled_points]

        # Get the transformation.
        # You may play withe the number of iterations here to see
        # the effect on the trajectory!
        trafo = get_icp_transform(world_points, iterations = 40)

        # Correct the initial position using trafo.
        pose = correct_pose(pose, trafo)

        # Write to file.
        # The pose.
        print >> out_file, "F %f %f %f" % pose
        # Write the scanner points and corresponding points.
        write_cylinders(out_file, "W C",
            [apply_transform(trafo, p) for p in world_points])

    out_file.close()
