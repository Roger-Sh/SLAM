# For each cylinder in the scan, find its cartesian coordinates,
# in the world coordinate system.
# Find the closest pairs of cylinders from the scanner and cylinders
# from the reference, and the optimal transformation which aligns them.
# Then, use this transform to correct the pose.
# 04_d_apply_transform
# Claus Brenner, 14 NOV 2012
from lego_robot import *
from slam_b_library import filter_step, compute_scanner_cylinders,\
    write_cylinders
from math import sqrt, atan2

# Given a list of cylinders (points) and reference_cylinders:
# For every cylinder, find the closest reference_cylinder and add
# the index pair (i, j), where i is the index of the cylinder, and
# j is the index of the reference_cylinder, to the result list.
# This is the function developed in slam_04_b_find_cylinder_pairs.
def find_cylinder_pairs(cylinders, reference_cylinders, max_radius):
    cylinder_pairs = []

    # --->>> Insert your previous solution here.
    for index1 in range(len(cylinders)):
        radius_old = max_radius
        
        for index2 in range(len(reference_cylinders)):
            radius_new = sqrt(
                    pow((cylinders[index1][0]-reference_cylinders[index2][0]),2)+
                    pow((cylinders[index1][1]-reference_cylinders[index2][1]),2))
            
            if radius_new < max_radius and radius_new < radius_old:
                radius_old = radius_new
                i = index1
                j = index2
    
        if radius_old < max_radius:
            cylinder_pairs.append((i,j))
        
    return cylinder_pairs

# Given a point list, return the center of mass.
def compute_center(point_list):
    # Safeguard against empty list.
    if not point_list:
        return (0.0, 0.0)
    # If not empty, sum up and divide.
    sx = sum([p[0] for p in point_list])
    sy = sum([p[1] for p in point_list])
    return (sx / len(point_list), sy / len(point_list))

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
    
    
    # compute the tranlation
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
    # --->>> This is what you'll have to implement.
    
    # transfer the variable, give them names
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


if __name__ == '__main__':
    # The constants we used for the filter_step.
    scanner_displacement = 30.0
    ticks_to_mm = 0.349
    robot_width = 150.0

    # The constants we used for the cylinder detection in our scan.    
    minimum_valid_distance = 20.0
    depth_jump = 100.0
    cylinder_offset = 90.0

    # The maximum distance allowed for cylinder assignment.
    max_cylinder_distance = 400.0

    # The start pose we obtained miraculously.
    pose = (1850.0, 1897.0, 3.717551306747922)

    # Read the logfile which contains all scans.
    logfile = LegoLogfile()
    logfile.read("robot4_motors.txt")
    logfile.read("robot4_scan.txt")

    # Also read the reference cylinders (this is our map).
    logfile.read("robot_arena_landmarks.txt")
    reference_cylinders = [l[1:3] for l in logfile.landmarks]

    out_file = file("apply_transform.txt", "w")
    for i in xrange(len(logfile.scan_data)):
        # Compute the new pose.
        pose = filter_step(pose, logfile.motor_ticks[i],
                           ticks_to_mm, robot_width,
                           scanner_displacement)

        # Extract cylinders, also convert them to world coordinates.
        cartesian_cylinders = compute_scanner_cylinders(
            logfile.scan_data[i],
            depth_jump, minimum_valid_distance, cylinder_offset)
        world_cylinders = [LegoLogfile.scanner_to_world(pose, c)
                           for c in cartesian_cylinders]

        # For every cylinder, find the closest reference cylinder.
        cylinder_pairs = find_cylinder_pairs(
            world_cylinders, reference_cylinders, max_cylinder_distance)

        # Estimate a transformation using the cylinder pairs.
        trafo = estimate_transform(
            [world_cylinders[pair[0]] for pair in cylinder_pairs],
            [reference_cylinders[pair[1]] for pair in cylinder_pairs],
            fix_scale = True)

        # Transform the cylinders using the estimated transform.
        transformed_world_cylinders = []
        if trafo:
            transformed_world_cylinders =\
                [apply_transform(trafo, c) for c in
                 [world_cylinders[pair[0]] for pair in cylinder_pairs]]

        # Also apply the trafo to correct the position and heading.
        if trafo:
            pose = correct_pose(pose, trafo)

        # Write to file.
        # The pose.
        print >> out_file, "F %f %f %f" % pose
        # The detected cylinders in the scanner's coordinate system.
        write_cylinders(out_file, "D C", cartesian_cylinders)
        # The detected cylinders, transformed using the estimated trafo.
        write_cylinders(out_file, "W C", transformed_world_cylinders)

    out_file.close()
