# The particle filter, prediciton and correction.
# In addition to the filtered particles, the density is estimated. In this
# simple case, the mean position and heading is computed from all particles.
#
# slam_08_c_density_estimation.
# Claus Brenner, 04.01.2013
from lego_robot import *
from slam_e_library import get_cylinders_from_scan, assign_cylinders
from math import sin, cos, pi, atan2, sqrt
import random
from scipy.stats import norm as normal_dist


class ParticleFilter:
    def __init__(self, initial_particles,
                 robot_width, scanner_displacement,
                 control_motion_factor, control_turn_factor,
                 measurement_distance_stddev, measurement_angle_stddev):
        # The particles.
        self.particles = initial_particles

        # Some constants.
        self.robot_width = robot_width
        self.scanner_displacement = scanner_displacement
        self.control_motion_factor = control_motion_factor
        self.control_turn_factor = control_turn_factor
        self.measurement_distance_stddev = measurement_distance_stddev
        self.measurement_angle_stddev = measurement_angle_stddev

    # State transition. This is exactly the same method as in the Kalman filter.
    @staticmethod
    def g(state, control, w):
        x, y, theta = state
        l, r = control
        if r != l:
            alpha = (r - l) / w
            rad = l/alpha
            g1 = x + (rad + w/2.)*(sin(theta+alpha) - sin(theta))
            g2 = y + (rad + w/2.)*(-cos(theta+alpha) + cos(theta))
            g3 = (theta + alpha + pi) % (2*pi) - pi
        else:
            g1 = x + l * cos(theta)
            g2 = y + l * sin(theta)
            g3 = theta

        return (g1, g2, g3)

    def predict(self, control):
        """The prediction step of the particle filter."""

        # --->>> Put the code of a previous solution here.
        left, right = control
        # --->>> Insert code from previous question here.
        sigma_l2 = (self.control_motion_factor*left)**2 + (self.control_turn_factor*(left-right))**2
        sigma_r2 = (self.control_motion_factor*right)**2 + (self.control_turn_factor*(left-right))**2
        
        particles_predict = []
        for particle in self.particles:
            left_sample = random.gauss(left, sqrt(sigma_l2))
            right_sample = random.gauss(right, sqrt(sigma_r2))
            control_sample = [left_sample, right_sample]
            particles_predict.append(self.g(particle, control_sample, self.robot_width))
        
        self.particles = particles_predict

    # Measurement. This is exactly the same method as in the Kalman filter.
    @staticmethod
    def h(state, landmark, scanner_displacement):
        """Takes a (x, y, theta) state and a (x, y) landmark, and returns the
           corresponding (range, bearing)."""
        dx = landmark[0] - (state[0] + scanner_displacement * cos(state[2]))
        dy = landmark[1] - (state[1] + scanner_displacement * sin(state[2]))
        r = sqrt(dx * dx + dy * dy)
        alpha = (atan2(dy, dx) - state[2] + pi) % (2*pi) - pi
        return (r, alpha)

    def probability_of_measurement(self, measurement, predicted_measurement):
        """Given a measurement and a predicted measurement, computes
           probability."""

        # --->>> Put the code of a previous solution here.
        dist, alpha = measurement
        dist_pre, alpha_pre = predicted_measurement
        
        dist_diff = dist - dist_pre
        alpha_diff = (alpha - alpha_pre + pi) % (2*pi) - pi
            
        alpha_prob = normal_dist.pdf(alpha_diff, 0, self.measurement_angle_stddev)
        dist_prob = normal_dist.pdf(dist_diff, 0, self.measurement_distance_stddev)
            
        return alpha_prob * dist_prob

    def compute_weights(self, cylinders, landmarks):
        """Computes one weight for each particle, return list of weights."""

        # --->>> Put the code of a previous solution here.
        weights = []
        for p in self.particles:
            # Get list of tuples:
            # [ ((range_0, bearing_0), (landmark_x, landmark_y)), ... ]
            assignment = assign_cylinders(cylinders, p,
                self.scanner_displacement, landmarks)

            # --->>> Insert code to compute weight for particle p here.
            # This will require a loop over all (measurement, landmark)
            # in assignment. Append weight to the list of weights.
            weight = 1.0
            for pair in assignment:
                measurement, landmark = pair
                predicted_measurement = self.h(p, landmark, self.scanner_displacement)
                weight *= self.probability_of_measurement(measurement, predicted_measurement)
            
            weights.append(weight)  # Replace this.
        return weights

    def resample(self, weights):
        """Return a list of particles which have been resampled, proportional
           to the given weights."""

        # --->>> Put the code of a previous solution here.
        new_particles = []
        max_weight = max(weights) # max of weights
        M = len(weights) # length of weights
        offset = 0.0
        index = random.randint(0,M-1)
        for i in xrange(M):
            offset += random.uniform(0, 2*max_weight)
            while (offset > weights[index]):
                offset -= weights[index]
                index = (index+1) % M
            new_particles.append(self.particles[index])
            
        return new_particles

    def correct(self, cylinders, landmarks):
        """The correction step of the particle filter."""
        # First compute all weights.
        weights = self.compute_weights(cylinders, landmarks)
        # Then resample, based on the weight array.
        self.particles = self.resample(weights)

    def print_particles(self, file_desc):
        """Prints particles to given file_desc output."""
        if not self.particles:
            return
        print >> file_desc, "PA",
        for p in self.particles:
            print >> file_desc, "%.0f %.0f %.3f" % p,
        print >> file_desc

    def get_mean(self):
        """Compute mean position and heading from all particles."""

        # --->>> This is the new code you'll have to implement.
        # Return a tuple: (mean_x, mean_y, mean_heading).
        sum_x = 0.0
        sum_y = 0.0
        sum_vx = 0.0
        sum_vy = 0.0 
        M = len(self.particles)
        
        for p in self.particles:
            sum_x += p[0]
            sum_y += p[1]
            sum_vx += cos(p[2])
            sum_vy += sin(p[2])
            
        mean_x = sum_x/M
        mean_y = sum_y/M
        mean_heading = atan2(sum_vy, sum_vx)
        
        return (mean_x, mean_y, mean_heading) 


if __name__ == '__main__':
    # Robot constants.
    scanner_displacement = 30.0
    ticks_to_mm = 0.349
    robot_width = 155.0

    # Cylinder extraction and matching constants.
    minimum_valid_distance = 20.0
    depth_jump = 100.0
    cylinder_offset = 90.0

    # Filter constants.
    control_motion_factor = 0.35  # Error in motor control.
    control_turn_factor = 0.6  # Additional error due to slip when turning.
    measurement_distance_stddev = 200.0  # Distance measurement error of cylinders.
    measurement_angle_stddev = 15.0 / 180.0 * pi  # Angle measurement error.

    # Generate initial particles. Each particle is (x, y, theta).
    number_of_particles = 200
    measured_state = (1850.0, 1897.0, 213.0 / 180.0 * pi)
    standard_deviations = (100.0, 100.0, 10.0 / 180.0 * pi)
    initial_particles = []
    for i in xrange(number_of_particles):
        initial_particles.append(tuple([
            random.gauss(measured_state[j], standard_deviations[j])
            for j in xrange(3)]))

    # Setup filter.
    pf = ParticleFilter(initial_particles,
                        robot_width, scanner_displacement,
                        control_motion_factor, control_turn_factor,
                        measurement_distance_stddev,
                        measurement_angle_stddev)

    # Read data.
    logfile = LegoLogfile()
    logfile.read("robot4_motors.txt")
    logfile.read("robot4_scan.txt")
    logfile.read("robot_arena_landmarks.txt")
    reference_cylinders = [l[1:3] for l in logfile.landmarks]

    # Loop over all motor tick records.
    # This is the particle filter loop, with prediction and correction.
    f = open("particle_filter_mean.txt", "w")
    for i in xrange(len(logfile.motor_ticks)):
        # Prediction.
        control = map(lambda x: x * ticks_to_mm, logfile.motor_ticks[i])
        pf.predict(control)

        # Correction.
        cylinders = get_cylinders_from_scan(logfile.scan_data[i], depth_jump,
            minimum_valid_distance, cylinder_offset)
        pf.correct(cylinders, reference_cylinders)

        # Output particles.
        pf.print_particles(f)
        
        # Output state estimated from all particles.
        mean = pf.get_mean()
        print >> f, "F %.0f %.0f %.3f" %\
              (mean[0] + scanner_displacement * cos(mean[2]),
               mean[1] + scanner_displacement * sin(mean[2]),
               mean[2])

    f.close()
