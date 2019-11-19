# Histogram implementation of a bayes filter - combines
# convolution and multiplication of distributions, for the
# movement and measurement steps.
# 06_d_histogram_filter
# Claus Brenner, 28 NOV 2012
from pylab import plot, show, ylim
from distribution import *

def move(distribution, delta):
    """Returns a Distribution that has been moved (x-axis) by the amount of
       delta."""

    # --->>> Copy your previous code here.
    new_center = distribution.offset + delta
    new_values = distribution.values
    new_distribution = Distribution(new_center,new_values)
    
    return new_distribution  # Replace this by your own result.


def convolve(a, b):
    """Convolve distribution a and b and return the resulting new distribution."""

    # --->>> Copy your previous code here.
    
    # offset for next step
    offset = a.offset + b.offset;
    
    # init the distri_list
    distri_list = []
    
    # calculate several distributions according to the number of a
    for a_value in a.values:
        distri_value = []
        for b_value in b.values:
            distri_value.append(a_value*b_value)
        distri_list.append(Distribution(offset, distri_value))
        offset += 1
    
    return Distribution.sum(distri_list)


def multiply(a, b):
    """Multiply two distributions and return the resulting distribution."""

    # --->>> Copy your previous code here.
    start = min([a.start(), b.start()])
    stop = max([a.stop(), b.stop()])
    
    values = []
    
    for i in range(start, stop):
        values.append(a.value(i) * b.value(i))
    
    result_distri = Distribution(start, values)
    
    result_distri.normalize()
    
    return result_distri


if __name__ == '__main__':
    arena = (0,220)

    # Start position. Exactly known - a unit pulse.
    start_position = 10
    
    # start distribution, unit pulse
    position = Distribution.unit_pulse(start_position)
    plot(position.plotlists(*arena)[0], position.plotlists(*arena)[1],
         linestyle='steps')

    # Movement data.
    controls  =    [ 20 ] * 10

    # Measurement data. Assume (for now) that the measurement data
    # is correct. - This code just builds a cumulative list of the controls,
    # plus the start position.
    p = start_position
    measurements = []
    for c in controls:
        p += c
        measurements.append(p)

    # This is the filter loop.
    for i in xrange(len(controls)):
        # Move, by convolution. Also termed "prediction".
        control = Distribution.triangle(controls[i], 10)
        position = convolve(position, control)
        plot(position.plotlists(*arena)[0], position.plotlists(*arena)[1],
             color='b', linestyle='steps')

        # Measure, by multiplication. Also termed "correction".
        measurement = Distribution.triangle(measurements[i], 10)
        position = multiply(position, measurement)
        plot(position.plotlists(*arena)[0], position.plotlists(*arena)[1],
             color='r', linestyle='steps')

    show()
