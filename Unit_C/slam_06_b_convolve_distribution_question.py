# Instead of moving a distribution, move (and modify) it using a convolution.
# 06_b_convolve_distribution
# Claus Brenner, 26 NOV 2012
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

    # --->>> Put your code here.
    
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
        
#    for i in xrange(len(distri_list)):
#        print distri_list[i]
    
    return Distribution.sum(distri_list)
    

if __name__ == '__main__':
    arena = (0,1000)

    # Move 3 times by 20.
    moves = [20] * 250  

    # Start with a known position: probability 1.0 at position 10.
    position = Distribution.unit_pulse(10)
    
    plot(position.plotlists(*arena)[0], position.plotlists(*arena)[1],
         linestyle='steps')

    # Now move and plot.
    for m in moves:
        move_distribution = Distribution.triangle(m, 2)
        
        # convolution 
        position = convolve(position, move_distribution)
        
        plot(position.plotlists(*arena)[0], position.plotlists(*arena)[1],
             linestyle='steps')

    ylim(0.0, 1.1)
    show()
