import numpy as np
import math
from math import sin, cos, asin, acos, atan2, sqrt
from utils import *
from matplotlib import pyplot as plt
from geometry_msgs.msg import Twist

class MotionPath:
    def target_state(self, s):
        """
        Target position of turtlebot given the path length s

        Parameters
        ----------
        s: float
            the path length the turtlebot should have travelled so far

        Returns
        -------
        :obj:`numpy.ndarray`
            target position of turtlebot
        """
        raise NotImplementedError()

    def target_velocity(self, s):
        """
        Target velocity of turtlebot given the path length s

        Parameters
        ----------
        s: float
            the path length the turtlebot should have travelled so far

        Returns
        -------
        :obj:`numpy.ndarray`
            target velocity of turtlebot
        """
        raise NotImplementedError()

    @property
    def total_length(self):
        """ total path length
        Returns
        -------
        float
            total path length
        """
        raise NotImplementedError()

    @property
    def end_state(self):
        """ Final state after completing the path
        Returns
        -------
        :obj:`numpy.ndarray`
            Final state after completing the path
        """
        return self.target_state(self.total_length)

class ArcPath(MotionPath):
    def __init__(self, radius, angle, left_turn):
        """
        Parameters
        ----------
        radius: float
            how big of a circle in meters
        angle: float
            how much of the circle do you want to complete (in radians).  
            Can be positive or negative
        left_turn: bool
            whether the turtlebot should turn left or right
        """
        self.radius = radius
        self.angle = angle
        self.left_turn = left_turn

    def target_state(self, s):
        """
        Target position of turtlebot given the current path length s for Circular Arc Path

        Parameters
        ----------
        s: float
            the path length the turtlebot should have travelled so far

        Returns
        -------
        :obj:`numpy.ndarray`
            target position of turtlebot
        """
        # YOUR CODE HERE

    def target_velocity(self, s):
        """
        Target velocity of turtlebot given the current path length s for Circular Arc Path

        Parameters
        ----------
        s: float
            the path length the turtlebot should have travelled so far

        Returns
        -------
        :obj:`numpy.ndarray`
            target velocity of turtlebot
        """
        # YOUR CODE HERE

    @property
    def total_length(self):
        """ total length of the path
        Returns
        -------
        float
            total length of the path
        """
        # YOUR CODE HERE

class LinearPath(MotionPath):
    def __init__(self, length, targ_speed, start_theta):
        """
        Parameters
        ----------
        length: float
            length of the path
        """
        self.length = length
        self.targ_speed = targ_speed
        self.start_theta = start_theta
        self.R = rotation2d(start_theta)


    def target_state(self, s):
        """
        Target position of turtlebot given the current path length s for Linear Path

        Parameters
        ----------
        s: float
            the path length the turtlebot should have travelled so far

        Returns
        -------
        :obj:`numpy.ndarray`
            target position of turtlebot
        """
        return np.append(np.dot(self.R, np.array([0, s+ self.targ_speed/10])), [self.start_theta])

    def target_velocity(self, s):
        """
        Target velocity of turtlebot given the current path length s for Linear Path

        Parameters
        ----------
        s: float
            the path length the turtlebot should have travelled so far

        Returns
        -------
        :obj:`numpy.ndarray`
            target velocity of turtlebot
        """
        toRet = Twist()
        toRet.linear = np.append(np.dot(self.R, np.array([0,self.targ_speed])), [0])
        toRet.angular = np.array([0,0,0])
        return toRet

    @property
    def total_length(self):
        """ total length of the path
        Returns
        -------
        float
            total length of the path
        """
        return self.length

class ChainPath(MotionPath):
    def __init__(self, subpaths):
        """
        Parameters
        ----------
        subpaths: :obj:`list` of :obj:`MotionPath`
            list of paths which should be chained together
        """
        self.subpaths = subpaths
        self.length = sum([subpath.total_length for subpath in subpaths])
        self.current_path = 0
        self.transition = 0

    def target_state(self, s):
        """
        Target position of turtlebot given the current path length s for Chained Path

        Parameters
        ----------
        s: float
            the path length the turtlebot should have travelled so far

        Returns
        -------
        :obj:`numpy.ndarray`
            target position of turtlebot
        """
        if s - self.transition >= self.subpaths[self.current_path].total_length():
            self.transition += self.subpaths[self.current_path].total_length()
            self.current_path += 1



        return self.subpaths[self.current_path].target_state(s-self.transition)

    def target_velocity(self, s):
        """
        Target velocity of turtlebot given the current path length s for Chained Path

        Parameters
        ----------
        s: float
            the path length the turtlebot should have travelled so far

        Returns
        -------
        :obj:`numpy.ndarray`
            target velocity of turtlebot
        """
        if s - self.transition >= self.subpaths[self.current_path].total_length():
            self.transition += self.subpaths[self.current_path].total_length()
            self.current_path += 1



        return self.subpaths[self.current_path].target_velocity(s-self.transition)

    @property
    def total_length(self):
        """ total length of the path
        Returns
        -------
        float
            total length of the path
        """
        return self.length

def compute_obstacle_avoid_path(dist, obs_center, obs_radius):
    return

def plot_path(path):
    """
    Plots on a 2D plane, the top down view of the path passed in

    Parameters
    ----------
    path: :obj:`MotionPath`
        Path to plot
    """
    s = np.linspace(0, path.total_length, 1000, endpoint=False)
    twists = np.array(list(path.target_state(si) for si in s))

    plt.plot(twists[:,0], twists[:,1])
    plt.show()

# YOUR CODE HERE
parallel_parking_path = ChainPath([])

# YOUR CODE HERE
three_point_turn_path = ChainPath([])

if __name__ == '__main__':
    path = three_point_turn_path
    # path = compute_obstacle_avoid_path()
    print(path.end_state)
    plot_path(path)
