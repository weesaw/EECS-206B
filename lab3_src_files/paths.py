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

    @property
    def going_back(self):
        raise NotImplementedError()

class ArcPath(MotionPath):
    def __init__(self, radius, angle, left_turn, targ_speed):
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
        self.targ_speed = targ_speed
        self.sign = np.sign(angle)
        # self.g = rigid(offset)
        # self.start_theta = offset[2]
        if left_turn:
            self.targ_angular = self.angle/self.total_length*self.targ_speed
        else:
            self.targ_angular = -self.angle/self.total_length*self.targ_speed

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
        if s > self.total_length:
            s = self.total_length

        theta = s/self.total_length*self.angle
        
        x = np.sin(theta)*self.radius
        if self.left_turn:
            y = (1 - np.cos(theta))*self.radius
        else:
            y = -(1 - np.cos(theta))*self.radius

        if not self.left_turn:
            theta = -theta

        theta = (theta+np.pi)%(np.pi*2) - np.pi
        # theta = theta%(np.pi*2)
        return np.array([x, y, theta])
        # return np.append(np.dot(self.g, np.array([x, y, 1]))[:2], theta + self.start_theta)

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
        if s > self.total_length:
            s = self.total_length
            angular = np.array([0, 0 ,0])
        else:
            angular = np.array([0,0,self.targ_angular])

        theta = s/self.total_length*self.angle
        v_x = np.cos(theta)*self.targ_speed
        if self.left_turn:
            v_y = np.sin(theta)*self.targ_speed
        else:
            v_y = -np.sin(theta)*self.targ_speed
            
        linear = np.array([v_x, v_y, 0])
        # linear = np.append(np.dot(self.g, np.array([v_x, v_y, 0]))[:2], [0])
        
        toRet = [linear, angular]
        
        return toRet




    @property
    def total_length(self):
        """ total length of the path
        Returns
        -------
        float
            total length of the path
        """
        # YOUR CODE HERE
        return abs(self.radius*self.angle)

    @property
    def going_back(self):
        return np.sign(self.angle)



class LinearPath(MotionPath):
    def __init__(self, length, targ_speed):
        """
        Parameters
        ----------
        length: float
            length of the path
        """
        self.length = length
        self.targ_speed = targ_speed

        self.sign = np.sign(length)
        # self.start_theta = offset[2]
        # self.g = rigid(offset)
        # self.se = rotation2d(start_theta)


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
        if s > self.total_length:
            s = self.total_length

        return np.array([self.sign*s, 0, 0])
        # return np.append(np.dot(self.g, np.array([s, 0, 1]))[:2], self.start_theta)

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
        if s > self.total_length:
            linear = np.array([0,0,0])
        else:
            linear = np.array([self.sign*self.targ_speed, 0, 0])
        
        angular = np.array([0,0,0])
        toRet = [linear, angular]

        return toRet

    @property
    def total_length(self):
        """ total length of the path
        Returns
        -------
        float
            total length of the path
        """
        return abs(self.length)

    @property
    def going_back(self):
        return np.sign(self.length)

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
        # self.offset = np.array([0, 0, 0])
        self.g = np.eye(3)
        self.start_theta = 0
        # self.start_theta = 0

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

        if s < self.total_length and s - self.transition >= self.subpaths[self.current_path].total_length:
            self.transition += self.subpaths[self.current_path].total_length
            offset = self.subpaths[self.current_path].target_state(self.subpaths[self.current_path].total_length)
            self.g = np.dot(self.g, rigid(offset))
            self.start_theta += offset[2]
            self.start_theta = (self.start_theta+np.pi)%(np.pi*2) - np.pi
            self.current_path += 1
        
        if s > self.total_length:
            s = self.total_length

        state = self.subpaths[self.current_path].target_state(s-self.transition)
        state[:2] = np.dot(self.g, np.append(state[:2], 1))[:2]
        state[2] += self.start_theta

        return state

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
        # if s - self.transition >= self.subpaths[self.current_path].total_length():
        #     self.transition += self.subpaths[self.current_path].total_length()
        #     self.current_path += 1

        if s > self.total_length:
            s = self.total_length

        linear, angular = self.subpaths[self.current_path].target_velocity(s-self.transition)
        linear = np.dot(self.g, linear)

        return [linear, angular]

    @property
    def total_length(self):
        """ total length of the path
        Returns
        -------
        float
            total length of the path
        """
        return self.length

    @property
    def going_back(self):
        return self.subpaths[self.current_path].going_back

def compute_obstacle_avoid_path(dist, obs_center, obs_radius, targ_speed):
    ## Obstcle Avoidance
    # obstacle_center = vec(4*tile, 0.0)
    # obstacle_radius = sqrt(2)*tile/2 + 0.1
    robot_radius = 0.2
    if abs(obs_center[1]) > obs_radius + robot_radius or np.sign(dist) != np.sign(obs_center[0]):
        return LinearPath(dist, targ_speed)

    if np.sign(obs_center[1]) > 0:
        left_turn = False
    else:
        left_turn = True

    r_arc = robot_radius + obstacle_radius
    r_prior = r_arc/4
    l = obstacle_center[0] - robot_radius - obstacle_radius - r_prior
    print l
    
    l_after = dist - l - 2*r_prior - 2*r_arc
    
    p1 = LinearPath(l, targ_speed)
    p2 = ArcPath(r_prior, np.pi/2, left_turn, targ_speed)
    p3 = ArcPath(r_arc, np.pi, not left_turn, targ_speed)
    p4 = ArcPath(r_prior, np.pi/2, left_turn, targ_speed)
    p5 = LinearPath(l_after, targ_speed)
    obs_path = ChainPath([p1, p2, p3, p4, p5])
    return obs_path

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
    print(twists.shape)
    plt.plot(twists[:,0], twists[:,1])
    plt.show()

# YOUR CODE HERE
robot_radius = 0.2
length1 = 0.6096
width = 0.3048
targ_speed1 = 0.1
tile = width

l1 = 0.27
l2 = 0.11
r1 = width - l1

# p1 = LinearPath(length1+r1, targ_speed1)
# p2 = ArcPath(l1+r1, -np.pi/2, False, targ_speed1)
# p3 = LinearPath(l2 - length1/2 + 0.1, targ_speed1)
# p4 = ArcPath(l2, -np.pi/2, True, targ_speed1)
# p5 = LinearPath(l1+l2-length1/2, targ_speed1)
# parallel_parking_path = ChainPath([p1, p2, p3, p4, p5])

# # # YOUR CODE HERE
# r_path = length1 - robot_radius - 0.1
# p1 = ArcPath(r_path, np.pi/2, False, targ_speed1)
# p2 = ArcPath(r_path, -np.pi/2, True, targ_speed1)
# p3 = LinearPath(2*r_path, targ_speed1)
# three_point_turn_path = ChainPath([p1, p2, p3])


obstacle_center = vec(4*tile, 0)
obstacle_radius = sqrt(2)*tile/2 + 0.1
r_arc = robot_radius*2 + obstacle_radius*(1+sqrt(2))
r_prior = r_arc/4
l = obstacle_center[0] - robot_radius - obstacle_radius*sqrt(2) - r_prior
l_after = 0.2
p1 = LinearPath(l, targ_speed1)
p2 = ArcPath(r_prior, np.pi/4, False, targ_speed1)
p3 = ArcPath(r_arc, np.pi/2, True, targ_speed1)
p4 = ArcPath(r_prior, np.pi/4, False, targ_speed1)
p5 = LinearPath(l_after, targ_speed1)
obs_path = ChainPath([p1, p2, p3, p4, p5])



if __name__ == '__main__':
    # path = three_point_turn_path
    # path = parallel_parking_path
    # path = compute_obstacle_avoid_path()
    path = obs_path
    print(path.end_state)
    # print('test')
    plot_path(path)
