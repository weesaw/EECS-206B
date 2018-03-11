#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist, Vector3
from paths import *
from utils import *
from controllers import *
import tf
import tf.transformations as tfs

cmd_vel = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)
target_speed = .5

# path = parallel_parking_path
# path = three_point_turn_path
# path = compute_obstacle_avoid_path(4.0, vec(2.0, -0.0), 0.5)
path = LinearPath(3.5, target_speed, 0)

k = [0,0,0]


obstacle = False
obstacle_center = vec(2.0, -0.0)
obstacle_radius = 1

controller = Controller(path, k, target_speed, obstacle, obstacle_center, obstacle_radius)

def main():
    rospy.init_node('Lab3', anonymous=False)

    rospy.loginfo("To stop TurtleBot CTRL + C")
    rospy.on_shutdown(shutdown)
    
    # setting up the transform listener to find turtlebot position
    listener = tf.TransformListener()
    from_frame = 'odom'
    to_frame = 'base_link'
    listener.waitForTransform(from_frame, to_frame, rospy.Time(), rospy.Duration(5.0))
    broadcaster = tf.TransformBroadcaster()

    # this is so that each loop of the while loop takes the same amount of time.  The controller works better 
    # if you have this here
    rate = rospy.Rate(10)

    # getting the position of the 
    start_pos, start_rot = listener.lookupTransform(from_frame, to_frame, listener.getLatestCommonTime(from_frame, to_frame))
    # 3x1 array, representing (x,y,theta) of robot starting state
    start_state = np.array([start_pos[0], start_pos[1], np.arctan2(2*(start_rot[0]*start_rot[3] + start_rot[1]*start_rot[2]), 1-2*(start_rot[2]**2 + start_rot[3]**2))])

    times = []
    actual_states = []
    target_states = []
    s = 0
    while not rospy.is_shutdown() and s <= path.total_length:
        current_pos, current_rot = listener.lookupTransform(from_frame, to_frame, listener.getLatestCommonTime(from_frame, to_frame))
        # 3x1 array, representing (x,y,theta) of current robot state
        current_state = np.array([current_pos[0], current_pos[1], np.arctan2(2*(current_rot[0]*current_rot[3] + current_rot[1]*current_rot[2]), 1-2*(current_rot[2]**2 + current_rot[3]**2))])
        # 3x1 array representing (x,y,theta) of current robot state, relative to starting state.  look at rigid method in utils.py
        current_state = current_state - start_state
        
        # for the plot at the end
        times.append(s * 10)
        actual_states.append(current_state)
        target_states.append(controller.path.target_state(s))

        # I may have forgotten some parameters here
        move_cmd = controller.step_path(current_state, s)
        # print(move_cmd)
        cmd_vel.publish(move_cmd)

        # I believe this should be the same as the ros rate time, so if you change that, change it here too
        s += target_speed / 10
        # this is governing how much each loop should run for.  look up ROS rates if you're interested
        rate.sleep()

    times = np.array(times)
    actual_states = np.array(actual_states)
    target_states = np.array(target_states)

    plt.figure()
    colors = ['blue', 'green', 'red']
    labels = ['x', 'y', 'theta']
    for i in range(3):
        plt.plot(times, actual_states[:,i], color=colors[i], ls='solid', label=labels[i])
        plt.plot(times, target_states[:,i], color=colors[i], ls='dotted')
    plt.legend()
    plt.show()
    
def shutdown():
    rospy.loginfo("Stopping TurtleBot")
    cmd_vel.publish(Twist())
    rospy.sleep(1)
 
if __name__ == '__main__':
    # try:
    #     main()
    # except Exception as e:
    #     print(e)
    #     rospy.loginfo("Lab3 node terminated.")
    main()
