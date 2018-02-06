import rospy
import baxter_interface


left_arm = baxter_interface.limb.Limb("left")
joints = left_arm.joint_names()

start = rospy.Time.now()

while (rospy.Time.now() - start < 5):
	left_arm.set_joint_velocities((joints[0], 1))