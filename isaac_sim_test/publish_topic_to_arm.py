import rospy
from sensor_msgs.msg import JointState

rospy.init_node('franka_joint_command_publisher')
pub = rospy.Publisher('/franka/joint_states', JointState, queue_size=10)

js = JointState()
js.name = ['panda_joint1', 'panda_joint2', 'panda_joint3', 'panda_joint4', 'panda_joint5', 'panda_joint6', 'panda_joint7']

rate = rospy.Rate(100)  # 100 Hz for near real-time
while not rospy.is_shutdown():
    # Replace with your custom control logic
    js.position = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7]
    pub.publish(js)
    rate.sleep()