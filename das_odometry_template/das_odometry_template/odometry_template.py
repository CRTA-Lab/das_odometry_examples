import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry

class JointStateToOdom(Node):

    def __init__(self):
        super().__init__('jointstate_to_odom')

        # Subscriber
        self.sub_ = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10)

        # Publisher
        self.pub_ = self.create_publisher(Odometry, '/das/odometry', 10)

    def joint_state_callback(self, msg: JointState):
        odom_msg = Odometry()
        odom_msg.header.frame_id="odom"
        odom_msg.child_frame_id="robot_base"
        
        # Write odometry calculations here
        
        self.pub_.publish(odom_msg)


def main(args=None):
    rclpy.init(args=args)
    node = JointStateToOdom()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
