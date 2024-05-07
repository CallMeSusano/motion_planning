import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from tf2_ros import TransformListener, Buffer
import time

class MyNode(Node):
    def __init__(self):
        super().__init__('mynode')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.start_time = None
        self.total_distance = 0.0

    def get_robot_pose(self):
        try:
            trans = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            return (trans.transform.translation.x, trans.transform.translation.y, trans.transform.rotation.z, trans.transform.rotation.w)
        except Exception as e:
            self.get_logger().info('Could not get transform: %s' % e)
            return None

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    while rclpy.ok():
        pose = node.get_robot_pose()
        if pose is not None:
            print('Robot pose in map frame: ', pose)
            cmd_vel_msg = Twist()
            if node.total_distance < 1.0:
                cmd_vel_msg.linear.x = 0.2
                if node.start_time is None:
                    node.start_time = time.time()
                else:
                    elapsed_time = time.time() - node.start_time
                    node.total_distance += cmd_vel_msg.linear.x * elapsed_time
                    node.start_time = time.time()
            else:
                cmd_vel_msg.linear.x = 0.0
                break
            node.cmd_vel_pub.publish(cmd_vel_msg)
        rclpy.spin_once(node)

if __name__ == '__main__':
    main()