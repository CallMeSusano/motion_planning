import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from tf2_ros import TransformListener, Buffer
import math

class MyNode(Node):
    def __init__(self):
        super().__init__('mynode')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.initial_pose = None

    def get_robot_pose(self):
        try:
            trans = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            return (trans.transform.translation.x, trans.transform.translation.y, trans.transform.rotation.z, trans.transform.rotation.w)
        except Exception as e:
            self.get_logger().info('Could not get transform: %s' % e)
            return None

    def calculate_distance(self, pose1, pose2):
        return math.sqrt((pose2[0] - pose1[0])**2 + (pose2[1] - pose1[1])**2)

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    while rclpy.ok():
        pose = node.get_robot_pose()
        if pose is not None:
            #print('Robot pose in map frame: ', pose)
            if node.initial_pose is None:
                node.initial_pose = pose
            distance = node.calculate_distance(node.initial_pose, pose)
            cmd_vel_msg = Twist()
            if distance < 2.0:
                cmd_vel_msg.linear.x = 0.10
                print('Distance: ', distance)
            else:
                print('Distance: ', distance)
                cmd_vel_msg.linear.x = 0.0
                node.cmd_vel_pub.publish(cmd_vel_msg)
                break
            node.cmd_vel_pub.publish(cmd_vel_msg)
        rclpy.spin_once(node)

if __name__ == '__main__':
    main()