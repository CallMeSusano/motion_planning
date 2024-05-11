import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from tf2_ros import TransformListener, Buffer
import math
import time
import csv

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

    # Initialize variables for storing data
    data = []
    start_time = time.time()

    while rclpy.ok():
        pose = node.get_robot_pose()
        if pose is not None:
            if node.initial_pose is None:
                node.initial_pose = pose
                cmd_vel_msg = Twist()
                cmd_vel_msg.linear.x = 0.15
                node.cmd_vel_pub.publish(cmd_vel_msg)
            distance = node.calculate_distance(node.initial_pose, pose)
            # Get current time
            current_time = time.time()
            if distance < 0.5:
                print('Distance:', distance, 'Time:', current_time)
            else:
                print('Distance:', distance, 'Time:', current_time)
                cmd_vel_msg = Twist()
                cmd_vel_msg.linear.x = 0.0
                node.cmd_vel_pub.publish(cmd_vel_msg)
                break
            # Append data to list
            time_elapsed = current_time - start_time
            data.append([distance, time_elapsed, distance/time_elapsed])
        rclpy.spin_once(node)

    # Save data to CSV file
    with open('data.csv', 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(['Distance', 'Time', 'Velocity'])
        writer.writerows(data)

if __name__ == '__main__':
    main()
