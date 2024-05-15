import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import time
import csv

class MyNode(Node):
    def __init__(self):
        super().__init__('mynode')
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.initial_odom = None
        self.distance_traveled = 0.0
        self.start_time = None

    def odom_callback(self, msg):
        if self.initial_odom is None:
            self.initial_odom = msg

        if self.start_time is None:
            # Convert header timestamp to Unix time (seconds since epoch)
            self.start_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        self.distance_traveled = math.sqrt(
            (msg.pose.pose.position.x - self.initial_odom.pose.pose.position.x) ** 2 +
            (msg.pose.pose.position.y - self.initial_odom.pose.pose.position.y) ** 2
        )

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()

    # Initialize variables for storing data
    data = []

    while rclpy.ok():
        distance = node.distance_traveled
        current_time = time.time()  # Get current time in Unix time
        cmd_vel_msg = Twist()
        if distance < 0.5:
            cmd_vel_msg.linear.x = 0.10
            print('Distance:', distance, 'Time:', current_time)
        else:
            print('Distance:', distance, 'Time:', current_time)
            cmd_vel_msg.linear.x = 0.0
            node.cmd_vel_pub.publish(cmd_vel_msg)
            break
        node.cmd_vel_pub.publish(cmd_vel_msg)
        # Append data to list
        data.append([distance, current_time, cmd_vel_msg.linear.x])
        rclpy.spin_once(node)

    # Save data to CSV file
    with open('data.csv', 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(['Distance', 'Time', 'Velocity'])
        writer.writerows(data)

if __name__ == '__main__':
    main()
