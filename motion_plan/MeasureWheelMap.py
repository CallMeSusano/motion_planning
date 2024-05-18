import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
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
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.initial_odom = None
        self.distance_traveled = 0.0
        self.start_time = None
        self.initial_pose = None

    def odom_callback(self, msg):
        if self.initial_odom is None:
            self.initial_odom = msg

        if self.start_time is None:
            # Convert header timestamp to Unix time (seconds since epoch)
            self.start_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        if self.initial_pose is None:
            self.initial_pose = self.get_robot_pose('map')  # Store the initial pose

        self.distance_traveled = math.sqrt(
            (msg.pose.pose.position.x - self.initial_odom.pose.pose.position.x) ** 2 +
            (msg.pose.pose.position.y - self.initial_odom.pose.pose.position.y) ** 2
        )
    
    def get_robot_pose(self, frame_id):
        try:
            trans = self.tf_buffer.lookup_transform(frame_id, 'base_link', rclpy.time.Time())
            return (trans.transform.translation.x, trans.transform.translation.y, trans.transform.rotation.z, trans.transform.rotation.w)
        except Exception as e:
            self.get_logger().info(f'Could not get transform from {frame_id} to base_link: {e}')
            return (0, 0, 0, 0)

def main(args=None):
    rclpy.init(args=args)
    
    for counter in range(1, 3):
        node = MyNode()

        # Initialize variables for storing data
        data = []

        while rclpy.ok():
            distance = node.distance_traveled
            current_time = time.time()  # Get current time in Unix time
            cmd_vel_msg = Twist()
            if distance < 1.0:
                if counter % 2 != 0:
                    cmd_vel_msg.linear.x = 0.10
                else:
                    cmd_vel_msg.linear.x = -0.10
            else:
                cmd_vel_msg.linear.x = 0.0
                node.cmd_vel_pub.publish(cmd_vel_msg)
                # Append data to list
                offset_x, offset_y = calculate_offset(node.initial_pose, node.get_robot_pose('map'))
                data.append([distance, current_time, cmd_vel_msg.linear.x, offset_x, offset_y])
                break
            node.cmd_vel_pub.publish(cmd_vel_msg)
            # Append data to list
            offset_x, offset_y = calculate_offset(node.initial_pose, node.get_robot_pose('map'))
            if offset_x is None:
                offset_x = 0
            if offset_y is None:
                offset_y = 0
            data.append([distance, current_time, cmd_vel_msg.linear.x, offset_x, offset_y])
            rclpy.spin_once(node)

        # Print final position
        final_pose_map = node.get_robot_pose('map')
        print(f'Final Position (Test {counter}) in Map Frame: {final_pose_map}')

        # Calculate distance from initial point
        if node.initial_pose is not None and final_pose_map is not None:
            initial_x, initial_y, _, _ = node.initial_pose
            final_x, final_y, _, _ = final_pose_map
            distance_x = final_x - initial_x
            distance_y = final_y - initial_y
            distance_from_initial_point = math.sqrt(distance_x**2 + distance_y**2)
            print(f'Distance from Initial Point (Test {counter}): {distance_from_initial_point:.2f} meters')
            print(f'Distance traveled in X direction (Test {counter}): {distance_x:.2f} meters')
            print(f'Distance traveled in Y direction (Test {counter}): {distance_y:.2f} meters')

        # Save data to CSV file
        filename = f'data{counter}.csv'
        with open(filename, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(['Distance', 'Time', 'Velocity', 'Offset_X', 'Offset_Y'])
            writer.writerows(data)

    rclpy.shutdown()

def calculate_offset(initial_pose, current_pose):
    if initial_pose is not None and current_pose is not None:
        initial_x, initial_y, _, _ = initial_pose
        current_x, current_y, _, _ = current_pose
        offset_x = current_x - initial_x
        offset_y = current_y - initial_y
        return offset_x, offset_y
    else:
        return None, None

if __name__ == '__main__':
    main()
