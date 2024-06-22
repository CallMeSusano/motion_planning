import os
import rclpy
import math
import subprocess

from rclpy.qos import QoSProfile
from nav_msgs.msg import Odometry

import sys
# importing sys
# adding maps to the system path
sys.path.insert(0, '/home/miguel/Desktop/maps')
from point_map import go_home
 

def quaternion_to_euler_angle(quat):
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    quat: geometry_msgs.msg.Quaternion
    """
    x, y, z, w = quat.x, quat.y, quat.z, quat.w
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)

    return roll_x, pitch_y, yaw_z

def odom_callback(msg):
    global current_yaw, data_available, current_x, current_y
    _, _, current_yaw = quaternion_to_euler_angle(msg.pose.pose.orientation)
    current_x = msg.pose.pose.position.x
    current_y = msg.pose.pose.position.y
    data_available = True

def main():
    global current_yaw, data_available, current_x, current_y, initial_x, initial_y
    data_available = False
    current_x = 0.0
    current_y = 0.0
    initial_x = None
    initial_y = None

    rclpy.init()

    qos = QoSProfile(depth=10)
    node = rclpy.create_node('measure_rotation')
    node.create_subscription(Odometry, 'odom', odom_callback, qos)
    
    try:
        print("Waiting for initial position...")
        while not data_available:
            rclpy.spin_once(node, timeout_sec=0.1)  # Allow the callback to be processed

        initial_x = current_x
        initial_y = current_y
        initial_yaw = current_yaw
        print(f"Initial position: x={initial_x}, y={initial_y}, yaw={math.degrees(initial_yaw)}")

        input("Press 'h' to get the current position and angle...")

        data_available = False  # Reset flag
        while not data_available:
            rclpy.spin_once(node, timeout_sec=0.1)  # Allow the callback to be processed

        angle = math.degrees(current_yaw)
        delta_x = initial_x - current_x
        delta_y = initial_y - current_y
        print(f"Current position: x={current_x}, y={current_y}, yaw={angle}")
        print(f"Difference from initial position: Δx={-delta_x}, Δy={-delta_y}")

    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

        # Save the map using the command
        map_save_command = 'ros2 run nav2_map_server map_saver_cli -f ~/maps'
        subprocess.run(map_save_command, shell=True)
        print("DoNE")
        #go_home((1, 0.0), angle)
        go_home((-delta_x, delta_y), angle)

if __name__ == '__main__':
    main()
