import os
import rclpy
import math
import time
import csv

from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile
from nav_msgs.msg import Odometry

if os.name == 'nt':
    import msvcrt
else:
    import termios
    import tty

BURGER_MAX_LIN_VEL = 0.15
BURGER_MAX_ANG_VEL = 2.84

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

def normalize_angle(angle):
    """
    Normalize an angle to the range [-pi, pi]
    """
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle

def odomCallback(msg):
    global initial_yaw, start_time, current_yaw, isDataAvailable

    if initial_yaw is None:
        _, _, initial_yaw = quaternion_to_euler_angle(msg.pose.pose.orientation)

    if start_time is None:
        start_time = time.time()

    _, _, current_yaw = quaternion_to_euler_angle(msg.pose.pose.orientation)
    isDataAvailable = True

def main():
    global current_yaw, isDataAvailable, start_time, initial_yaw
    
    for counter in range(1, 3):
        data = []
        initial_yaw = None
        current_yaw = 0.0
        start_time = None
        isDataAvailable = False
        if counter % 2 != 0:
            ang_vel = 0.5
        else:
            ang_vel = -0.5
        rclpy.init()

        qos = QoSProfile(depth=10)
        node = rclpy.create_node('measure_rotation')
        velocity = node.create_publisher(Twist, 'cmd_vel', qos)
        position = node.create_subscription(Odometry, 'odom', odomCallback, qos)
        velValue = Twist()
        try:
            print("test")
            velValue.angular.z = ang_vel
            isTestDone = False
            initialTime = time.time()
            while(True):
                rclpy.spin_once(node, timeout_sec=0.1)  # Allow the callback to be processed
                if not isTestDone:
                    while(isDataAvailable):
                        isDataAvailable = False
                        angle_rotated = normalize_angle(current_yaw - initial_yaw)
                        print("Angle rotated: ", math.degrees(angle_rotated))
                        if abs(angle_rotated) >= math.radians(179):  # 180 degrees in radians
                            velValue.angular.z = 0.0
                            velocity.publish(velValue)
                            isTestDone = True
                            current_time = time.time() - initialTime  # Get current time in Unix time
                            data.append([math.degrees(angle_rotated), current_time, math.degrees(angle_rotated) / current_time])
                            break
                        velocity.publish(velValue)
                        current_time = time.time() - initialTime  # Get current time in Unix time
                        data.append([math.degrees(angle_rotated), current_time, math.degrees(angle_rotated) / current_time])
                        break
                else:
                    break
                
        except Exception as e:
            print(e)

        finally:
            twist = Twist()
            twist.linear.x = 0.0
            twist.linear.y = 0.0
            twist.linear.z = 0.0

            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = 0.0

            velocity.publish(twist)
            node.destroy_node()
            rclpy.shutdown()

        # Save data to CSV file
        filename = f'data_rotation{counter}.csv'
        with open(filename, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(['Angle', 'Time', 'Angular Velocity'])
            writer.writerows(data)

if __name__ == '__main__':
    main()
