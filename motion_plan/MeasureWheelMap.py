import os
import select
import sys
import rclpy
import math
import time
import csv

from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile
from nav_msgs.msg import Odometry
from tf2_ros import TransformListener, Buffer
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

if os.name == 'nt':
    import msvcrt
else:
    import termios
    import tty

BURGER_MAX_LIN_VEL = 0.15
BURGER_MAX_ANG_VEL = 2.84

initial_odom = None
distance_traveled = 0.0
start_time = None
isDataAvailable = False
odom = None

def odomCallback(msg):
    global initial_odom, start_time, distance_traveled, isDataAvailable, odom

    if initial_odom is None:
        initial_odom = msg

    if start_time is None:
        # Convert header timestamp to Unix time (seconds since epoch)
        start_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    distance_traveled = math.sqrt(
        (msg.pose.pose.position.x - initial_odom.pose.pose.position.x) ** 2 +
        (msg.pose.pose.position.y - initial_odom.pose.pose.position.y) ** 2
    )
    isDataAvailable = True
    odom = msg
        
def get_robot_pose(tf_buffer, node):
    try:
        now = rclpy.time.Time()
        trans = tf_buffer.lookup_transform('map', 'base_link', now)
        return (
            trans.transform.translation.x,
            trans.transform.translation.y,
            trans.transform.rotation.z,
            trans.transform.rotation.w
        )
    except (LookupException, ConnectivityException, ExtrapolationException) as e:
        #node.get_logger().error(f'Could not get transform from "map" to "base_link": {e}')
        return None

def main():
    global distance_traveled, isDataAvailable, start_time, initial_odom, odom
    
    for counter in range(1, 21):
        data = []
        dataInfo = []
        initial_odom = None
        distance_traveled = 0.0
        start_time = None
        isDataAvailable = False
        if counter % 2 != 0:
            vel = 0.10
        else:
            vel = -0.10
        rclpy.init()

        qos = QoSProfile(depth=10)
        node = rclpy.create_node('measure')
        velocity = node.create_publisher(Twist, 'cmd_vel', qos)
        position = node.create_subscription(Odometry, 'odom', odomCallback, qos)
        tf_buffer = Buffer()
        tf_listener = TransformListener(tf_buffer, node)
        velValue = Twist()
        try:
            velValue.linear.x = vel
            isTestDone = False
            initialTime = time.time()
            isTestStarting = True

            # Wait for the transform to be available
            initialMapPosition = None

            print("Waiting for map...")
            while initialMapPosition is None:
                initialMapPosition = get_robot_pose(tf_buffer, node)
                rclpy.spin_once(node, timeout_sec=0.1)
            print("Done!")

            while True:
                rclpy.spin_once(node, timeout_sec=0.1)  # Allow the callback to be processed
                if not isTestDone:
                    while isDataAvailable:
                        isDataAvailable = False
                        if distance_traveled >= 2:
                            velValue.linear.x = 0.0
                            velocity.publish(velValue)
                            isTestDone = True
                            current_time = time.time() - initialTime  # Get current time in Unix time
                            print("Position: ", abs(odom.pose.pose.position.x - initialOdomPositionX), "    ", abs(odom.pose.pose.position.y - initialOdomPositionY))
                            finalPosition = get_robot_pose(tf_buffer, node)
                            print("Map Position: ", abs(finalPosition[0] - initialMapPosition[0]), abs(finalPosition[1] - initialMapPosition[1]))
                            goalOffsetX = abs(finalPosition[0] - initialMapPosition[0]) - abs(odom.pose.pose.position.x - initialOdomPositionX)
                            goalOffsetY = abs(finalPosition[1] - initialMapPosition[1]) - abs(odom.pose.pose.position.y - initialOdomPositionY)
                            data.append([distance_traveled, current_time, distance_traveled / current_time])
                            dataInfo.append([distance_traveled, current_time, distance_traveled / current_time, goalOffsetX, goalOffsetY])
                            break

                        if isTestStarting:
                            initialOdomPositionX = odom.pose.pose.position.x
                            initialOdomPositionY = odom.pose.pose.position.y
                            isTestStarting = False
                    
                        velocity.publish(velValue)
                        current_time = time.time() - initialTime  # Get current time in Unix time
                        data.append([distance_traveled, current_time, distance_traveled / current_time])
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
        filename = f'data{counter}.csv'
        with open(filename, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(['Distance', 'Time', 'Velocity'])
            writer.writerows(data)
        
        # Save data to CSV file
        filename = f'data{counter}Info.csv'
        with open(filename, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(['Distance', 'Time', 'Velocity', 'OffsetX', 'OffsetY'])
            writer.writerows(dataInfo)

if __name__ == '__main__':
    main()
