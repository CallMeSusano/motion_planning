import rclpy
from geometry_msgs.msg import Twist
import csv
import time

def publish_movement(node, publisher, movement_type, duration, direction=None):
    twist = Twist()
    if movement_type == "rotation":
        if direction == "left":
            twist.angular.z = 0.5  # Adjust angular velocity as needed
        elif direction == "right":
            twist.angular.z = -0.5  # Adjust angular velocity as needed
    elif movement_type == "forward":
        twist.linear.x = 0.1  # Adjust linear velocity as needed

    # Start publishing the command
    publisher.publish(twist)
    node.get_logger().info(f"Publishing {movement_type} movement for {duration} seconds")
    time.sleep(duration)
    
    # Stop the movement after the duration
    twist = Twist()  # Zero velocities to stop the robot
    publisher.publish(twist)
    node.get_logger().info(f"Stopped {movement_type} movement")

def main(args=None):
    rclpy.init(args=args)
    
    node = rclpy.create_node('movement_publisher')
    publisher = node.create_publisher(Twist, 'cmd_vel', 10)
    node.get_logger().info("Movement Publisher Node has been started")

    # Read the CSV file
    csv_file_path = 'path_instructions.csv'  # Replace with the actual path to your CSV file
    with open(csv_file_path, newline='') as csvfile:
        reader = csv.DictReader(csvfile)
        for row in reader:
            movement_type = row['type']
            duration = float(row['time'])
            direction = row['move'] if row['move'] != 'NULL' else None

            publish_movement(node, publisher, movement_type, duration, direction)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
