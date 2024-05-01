import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class MinimalSubscriber(Node):
    def __init__(self):
        self.last_x = 0
        self.last_z = 0
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',  # Subscribe to the cmd_vel topic
            self.cmd_vel_callback,
            10)
        self.subscription  # prevent unused variable warning

    def cmd_vel_callback(self, msg):
        # Read the linear and angular velocity components
        if self.last_x != msg.linear.x or self.last_z != msg.angular.z:
            linear_x = msg.linear.x
            angular_z = msg.angular.z
            self.last_x = linear_x
            self.last_z = angular_z
            self.get_logger().info(f'Linear Velocity: {linear_x}, Angular Velocity: {angular_z}')

def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
