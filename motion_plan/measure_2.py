import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

class GoalPosePublisher(Node):

    def __init__(self):
        super().__init__('goal_pose_publisher')
        self.publisher_ = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.timer = self.create_timer(1.0, self.publish_goal_pose)  # Publish every 1 second

    def publish_goal_pose(self):
        goal_pose_msg = PoseStamped()
        goal_pose_msg.header.frame_id = 'map'  # Specify the frame ID of the goal pose
        goal_pose_msg.pose.position.x = 1.0  # Set the target position (x)
        goal_pose_msg.pose.position.y = 0.0  # Set the target position (y)
        goal_pose_msg.pose.position.z = 0.0  # Set the target position (z)
        goal_pose_msg.pose.orientation.w = 1.0  # Set the target orientation (no rotation)

        self.publisher_.publish(goal_pose_msg)
        self.get_logger().info('Published goal pose: {}'.format(goal_pose_msg))

def main(args=None):
    rclpy.init(args=args)

    goal_pose_publisher = GoalPosePublisher()

    rclpy.spin_once(goal_pose_publisher)

    goal_pose_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
