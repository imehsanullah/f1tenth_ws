import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from ackermann_msgs.msg import AckermannDriveStamped

class TwistToAckermannNode(Node):
    def __init__(self):
        super().__init__('twist_to_ackermann')
        
        # Create a subscriber to the /cmd_vel topic (from teleop keyboard)
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10)
        
        # Create a publisher to the /drive topic (for AckermannDriveStamped messages)
        self.publisher = self.create_publisher(AckermannDriveStamped, '/drive', 10)

        # Define constant speed and steering values for Ackermann messages
        self.speed = 1.0  # Constant speed (m/s)
        self.max_steering_angle = 0.34  # Maximum steering angle (radians)

    def cmd_vel_callback(self, msg):
        # Create AckermannDriveStamped message
        ackermann_msg = AckermannDriveStamped()

        # Set speed (linear.x is forward/backward velocity)
        if msg.linear.x > 0:
            ackermann_msg.drive.speed = self.speed  # Forward
        elif msg.linear.x < 0:
            ackermann_msg.drive.speed = -self.speed  # Reverse
        else:
            ackermann_msg.drive.speed = 0.0  # Stop

        # Set steering (angular.z is left/right turning)
        if msg.angular.z > 0:
            ackermann_msg.drive.steering_angle = self.max_steering_angle  # Turn left
        elif msg.angular.z < 0:
            ackermann_msg.drive.steering_angle = -self.max_steering_angle  # Turn right
        else:
            ackermann_msg.drive.steering_angle = 0.0  # Go straight

        # Publish Ackermann message
        self.publisher.publish(ackermann_msg)

def main(args=None):
    rclpy.init(args=args)
    node = TwistToAckermannNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
