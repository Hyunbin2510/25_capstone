import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Twist

class TurtlebotFollower(Node):
    def __init__(self):
        super().__init__('turtlebot_follower')
        self.subscription = self.create_subscription(
            Point,
            'person_tracking',
            self.tracking_callback,
            10
        )
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # PID 제어 하이퍼파라미터
        self.kp_angular = 1.0
        self.ki_angular = 0.01
        self.kd_angular = 0.1
        self.kp_linear = 0.5
        self.ki_linear = 0.01
        self.kd_linear = 0.1
        
        self.desired_bbox_width = 0.2
        self.distance_tolerance = 0.05
        
        self.previous_error_angular = 0.0
        self.previous_error_linear = 0.0
        self.integral_angular = 0.0
        self.integral_linear = 0.0

    def pid_control(self, error, prev_error, integral, kp, ki, kd):
        integral += error
        derivative = error - prev_error
        output = (kp * error) + (ki * integral) + (kd * derivative)
        return output, error, integral

    def tracking_callback(self, msg):
        error_angular = msg.x - 0.5
        angular_z, self.previous_error_angular, self.integral_angular = self.pid_control(
            error_angular, self.previous_error_angular, self.integral_angular,
            self.kp_angular, self.ki_angular, self.kd_angular
        )

        error_linear = self.desired_bbox_width - msg.z
        linear_x, self.previous_error_linear, self.integral_linear = self.pid_control(
            error_linear, self.previous_error_linear, self.integral_linear,
            self.kp_linear, self.ki_linear, self.kd_linear
        )

        twist = Twist()
        twist.linear.x = linear_x
        twist.angular.z = -angular_z

        self.cmd_pub.publish(twist)
        self.get_logger().info(f'cmd_vel: linear.x={linear_x:.2f}, angular.z={angular_z:.2f}')

def main(args=None):
    rclpy.init(args=args)
    node = TurtlebotFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()