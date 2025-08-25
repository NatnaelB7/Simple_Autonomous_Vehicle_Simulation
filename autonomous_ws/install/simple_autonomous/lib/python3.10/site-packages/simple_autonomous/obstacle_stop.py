import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist   # use Twist instead of String

class ObstacleStop(Node):
    def __init__(self):
        super().__init__('obstacle_stop')
        self.subscriber = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            10)
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.get_logger().info("ObstacleStop node started")

    def lidar_callback(self, msg: LaserScan):
        min_dist = min(msg.ranges)
        cmd = Twist()
        if min_dist < 0.2:   # stop if obstacle closer than 0.2 m
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.get_logger().info(f"Min dist: {min_dist:.2f} m → STOP")
        else:
            cmd.linear.x = 0.2   # move forward
            cmd.angular.z = 0.0
            self.get_logger().info(f"Min dist: {min_dist:.2f} m → GO")
        self.publisher.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleStop()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


