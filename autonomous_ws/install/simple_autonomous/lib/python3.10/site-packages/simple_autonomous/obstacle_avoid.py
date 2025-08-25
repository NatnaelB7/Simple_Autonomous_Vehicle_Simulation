import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class ObstacleAvoid(Node):
    def __init__(self):
        super().__init__('obstacle_avoid')
        self.subscriber = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            10)
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.get_logger().info("ObstacleAvoid node started")

    def lidar_callback(self, msg: LaserScan):
        cmd = Twist()

        # Divide scan into regions (front, left, right)
        front = min(min(msg.ranges[0:15] + msg.ranges[-15:]), 10.0)
        left = min(min(msg.ranges[75:105]), 10.0)
        right = min(min(msg.ranges[255:285]), 10.0)

        if front > 0.5:   # clear ahead
            cmd.linear.x = 0.2
            cmd.angular.z = 0.0
            self.get_logger().info("GO forward")
        else:
            if left > right:
                cmd.linear.x = 0.0
                cmd.angular.z = 0.5
                self.get_logger().info("Obstacle ahead → Turn LEFT")
            else:
                cmd.linear.x = 0.0
                cmd.angular.z = -0.5
                self.get_logger().info("Obstacle ahead → Turn RIGHT")

        self.publisher.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoid()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


