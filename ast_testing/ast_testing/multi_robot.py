import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
import sys

class RobotController(Node):
    def __init__(self, robot_id):
        super().__init__(f'robot_controller_{robot_id}')
        self.robot_id = robot_id
        self.is_leader = (robot_id == 1)

        if self.is_leader:
            # For the leader (robot1)
            self.new_cmd_vel_pub = self.create_publisher(Twist, '/robot1/new_cmd_vel', 10)
            self.cmd_vel_sub = self.create_subscription(
                Twist,
                'cmd_vel',
                self.leader_cmd_vel_callback,
                10
            )
        else:
            # For followers
            # Get the pre-existing cmd_vel topic from a parameter
            self.follower_cmd_vel_topic = self.declare_parameter('cmd_vel_topic', f'/robile{robot_id}/cmd_vel').value
            self.cmd_vel_pub = self.create_publisher(Twist, self.follower_cmd_vel_topic, 10)
            self.cmd_vel_sub = self.create_subscription(
                Twist,
                '/robot1/new_cmd_vel',  # Subscribe to the leader's topic
                self.follower_cmd_vel_callback,
                10
            )

        self.battery_sub = self.create_subscription(
            Float32,
            f'/robot{robot_id}/battery_voltage',
            self.battery_callback,
            10
        )

        self.laser_sub = self.create_subscription(
            LaserScan,
            f'/robot{robot_id}/scan',
            self.laser_callback,
            10
        )

        self.battery_level = 100.0
        self.obstacle_detected = False
        self.safe_range = 0.5

    def leader_cmd_vel_callback(self, msg):
        # Publish to new_cmd_vel topic
        self.new_cmd_vel_pub.publish(msg)
        self.get_logger().info(f"Leader received and republished: Linear: {msg.linear.x}, Angular: {msg.angular.z}")

    def follower_cmd_vel_callback(self, msg):
        if not self.obstacle_detected and self.battery_level > 20.0:
            self.cmd_vel_pub.publish(msg)
            self.get_logger().info(f"Follower {self.robot_id} received and published to {self.follower_cmd_vel_topic}: Linear: {msg.linear.x}, Angular: {msg.angular.z}")
        else:
            if self.obstacle_detected:
                self.get_logger().warn(f"Follower {self.robot_id}: Obstacle detected, not publishing command.")
            if self.battery_level <= 20.0:
                self.get_logger().warn(f"Follower {self.robot_id}: Low battery, not publishing command.")

    def battery_callback(self, msg):
        self.battery_level = msg.data
        if self.battery_level <= 20.0:
            self.get_logger().warn(f'Robot {self.robot_id}: Low battery!')

    def laser_callback(self, msg):
        self.obstacle_detected = any(distance < self.safe_range for distance in msg.ranges)
        if self.obstacle_detected:
            self.get_logger().warn(f'Robot {self.robot_id}: Obstacle detected!')
            self.stop_robot()

    def stop_robot(self):
        stop_msg = Twist()
        if self.is_leader:
            self.new_cmd_vel_pub.publish(stop_msg)
        else:
            self.cmd_vel_pub.publish(stop_msg)
        self.get_logger().info(f"Robot {self.robot_id}: Stopping")

def main(args=None):
    print("Starting Robot Controller")
    rclpy.init(args=args)
    
    # Get robot_id from ROS 2 parameter
    node = rclpy.create_node('robot_id_param_node')
    robot_id = node.declare_parameter('robot_id', 1).value
    node.destroy_node()
    print(f'Robot ID: {robot_id}')

    controller = RobotController(robot_id)
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
