import rclpy
import smach
import smach_ros

from std_msgs.msg import String
from std_msgs.msg import Float32
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import yaml
import time
import numpy as np

# Reference: https://wiki.ros.org/smach/Tutorials/Simple%2 State% Machine [but in ROS1]


class MonitorBatteryAndCollision(smach.State):
    """State to monitor the battery level and possible collisions
    """

    def __init__(self, node):
        smach.State.__init__(self, outcomes=['low_battery', 'normal', 'collision'])
        self.node = node
        self.battery_voltage = 100.0
        self.battery_threshold = 25.0
        self.min_distance_threshold = 0.5
        self.min_distance = 10.0
        self.node.subscription1 = self.node.create_subscription(
            Float32,
            '/battery_voltage',
            self.battery_callback,
            10
        )
        self.node.subscription2 = self.node.create_subscription(
            LaserScan,
            '/scan',
            self.collision_callback,
            10
        )
    def collision_callback(self, msg):
        laser_data = np.array(msg.ranges)
        laser_data[laser_data <= 0.05] = 1.0
        laser_data = np.nan_to_num(laser_data, nan=40)
        laser_data[np.isinf(laser_data)]=50
        # print(laser_data)
        self.min_distance = min(laser_data)
        # print(self.min_distance)
        

      
    def battery_callback(self, msg):
        self.battery_voltage = msg.data
        self.node.get_logger().info(f"Received battery voltage: {self.battery_voltage}")

    def execute(self, userdata):
        while True:
            if self.battery_voltage < self.battery_threshold:
                return 'low_battery'
            elif self.min_distance < self.min_distance_threshold:
                return 'collision'
            else:
                return 'normal'
            
class RotateBase(smach.State):
    """State to rotate the Robile base
    """
    def __init__(self, node):
        smach.State.__init__(self, outcomes=['low_battery'])
        self.node = node
        self.node.cmd_vel_pub = self.node.create_publisher(Twist, '/cmd_vel', 10)

    def execute(self, userdata):
        twist1 = Twist()
        twist1.angular.z = 0.5
        self.node.cmd_vel_pub.publish(twist1)
        return 'low_battery'
    
class StopMotion(smach.State):
    """State to stop the robot's motion
    """
    def __init__(self, node):
        smach.State.__init__(self, outcomes=['collision'])
        self.node = node
        self.node.cmd_vel_pub1 = self.node.create_publisher(Twist, '/cmd_vel', 10)

    def execute(self, userdata):
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.angular.z = 0.0
        self.node.cmd_vel_pub1.publish(twist)
        return 'collision'
    
class Control(smach.State):
    """State to control the robot's motion
    """
    def __init__(self, node):
        smach.State.__init__(self, outcomes=['normal'])
        self.node = node
        self.node.cmd_vel_pub2 = self.node.create_publisher(Twist, '/cmd_vel', 10)

    def cmd_vel_callback(self, msg):
        self.node.get_logger().info(f"Received cmd_vel: {msg}")

    def execute(self, userdata):
        twist = Twist()
        twist.linear.x = 0.35
        twist.linear.y = 0.0
        twist.angular.z = 0.0
        self.node.cmd_vel_pub2.publish(twist)
        return 'normal'


def main(args=None):
    """Main function to initialise and execute the state machine
    """

    rclpy.init(args=args)
    node = rclpy.create_node('robile')
    sm = smach.StateMachine(outcomes=['finished'])
    with sm:
        # Add states to the state machine
        smach.StateMachine.add('MONITOR_BATTERY_Collision', MonitorBatteryAndCollision(node),
                                transitions={'low_battery': 'ROTATE_BASE',
                                              'collision': 'STOP_MOTION',
                                              'normal': 'CONTROL'})
        smach.StateMachine.add('ROTATE_BASE', RotateBase(node),
                                transitions={'low_battery': 'finished'})
        smach.StateMachine.add('STOP_MOTION', StopMotion(node),
                                transitions={'collision': 'finished'})
        smach.StateMachine.add('CONTROL', Control(node),
                                transitions={'normal': 'finished'})

    # Execute the state machine
    while rclpy.ok():
        outcome = sm.execute()
        print(outcome)
        rclpy.spin_once(node)
        if outcome == 'error':
            break

 
    rclpy.shutdown()

if __name__ == "__main__":
    main()


