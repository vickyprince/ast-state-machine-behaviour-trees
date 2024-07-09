import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import csv
from datetime import datetime

class OdomDataCollector(Node):
    def __init__(self):
        super().__init__('odom_data_collector')
        
        # Create subscribers for each robot's odometry
        self.odom_subs = [
            self.create_subscription(Odometry, '/odom', self.odom_callback(1), 10),
            self.create_subscription(Odometry, '/robile2/odom', self.odom_callback(2), 10),
            self.create_subscription(Odometry, '/robile3/odom', self.odom_callback(3), 10)
        ]
        
        # Open CSV file for writing
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.csv_filename = f'robot_y_positions_{timestamp}.csv'
        self.csv_file = open(self.csv_filename, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        
        # Write header
        self.csv_writer.writerow(['Timestamp', 'Robot1_Y', 'Robot2_Y', 'Robot3_Y'])
        
        # Initialize position data
        self.positions = {1: 0, 2: 0, 3: 0}
        
        # Create a timer to write data periodically
        self.timer = self.create_timer(1.0, self.write_data)  # Write data every 1 second
        
        self.get_logger().info(f"Odometry Y data collection started. Saving to {self.csv_filename}")

    def odom_callback(self, robot_id):
        def callback(msg):
            y = msg.pose.pose.position.y
            self.positions[robot_id] = y
        return callback

    def write_data(self):
        timestamp = self.get_clock().now().to_msg()
        row = [
            timestamp,
            self.positions[1],
            self.positions[2],
            self.positions[3]
        ]
        self.csv_writer.writerow(row)
        self.csv_file.flush()  # Ensure data is written to file

    def __del__(self):
        if hasattr(self, 'csv_file'):
            self.csv_file.close()
            self.get_logger().info(f"Odometry Y data collection completed. Data saved to {self.csv_filename}")

def main(args=None):
    rclpy.init(args=args)
    odom_collector = OdomDataCollector()
    try:
        rclpy.spin(odom_collector)
    except KeyboardInterrupt:
        pass
    finally:
        odom_collector.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
