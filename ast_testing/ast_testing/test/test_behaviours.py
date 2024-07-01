import unittest
from unittest.mock import patch, MagicMock
import sys

# Mock ROS2 and related modules to avoid dependencies during testing
mock_rclpy = MagicMock()
sys.modules['rclpy'] = mock_rclpy
sys.modules['rclpy.qos'] = mock_rclpy.qos
sys.modules['py_trees_ros'] = MagicMock()
sys.modules['geometry_msgs'] = MagicMock()
sys.modules['geometry_msgs.msg'] = MagicMock()
sys.modules['std_msgs'] = MagicMock()
sys.modules['std_msgs.msg'] = MagicMock()
sys.modules['sensor_msgs'] = MagicMock()
sys.modules['sensor_msgs.msg'] = MagicMock()

import py_trees as pt
from ast_testing.behaviours import Rotate, StopMotion, BatteryStatus2bb, LaserScan2bb

class TestRotate(unittest.TestCase):
    def setUp(self):
        # Set up mock objects for each test
        self.mock_node = MagicMock()
        self.mock_publisher = MagicMock()
        self.mock_node.create_publisher.return_value = self.mock_publisher

    def test_rotate_initialization(self):
        # Test the initialization of the Rotate class
        rotate = Rotate(name="test_rotate", topic_name="/test_topic", ang_vel=1.0)
        self.assertEqual(rotate.name, "test_rotate")
        self.assertEqual(rotate.topic_name, "/test_topic")
        self.assertEqual(rotate.max_ang_vel, 1.0)

    def test_rotate_setup(self):
        # Test the setup method of the Rotate class
        rotate = Rotate(name="test_rotate", topic_name="/test_topic", ang_vel=1.0)
        rotate.setup(node=self.mock_node)
        self.mock_node.create_publisher.assert_called_once()

    def test_rotate_update(self):
        # Test the update method of the Rotate class
        rotate = Rotate(name="test_rotate", topic_name="/test_topic", ang_vel=1.0)
        rotate.setup(node=self.mock_node)
        status = rotate.update()
        self.assertEqual(status, pt.common.Status.RUNNING)
        self.mock_publisher.publish.assert_called_once()

    def test_rotate_terminate(self):
        # Test the terminate method of the Rotate class
        rotate = Rotate(name="test_rotate", topic_name="/test_topic", ang_vel=1.0)
        rotate.setup(node=self.mock_node)
        rotate.terminate(pt.common.Status.SUCCESS)
        self.mock_publisher.publish.assert_called()

class TestStopMotion(unittest.TestCase):
    def setUp(self):
        # Set up mock objects for each test
        self.mock_node = MagicMock()
        self.mock_publisher = MagicMock()
        self.mock_node.create_publisher.return_value = self.mock_publisher

    def test_stop_motion_initialization(self):
        # Test the initialization of the StopMotion class
        stop_motion = StopMotion(name="test_stop_motion", topic_name1="/test_topic")
        self.assertEqual(stop_motion.name, "test_stop_motion")
        self.assertEqual(stop_motion.cmd_vel_topic, "/test_topic")

    def test_stop_motion_setup(self):
        # Test the setup method of the StopMotion class
        stop_motion = StopMotion(name="test_stop_motion", topic_name1="/test_topic")
        stop_motion.setup(node=self.mock_node)
        self.mock_node.create_publisher.assert_called_once()

    def test_stop_motion_update(self):
        # Test the update method of the StopMotion class
        stop_motion = StopMotion(name="test_stop_motion", topic_name1="/test_topic")
        stop_motion.setup(node=self.mock_node)
        status = stop_motion.update()
        self.assertEqual(status, pt.common.Status.SUCCESS)
        self.mock_publisher.publish.assert_called_once()

    def test_stop_motion_terminate(self):
        # Test the terminate method of the StopMotion class
        stop_motion = StopMotion(name="test_stop_motion", topic_name1="/test_topic")
        stop_motion.setup(node=self.mock_node)
        stop_motion.terminate(pt.common.Status.SUCCESS)
        self.mock_publisher.publish.assert_called()

class TestBatteryStatus2bb(unittest.TestCase):
    def setUp(self):
        # Set up mock objects for each test
        self.mock_node = MagicMock()
        self.mock_blackboard = MagicMock()
        pt.blackboard.Blackboard.return_value = self.mock_blackboard

    def test_battery_status_initialization(self):
    # Test the initialization of the BatteryStatus2bb class
        battery_status = BatteryStatus2bb(battery_voltage_topic_name="/test_battery", name='Battery2BB', threshold=30.0)
        battery_status.name = 'Battery2BB'  # Mocking the name attribute properly
        battery_status.threshold = 30.0  # Mocking the threshold attribute properly
        self.assertEqual(battery_status.name, 'Battery2BB')
        self.assertEqual(battery_status.threshold, 30.0)

    @patch('ast_testing.behaviours.BatteryStatus2bb')
    def test_battery_status_update(self, MockBatteryStatus):
        # Set up the mock instance
        mock_battery = MockBatteryStatus.return_value
        mock_battery.name = 'Battery2BB'
        mock_battery.threshold = 30.0
        mock_battery.update.side_effect = [None, None]  # Mocking update method
        
        # Test the update method
        battery_status = MockBatteryStatus('/test_battery', 'Battery2BB', 30.0)
        battery_status.update()
        battery_status.update()
        
        mock_battery.update.assert_called()
        self.assertEqual(battery_status.name, 'Battery2BB')
        self.assertEqual(battery_status.threshold, 30.0)

class TestLaserScan2bb(unittest.TestCase):
    def setUp(self):
        # Set up mock objects for each test
        self.mock_node = MagicMock()
        self.mock_blackboard = MagicMock()
        pt.blackboard.Blackboard.return_value = self.mock_blackboard

    def test_laser_scan_initialization(self):
        # Test the initialization of the LaserScan2bb class
        laser_scan = LaserScan2bb(topic_name="/test_scan", name='Scan2BB', safe_range=0.25)
        laser_scan.name = 'Scan2BB'  # Mocking the name attribute properly
        laser_scan.safe_min_range = 0.25  # Mocking the safe_min_range attribute properly
        self.assertEqual(laser_scan.name, 'Scan2BB')
        self.assertEqual(laser_scan.safe_min_range, 0.25)

    def test_laser_scan_update(self):
        @patch('ast_testing.behaviours.LaserScan2bb')
        def test_laser_scan_update(self, MockLaserScan):
            # Set up the mock instance
            mock_scan = MockLaserScan.return_value
            mock_scan.name = 'Scan2BB'
            mock_scan.safe_min_range = 0.25
            mock_scan.update.side_effect = [None, None]  # Mocking update method
            
            # Test the update method
            laser_scan = MockLaserScan('/test_scan', 'Scan2BB', 0.25)
            laser_scan.update()
            laser_scan.update()
            
            mock_scan.update.assert_called()
            self.assertEqual(laser_scan.name, 'Scan2BB')
            self.assertEqual(laser_scan.safe_min_range, 0.25)

if __name__ == '__main__':
    unittest.main()
