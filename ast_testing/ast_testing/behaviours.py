import rclpy
import py_trees as pt
import py_trees_ros as ptr
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import numpy as np

class Rotate(pt.behaviour.Behaviour):
    """Rotates the robot about the z-axis 
    """
    def __init__(self, name="rotate platform",
                 topic_name="/cmd_vel",
                 ang_vel=1.0):
        self.topic_name = topic_name
        self.max_ang_vel = ang_vel # units: rad/sec
        super(Rotate, self).__init__(name)

    def setup(self, **kwargs):
        """Setting up things which generally might require time to prevent delay in the tree initialisation
        """
        self.logger.info("[ROTATE] setting up rotate behaviour")
        
        try:
            self.node = kwargs['node']
        except KeyError as e:
            error_message = "didn't find 'node' in setup's kwargs [{}][{}]".format(self.qualified_name)
            raise KeyError(error_message) from e 
        
        self.cmd_vel_pub = self.node.create_publisher(
            msg_type=Twist,
            topic=self.topic_name,
            qos_profile=ptr.utilities.qos_profile_latched()
        )

        self.feedback_message = "setup"

        return True

    def update(self):
        """Rotates the robot at the maximum allowed angular velocity.
        Note: The actual behaviour is implemented here.

        """
        self.logger.info("[ROTATE] update: updating rotate behaviour")
        self.logger.debug("%s.update()" % self.__class__.__name__)

        msg = Twist()
        msg.angular.z = 0.5
        """if (self.direction == +1):
        	msg.angular.z = 0.5
        elif (self.direction == -1):
        	msg.angular.z = - 0.5
        """
        self.cmd_vel_pub.publish(msg)
        
        return pt.common.Status.RUNNING

    def terminate(self, new_status):
        """Trigerred once the execution of the behaviour finishes, 
        i.e. when the status changes from RUNNING to SUCCESS or FAILURE
        """
        self.logger.info("[ROTATE] terminate: publishing zero angular velocity")

        twist_msg = Twist()
        twist_msg.linear.x = 0.
        twist_msg.linear.y = 0.
        twist_msg.angular.z = 0.
                    
        self.cmd_vel_pub.publish(twist_msg)
        self.sent_goal = False
        
        return super().terminate(new_status)

class StopMotion(pt.behaviour.Behaviour):
    """Stops the robot when it is controlled using a joystick or with a cmd_vel command
    """
    
    # TODO: Implement a behaviour to stop the robot's motion

    # YOUR CODE HERE
    def __init__(self, 
                 name: str="stop platform", 
                 topic_name1: str="/cmd_vel"):
        super(StopMotion, self).__init__(name)
        # Set up topic name to publish rotation commands
        self.cmd_vel_topic = topic_name1
        
        

    def setup(self, **kwargs):
        """
        Setting up things which generally might require time to prevent delay in tree initialisation
        """
        self.logger.info("[STOP MOTION] setting up stop motion behavior")

        try:
            self.node = kwargs['node']
        except KeyError as e:
            error_message = "didn't find 'node' in setup's kwargs [{}][{}]".format(self.qualified_name)
            raise KeyError(error_message) from e  # 'direct cause' traceability

        # Create publisher to publish rotation commands
        self.cmd_vel_topic = self.node.create_publisher(
            msg_type=Twist,
            topic=self.cmd_vel_topic,
            qos_profile=ptr.utilities.qos_profile_latched()
        )

        self.feedback_message = "setup"
        return True

    def update(self):
        """
        Primary function of the behavior is implemented in this method

        Rotating the robot at maximum allowed angular velocity in a given direction, 
        where if _direction_ is +1, it implies clockwise rotation, and if it is -1, it implies
        counter-clockwise rotation
        """
        self.logger.info("[STOP] update: updating stop behavior")
        self.logger.debug("%s.update()" % self.__class__.__name__)

        """
        Send the zero rotation command to self.cmd_vel_topic
        """
        
        ## YOUR CODE HERE ##
        msg1 = Twist()
        msg1.linear.x = 0.0
        msg1.linear.x = 0.0
        # msg1.angular.z = 0.0
        self.cmd_vel_topic.publish(msg1)

           
        return pt.common.Status.SUCCESS
        # return pt.common.Status.RUNNING


    def terminate(self, new_status):
        """
        terminate() is trigerred once the execution of the behavior finishes, 
        i.e. when the status changes from RUNNING to SUCCESS or FAILURE
        """
        self.logger.info("[ROTATE] terminate: publishing zero angular velocity")
        twist_msg = Twist()
        twist_msg.linear.x = 0.
        twist_msg.linear.y = 0.
        twist_msg.angular.z = 0.

       
                    
        self.cmd_vel_topic.publish(twist_msg)
        self.sent_goal = False
        return super().terminate(new_status)
    


class BatteryStatus2bb(ptr.subscribers.ToBlackboard):
    """Checks the battery status
    """
    def __init__(self, battery_voltage_topic_name: str="/battery_voltage",
                 name: str='Battery2BB',
                 threshold: float=30.0):
        super().__init__(name=name,
                         topic_name=battery_voltage_topic_name,
                         topic_type=Float32,
                         blackboard_variables={'battery': 'data'},
                         initialise_variables={'battery': 100.0},
                         clearing_policy=pt.common.ClearingPolicy.NEVER,  # to decide when data should be cleared/reset.
                         qos_profile=ptr.utilities.qos_profile_unlatched())
        self.blackboard.register_key(key='battery_low_warning', access=pt.common.Access.WRITE)

        # YOUR CODE HERE
        self.blackboard.battery_low_warning = False   # decision making
        self.threshold = threshold
#         raise NotImplementedError()


    def update(self):
        """Calls the parent to write the raw data to the blackboard and then check against the
        threshold to determine if a low warning flag should also be updated.
        """
        self.logger.info('[BATTERY] update: running battery_status2bb update')
        self.logger.debug("%s.update()" % self.__class__.__name__)
        status = super(BatteryStatus2bb, self).update() # not given initially
        
        """check battery voltage level stored in self.blackboard.battery. By comparing with 
        threshold value, update the value of self.blackboad.battery_low_warning
        """

        # TODO: based on the battery voltage level, update the value of self.blackboard.battery_low_warning
        # and return the status of the behaviour based on your logic of the behaviour tree

        # YOUR CODE HERE
        if self.blackboard.battery < self.threshold:
            self.blackboard.battery_low_warning = True
        else:
            self.blackboard.battery_low_warning = False
                
        return pt.common.Status.SUCCESS


class LaserScan2bb(ptr.subscribers.ToBlackboard):
    """Checks the laser scan measurements to avoid possible collisions.
    """
    def __init__(self, topic_name: str="/scan",
                 name: str='Scan2BB',
                 safe_range: float=0.25):
        super().__init__(name=name,
                         topic_name=topic_name,
                         topic_type=LaserScan,
                         blackboard_variables={'laser_scan':'ranges'},
                         clearing_policy=pt.common.ClearingPolicy.NEVER,  # to decide when data should be cleared/reset.
                         qos_profile=QoSProfile(reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
                                                history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
                                                depth=10))
        
        # TODO: initialise class variables and blackboard variables
        # YOUR CODE HERE
        self.blackboard.register_key(
            key='collison_warning',
            access=pt.common.Access.WRITE
        )
        
        self.blackboard.register_key(
            key='point_at_min_dist',
            access=pt.common.Access.WRITE
        )
        
        
        self.blackboard.collison_warning = False   # decision making
        self.safe_min_range = safe_range
        
#         raise NotImplementedError()

    def update(self):
        # TODO: impletment the update function to check the laser scan data and update the blackboard variable
        # YOUR CODE HERE
        self.logger.info("[LASER SCAN] update: running scan_2bb update")
        self.logger.debug("%s.update()" % self.__class__.__name__)
        status = super(LaserScan2bb, self).update()
        
        if self.blackboard.exists('laser_scan'):
            laser_data = np.array(self.blackboard.laser_scan)
            laser_data[laser_data <= 0.05] = 1.0
            laser_data = np.nan_to_num(laser_data, nan=40)
            laser_data[np.isinf(laser_data)]=50
            # laser_data = laser_data.tolist()

            self.blackboard.point_at_min_dist = min(laser_data)

            if self.blackboard.point_at_min_dist < self.safe_min_range:
                self.blackboard.collison_warning = True
            else:
                self.blackboard.collison_warning = False
            
            return pt.common.Status.RUNNING
            
        else:
            print("Initiating Laser Scan")
            return pt.common.Status.RUNNING 

class Rotate90(pt.behaviour.Behaviour):
    """Rotates the robot by 90 degrees
    """
    def __init__(self, name="rotate 90",
                topic_name="/cmd_vel",
                ang_vel=1.0):
        self.topic_name = topic_name
        self.max_ang_vel = ang_vel # units: rad/sec
        super(Rotate90, self).__init__(name)
     
    def setup(self, **kwargs):
        """Setting up things which generally might require time to prevent delay in the tree initialisation
        """
        self.logger.info("[ROTATE] setting up rotate behaviour")
        
        try:
            self.node = kwargs['node']
        except KeyError as e:
            error_message = "didn't find 'node' in setup's kwargs [{}][{}]".format(self.qualified_name)
            raise KeyError(error_message) from e
        
        # Create publisher to publish rotation commands
        self.cmd_vel_pub = self.node.create_publisher(
            msg_type=Twist,
            topic=self.topic_name,
            qos_profile=ptr.utilities.qos_profile_latched()
        )

        self.feedback_message = "setup"

        return True
    
    def update(self):
        """Rotate the robot by 90 degrees with in a for loop

        """
        self.logger.info("[ROTATE] update: updating rotate behaviour")
        self.logger.debug("%s.update()" % self.__class__.__name__)

        # Rotate the robot by 90 degrees
        msg = Twist()
        msg.angular.z = 0.5

        for _ in range(100):
            self.cmd_vel_pub.publish(msg)
            rclpy.spin_once(self.node, timeout_sec=0.1)

        self.feedback_message = "rotated 90 degrees"
        return pt.common.Status.FAILURE
    
    def terminate(self, new_status):
        """Trigerred once the execution of the behaviour finishes, 
        i.e. when the status changes from RUNNING to SUCCESS or FAILURE
        """
        self.logger.info("[ROTATE] terminate: publishing zero angular velocity")

        # Stop the robot
        twist_msg = Twist()
        twist_msg.linear.x = 0.
        twist_msg.linear.y = 0.
        twist_msg.angular.z = 0.
                    
        self.cmd_vel_pub.publish(twist_msg)
        self.sent_goal = False
        
        return pt.common.Status.SUCCESS