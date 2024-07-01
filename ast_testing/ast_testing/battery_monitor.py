from ast_testing.behaviours import *
import py_trees as pt
import py_trees_ros as ptr
import operator

import py_trees.console as console
import rclpy
import sys

def create_root() -> pt.behaviour.Behaviour:
    """Structures a behaviour tree to monitor the battery status, and start
    to rotate if the battery is low and stop if it detects an obstacle in front of it.
    """

    # we define the root node
    root = pt.composites.Parallel(name="root",
                                  policy=pt.common.ParallelPolicy.SuccessOnAll(synchronise=False))    

    ### we create a sequence node called "Topics2BB" and a selector node called "Priorities"
    topics2BB = pt.composites.Sequence("Topics2BB", memory=False)
    priorities = pt.composites.Selector("Priorities", memory=False)

    ### we create an "Idle" node, which is a running node to keep the robot idle
    idle = pt.behaviours.Running(name="Idle")
    
    """
    TODO:  The first and second level of the tree structure is defined above, but please
    define the rest of the tree structure.

    Class definitions for your behaviours are provided in behaviours.py; you also need to fill out
    the behaviour implementations!

    HINT: Some behaviours from pt.behaviours may be useful to use as well.
    """

    # YOUR CODE HERE
    battery2bb = BatteryStatus2bb(
        name="Battery2BB",
        battery_voltage_topic_name="/battery_voltage"  
    )

    scan2bb = LaserScan2bb(
        name="Scan2BB",
        topic_name="/scan",
        safe_range=0.5
    )
     
    rotate_platform = Rotate(name="rotate_platform", topic_name="/cmd_vel")
    stop_platform = StopMotion(name="stop_platform", topic_name1="/cmd_vel")
    rotate_90_platform = Rotate90(name="rotate_90_platform", topic_name="/cmd_vel", ang_vel=1.0, angle=90.0)

    def check_battery_low_on_blackboard(blackboard):
        return blackboard.battery_low_warning 
    

    def check_collison_warn_on_blackboard(blackboard):
        return blackboard.collison_warning
     
    
    blackboard = pt.blackboard.Blackboard()
    blackboard.battery_low_warning = False
     
    battery_emergency = pt.decorators.EternalGuard(
        name="Battery Low?",
        condition=check_battery_low_on_blackboard,
        blackboard_keys={"battery_low_warning"},
        child = rotate_platform
    )
    
    collide_emergency = pt.decorators.EternalGuard(
        name="Colliding?",
        condition=check_collison_warn_on_blackboard,
        blackboard_keys={"collison_warning"},
        child = rotate_90_platform
    )

    

#     raise NotImplementedError()

    # TODO: construct the behaviour tree structure using the nodes and behaviours defined above
    # HINT: for reference, the sample tree structure in the README.md file might be useful

    root.add_children([topics2BB, priorities])
    topics2BB.add_child(battery2bb)
    topics2BB.add_child(scan2bb)
    priorities.add_child(battery_emergency)
    priorities.add_child(collide_emergency)
    priorities.add_child(idle)

    # YOUR CODE HERE
#     raise NotImplementedError()

    return root

def main():
    """Initialises and executes the behaviour tree
    """
    rclpy.init(args=None)

    root = create_root()
    tree = ptr.trees.BehaviourTree(root=root, unicode_tree_debug=True)

    try:
        tree.setup(timeout=30.0)
    except ptr.exceptions.TimedOutError as e:
        console.logerror(console.red + "failed to setup the tree, aborting [{}]".format(str(e)) + console.reset)
        tree.shutdown()
        rclpy.try_shutdown()
        sys.exit(1)
    except KeyboardInterrupt:
        # user-initiated shutdown
        console.logerror("tree setup interrupted")
        tree.shutdown()
        rclpy.try_shutdown()
        sys.exit(1)
    
    # frequency of ticks
    tree.tick_tock(period_ms=100)    

    try:
        rclpy.spin(tree.node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        tree.shutdown()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()