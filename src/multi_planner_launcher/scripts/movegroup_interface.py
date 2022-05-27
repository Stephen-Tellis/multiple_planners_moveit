#!/usr/bin/env python3

# import ROS libraries
import rospy

# Import pre-defined msg/srv/actions

# import custom message types

# Created classes 
from bot_movegroup import BotMovegroup


EXAMPLE_JOINT_STATES = {
                'POS_START': [-90, -60, -78, -78, 91, 0],
                'POS_ONE': [-36, -173, -9, -103, 9, 9],
                'POS_TWO': [-80, -173, -20, -7, 90, 9]
                }


class MoveCheckbot(BotMovegroup):
    """
    The one and the only interface between moveit and 
    every other package within the checkbots codebase
    """
    def __init__(self):
        super().__init__()

        rospy.init_node('movegroup_interface', anonymous=True)


def main():
    mcc = MoveCheckbot() 
    mcc.interface_primary_movegroup()
    mcc.interface_secondary_movegroup()
    rospy.loginfo("Python MoveGroup is ready for a demo ...\n")

    rospy.sleep(1) # Sleeps for 1 sec (demo)

    planners = ["primary", "secondary"]
    for curr_planner in planners:
        rospy.loginfo(f"{curr_planner} planner will now be used \n")

        for joint_target_name, target_values in EXAMPLE_JOINT_STATES.items():
            rospy.loginfo(f"Going to target {joint_target_name} with {curr_planner} planner")
            if mcc.plan_and_execute_traj(target_values, curr_planner):
                rospy.loginfo("Successfully executed trajectory \n")

                rospy.sleep(1) # Sleeps for 1 sec (demo)


if __name__ == "__main__":
    main()
