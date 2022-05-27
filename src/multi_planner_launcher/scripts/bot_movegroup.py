#!/usr/bin/env python3

# Std python libs
import sys
from math import radians

# import ROS libraries
import rospy
import moveit_commander

# Get ros params
try:
    PLANNING_TIME = rospy.get_param("/plan/time")
    PLANNING_ATTEMPTS = rospy.get_param("/plan/attempts")
    VEL_SCALE = rospy.get_param("/vel_scale")
    ACC_SCALE = rospy.get_param("/acc_scale")
    PRIMARY_PLANNER = rospy.get_param("/primary_planner")
    SECONDARY_PLANNER = rospy.get_param("/secondary_planner")
    SECONDARY_NAMESPACE = rospy.get_param("/secondary_namespace")
except:
    sys.exit("The required parameters are not loaded onto the param server")

class BotMovegroup:

    def __init__(self):
        # initialize moveit_commander
        moveit_commander.roscpp_initialize(sys.argv)

    def interface_primary_movegroup(self):
        """
        Initialize move group within seperate namespaces
        The primary planner is in the default namespace
        Planner name is taken from the param server
        """
        self.robot , self.planning_scene, self.move_group = self._initialize_planner(planner_name=PRIMARY_PLANNER)

    def interface_secondary_movegroup(self):
        """
        Initialize move group within seperate namespaces
        The secondary planner takes in the namespace and planner from the param server
        """
        self.robot_secondary, self.planning_scene_secondary, self.move_group_secondary = self._initialize_planner(
            planner_name=SECONDARY_PLANNER, 
            namespace=SECONDARY_NAMESPACE)

    def _initialize_planner(self, planner_name, namespace=""):

        # instantiate the robot commander we need this to get data about the current robot state
        robot = moveit_commander.RobotCommander()

        # instantiate the scene. Needed to get, set and update the robots surrounding of the outside world
        scene = moveit_commander.PlanningSceneInterface()

        # Instantiate the move group object - this is the one that takes care of planning and execution
        group_name = "manipulator"
        move_group = moveit_commander.MoveGroupCommander(group_name, ns=namespace)

        # set planner to be used
        move_group.set_planner_id(planner_name)

        # Initialize planning parameters
        move_group.allow_replanning(True)
        move_group.set_planning_time(PLANNING_TIME)
        move_group.set_num_planning_attempts(PLANNING_ATTEMPTS)
        move_group.set_max_velocity_scaling_factor(VEL_SCALE)
        move_group.set_max_acceleration_scaling_factor(ACC_SCALE)

        return robot, scene, move_group

    def plan_and_execute_traj(self, target_joints, planner="primary"):
        """
        Given a target joint state, plans and executes the trajectory
        with moveit
        @param
        """
        target_joints_rad = BotMovegroup._convert_to_radians(target_joints)
        if planner == "primary":
            self.move_group.set_start_state_to_current_state()
            self.move_group.set_joint_value_target(target_joints_rad)
            plan = self.move_group.plan()
        elif planner == "secondary":
            self.move_group_secondary.set_start_state_to_current_state()
            self.move_group_secondary.set_joint_value_target(target_joints_rad)
            plan = self.move_group_secondary.plan()

        exec_val = self._execute_given_traj(plan)

        return exec_val

    def _execute_given_traj(self, plan):
        """
        Given a moveit plan tuple, executes the trajectory synchronously
        Only the primary movegroup is capable of trajectory exec as per our move_group.launch
        All plans are redirected to it
        """
        if plan[0]:
            return self.move_group.execute(plan[1], wait=True)
        else:
            return False
        

    @classmethod
    def _convert_to_radians(cls, target_joints):
        return [radians(i) for i in target_joints]