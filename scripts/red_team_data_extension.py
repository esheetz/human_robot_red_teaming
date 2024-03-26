#!/usr/bin/env python3
"""
Red Team Data Extension Node
Emily Sheetz, NSTGRO VTE 2024
"""

import rospy

import os, sys

from yaml_formatting_checks import YAMLChecks

# import state space, action space, and policy data point classes
from risky_condition import RiskyCondition
from risk_mitigating_action import RiskMitigatingAction
from risk_mitigating_policy_data_point import RiskMitigatingPolicyDataPoint

# import state space, action space, and policy data readers
from risky_condition_reader import RiskyConditionReader
from risk_mitigating_action_reader import RiskMitigatingActionReader
from risk_mitigating_policy_data_reader import RiskMitigatingPolicyDataReader

# import red team
from red_team_policy import RedTeamPolicy

class RedTeamDataExtension:
    def __init__(self, robot="val", environment="lunar_habitat"):
        # set internal parameters
        self.robot_name = robot
        self.environment_name = environment

        # initialize red team
        self.red_team = RedTeamPolicy(robot=self.robot_name,
                                      environment=self.environment_name)

    #######################
    ### GETTERS/SETTERS ###
    #######################

    def check_initialized(self):
        return self.red_team.initialized

    def check_valid_policy(self):
        return self.red_team.valid_policy

    ######################
    ### INITIALIZATION ###
    ######################

    def initialize_red_team(self):
        self.red_team.initialize()



#####################
### MAIN FUNCTION ###
#####################

if __name__ == '__main__':
    # set node name
    node_name = "RedTeamDataExtension"
    param_prefix = "/" + node_name + "/"

    # get ROS parameters
    robot_name = rospy.get_param(param_prefix + 'robot', "val")
    env_name = rospy.get_param(param_prefix + 'environment', "lunar_habitat")

    # initialize node
    rospy.init_node(node_name)

    # create red team node
    rospy.loginfo("[Red Team Data Extension] Creating human-robot red team data extension node...")
    red_team = RedTeamDataExtension(robot=robot_name, environment=env_name)
    rospy.loginfo("[Red Team Data Extension] Initializing human-robot red team data extension node...")
    red_team.initialize_red_team()
    
    # verify initialization and policy
    if red_team.check_initialized() and red_team.check_valid_policy():
        rospy.loginfo("[Red Team Data Extension] Successfully initialized human-robot red team data extension node!")
        rospy.loginfo("[Red Team Data Extension] Initial human-robot red team policy is valid!")
    else:
        if not red_team.check_initialized():
            rospy.logerr("[Red Team Data Extension] Could not initialize human-robot red team data extension node")
        if not red_team.check_valid_policy():
            rospy.logerr("[Red Team Data Extension] Initial human-robot red team policy is invalid")
        # exit with error
        sys.exit(1)

    # # run node
    # while not rospy.is_shutdown():
    #     rospy.spin()
    #     # TODO how will this node run?

    rospy.loginfo("[Red Team Data Extension] Node stopped, all done!")
