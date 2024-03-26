#!/usr/bin/env python3
"""
Red Team Data Extension Node
Emily Sheetz, NSTGRO VTE 2024
"""

import rospy

import os, sys, shutil
from copy import deepcopy
import yaml

from yaml_formatting_checks import YAMLChecks

# import state and action space classes
from risky_condition import RiskyCondition
from risk_mitigating_action import RiskMitigatingAction

# import state space, action space, and policy data readers
from risky_condition_reader import RiskyConditionReader
from risk_mitigating_action_reader import RiskMitigatingActionReader
from risk_mitigating_policy_data_reader import RiskMitigatingPolicyDataReader

class RedTeamDataExtension:
    def __init__(self, robot="val", environment="lunar_habitat"):
        # set internal parameters
        self.robot_name = robot
        self.environment_name = environment

        # get path of this script
        script_path = os.path.abspath(os.path.dirname( __file__ ))

        # initialize readers
        self.state_space_reader = RiskyConditionReader(robot=self.robot_name,
                                                      environment=self.environment_name)
        self.action_space_reader = RiskMitigatingActionReader(robot=self.robot_name,
                                                             environment=self.environment_name)
        self.policy_starter_reader = RiskMitigatingPolicyDataReader(robot=self.robot_name,
                                                                   environment=self.environment_name,
                                                                   human_gen_data=True)
        self.red_team_policy_reader = RiskMitigatingPolicyDataReader(robot=self.robot_name,
                                                                    environment=self.environment_name,
                                                                    human_gen_data=False)

        # set red teamed data file path
        self.red_team_data_full_path = self.red_team_policy_reader.get_risk_mitigating_policy_data_file_path()

        # initialize dictionary for policy data
        self.policy_data = {}

        # initialize flags
        self.valid_policy = False
        self.initialized = False

    #######################
    ### GETTERS/SETTERS ###
    #######################

    def get_robot_name(self):
        return self.robot_name

    def get_environment_name(self):
        return self.environment_name

    def get_red_team_data_file_path(self):
        return self.red_team_data_full_path

    def get_red_team_policy_data(self):
        return self.policy_data

    def get_num_red_team_policy_data(self):
        return len(self.policy_data.keys())

    ######################
    ### INITIALIZATION ###
    ######################

    def initialize_red_team(self):
        # initialize flag
        self.initialized = True

        # initialize spaces
        self.initialize_state_space()
        self.initialize_action_space()
        self.initialize_policies()        

        # error check policies against state and action spaces
        valid_policies = self.check_policies_against_state_action_spaces()

        # make sure human-generated policy data is in red teamed policy
        if valid_policies:
            self.policy_data = deepcopy(self.red_team_policy_reader.get_risk_mitigating_policy_data())
            self.valid_policy = self.red_team_policy_includes_human_policy()

        return

    #######################################
    ### INITIALIZATION HELPER FUNCTIONS ###
    #######################################

    def initialize_state_space(self):
        # process state space
        self.state_space_reader.process_risky_conditions()
        if not self.state_space_reader.check_valid_conditions():
            rospy.logerr("[Red Team Data Extension] Could not initialize state space")
            self.initialized = False
        else:
            rospy.loginfo("[Red Team Data Extension] Successfully initialized state space!")
        return

    def initialize_action_space(self):
        # process action space
        self.action_space_reader.process_risk_mitigating_actions()
        if not self.action_space_reader.check_valid_actions():
            rospy.logerr("[Red Team Data Extension] Could not initialize action space")
            self.initialized = False
        else:
            rospy.loginfo("[Red Team Data Extension] Successfully initialized action space!")
        return

    def initialize_policies(self):
        self.initialize_human_generated_policy()
        self.initialize_red_team_generated_policy()
        return

    def initialize_human_generated_policy(self):
        # process policy data
        self.policy_starter_reader.process_risk_mitigating_policy_data()
        if not self.policy_starter_reader.check_valid_policy():
            rospy.logerr("[Red Team Data Extension] Could not initialize human-generated policy data points")
            rospy.logwarn("[Red Team Data Extension] Skipping initialization of red team generated policy data points")
            self.initialized = False
        else:
            rospy.loginfo("[Red Team Data Extension] Successfully initialized human-generated policy data points!")
        return

    def initialize_red_team_generated_policy(self):
        # process red team policy data only if policy starter processed correctly
        if self.policy_starter_reader.check_valid_policy():
            # check if red team policy data exists and is non-empty
            red_team_exists = YAMLChecks.check_yaml_existence(self.red_team_policy_reader.get_risk_mitigating_policy_data_file_path())
            # check if red team policy data is non-empty
            red_team_nonempty = YAMLChecks.check_yaml_nonempty(self.red_team_policy_reader.get_risk_mitigating_policy_data_file_path())
            # check if red team needs to be initialized
            if not (red_team_exists and red_team_nonempty):
                rospy.loginfo("[Red Team Data Extension] Initializing red team generated policy data from human-generated policy data...")
                # red team data does not exist or is empty, so copy from human-generated data
                shutil.copyfile(src=self.policy_starter_reader.get_risk_mitigating_policy_data_file_path(),
                                dst=self.red_team_policy_reader.get_risk_mitigating_policy_data_file_path())

            # process red team policy data
            self.red_team_policy_reader.process_risk_mitigating_policy_data()
            if not self.red_team_policy_reader.check_valid_policy():
                rospy.logerr("[Red Team Data Extension] Could not initialize red team generated policy data points")
                self.initialized = False
            else:
                rospy.loginfo("[Red Team Data Extension] Successfully initialized red team generated policy data points!")

        return

    def check_policies_against_state_action_spaces(self):
        # get names of conditions and actions
        state_space_names = self.state_space_reader.get_risky_condition_names()
        action_space_names = self.action_space_reader.get_risk_mitigating_action_names()

        # check human-generated policy
        valid_human_gen_policy = self.check_policy_against_state_action_spaces("human-generated",
                                                                              self.policy_starter_reader.get_risk_mitigating_policy_data(),
                                                                              state_space_names,
                                                                              action_space_names)
        # check red team policy
        valid_red_team_policy = self.check_policy_against_state_action_spaces("red team generated",
                                                                              self.red_team_policy_reader.get_risk_mitigating_policy_data(),
                                                                              state_space_names,
                                                                              action_space_names)

        # check valid policies
        valid_policies = valid_human_gen_policy and valid_red_team_policy
        if not valid_policies:
            rospy.logerr("[Red Team Data Extension] Invalid policies do not match state and action spaces")
            self.initialized = False
        else:
            rospy.loginfo("[Red Team Data Extension] Policies match state and action spaces!")

        return valid_policies

    def check_policy_against_state_action_spaces(self, policy_nickname : str, policy : dict, state_space : list, action_space : list):
        # look through policy
        for conds in policy.keys():
            # get policy data point
            pol_data_point = policy[conds]
            # check data point against state and action spaces
            valid_data_point = pol_data_point.validate_data_point(state_space, action_space)
            if not valid_data_point:
                rospy.logerr("[Red Team Data Extension] Invalid %s policy data point; conditions and action not in state/action spaces; please resolve manually", policy_nickname)
                return False

        return True

    def red_team_policy_includes_human_policy(self):
        # look through human generated policy
        for conds in self.policy_starter_reader.get_risk_mitigating_policy_data().keys():
            # get policy data point
            pol_data_point = self.policy_starter_reader.get_risk_mitigating_policy_data()[conds]
            # check if data point already exists in policy
            if pol_data_point.check_data_point_duplicated(self.policy_data):
                # data point exists in policy, check if actions conflict
                if pol_data_point.check_conflicting_data_point(self.policy_data):
                    # data point conflicts, get conflicting actions
                    _, point_act, pol_act = pol_data_point.check_and_get_conflicting_data_point(self.policy_data)
                    rospy.logerr("[Red Team Data Extension] Conflict between human-generated and red team generated policy; please resolve manually")
                    print("    Conditions:", pol_data_point.get_policy_data_point_condition_names())
                    print("    Human-generated action:", point_act)
                    print("    Red team generated action:", pol_act)
                    return False
                # otherwise, no conflict
            else:
                # add data point to policy
                self.policy_data[pol_data_point.get_policy_data_point_condition_names()] = pol_data_point

        # if we get here, human-generated policy included in red teamed policy
        rospy.loginfo("[Red Team Data Extension] Human-generated and red team generated policies agree!")
        return True



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
    if red_team.initialized and red_team.valid_policy:
        rospy.loginfo("[Red Team Data Extension] Successfully initialized human-robot red team data extension node!")
        rospy.loginfo("[Red Team Data Extension] Initial human-robot red team policy is valid!")
    else:
        if not red_team.initialized:
            rospy.logerr("[Red Team Data Extension] Could not initialize human-robot red team data extension node")
        if not red_team.valid_policy:
            rospy.logerr("[Red Team Data Extension] Initial human-robot red team policy is invalid")
        # exit with error
        sys.exit(1)

    # # run node
    # while not rospy.is_shutdown():
    #     rospy.spin()
    #     # TODO how will this node run?

    rospy.loginfo("[Red Team Data Extension] Node stopped, all done!")
