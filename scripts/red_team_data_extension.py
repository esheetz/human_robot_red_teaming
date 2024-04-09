#!/usr/bin/env python3
"""
Red Team Data Extension Node
Emily Sheetz, NSTGRO VTE 2024
"""

import rospy

import os, sys
import random

from yaml_formatting_checks import YAMLChecks

# import state space, consequence space, action space, and policy data point classes
from risky_condition import RiskyCondition
from consequence_state import ConsequenceState
from risk_mitigating_action import RiskMitigatingAction
from risk_mitigating_policy_data_point import RiskMitigatingPolicyDataPoint

# import state space, consequence space, action space, and policy data readers
from risky_condition_reader import RiskyConditionReader
from consequence_state_reader import ConsequenceStateReader
from risk_mitigating_action_reader import RiskMitigatingActionReader
from risk_mitigating_policy_data_reader import RiskMitigatingPolicyDataReader

# import red team
from red_team_policy import RedTeamPolicy

# import command line tools
from red_team_command_line_tools import RedTeamCommandLinePrinting as CLP
from red_team_command_line_tools import UserInputActionProcessing as UIAction
from red_team_command_line_tools import UserInputConsequenceProcessing as UIConseq

class RedTeamDataExtension:
    def __init__(self, robot="val", environment="lunar_habitat", num_points=10, max_conds=-1, counter_factual_mode=False):
        # set internal parameters
        self.robot_name = robot
        self.environment_name = environment
        self.num_red_team_points = num_points
        self.max_conds_per_point = max_conds
        self.cf_mode = counter_factual_mode

        # write policy to file after # of new policy points generated
        self.save_new_policy_points = 10

        # initialize red team
        self.red_team = RedTeamPolicy(robot=self.robot_name,
                                      environment=self.environment_name)
        self.num_starting_points = None
        self.continue_data_generation = False

    #######################
    ### GETTERS/SETTERS ###
    #######################

    def check_initialized(self):
        return self.red_team.initialized

    def check_valid_policy(self):
        return self.red_team.valid_policy

    def get_points_generated(self):
        # compute number of new data points generated
        if not self.cf_mode:
            return self.red_team.get_num_red_team_policy_data() - self.num_starting_points
        else:
            return self.red_team.get_num_counter_factual_policy_data() - self.num_starting_points

    def check_points_generated(self):
        return (self.get_points_generated() == self.num_red_team_points)

    def check_continue_data_generation(self):
        return self.continue_data_generation

    def get_mode_name(self):
        if not self.cf_mode:
            return "risky scenario"
        else:
            return "counter-factual"

    ######################
    ### INITIALIZATION ###
    ######################

    def initialize_red_team(self):
        # initialize
        self.red_team.initialize()

        # if initialization successful, prep for data generation
        if self.red_team.initialized and self.red_team.valid_policy:
            if not self.cf_mode:
                self.num_starting_points = self.red_team.get_num_red_team_policy_data()
            else:
                self.num_starting_points = self.red_team.get_num_counter_factual_policy_data()
            self.continue_data_generation = True
        return

    ####################################
    ### RED TEAM SCENARIO GENERATION ###
    ####################################

    def get_random_red_teamed_scenario(self, state_space, consequence_space):
        # set bounds for number of conditions to be generated
        lower_bound = 1
        upper_bound = self.max_conds_per_point if (self.max_conds_per_point != -1) else len(state_space)

        # get number of conditions to generate for this scenario
        k = random.randint(lower_bound, upper_bound)

        # get random conditions
        scenario = random.sample(state_space, k)

        # make sure conditions are in sorted order
        scenario.sort()

        # get all corresponding consequences
        conseq_set = set()
        for condition_name in scenario:
            # get condition
            condition = self.red_team.get_risky_condition_with_name(condition_name)
            # get consequences
            consequences = condition.get_consequence_states()
            # add each consequence to set
            for conseq in consequences:
                conseq_set.add(conseq)

        # convert to list
        scenario_consequences = list(conseq_set)

        # make sure consequences are in sorted order
        scenario_consequences.sort()

        return scenario, scenario_consequences

    #############################
    ### DATA POINT GENERATION ###
    #############################

    def generate_new_data_point(self):
        if not self.cf_mode:
            self.__generate_new_risky_scenario_data_point()
        else:
            self.__generate_new_counter_factual_data_point()
        return

    def __generate_new_risky_scenario_data_point(self):
        # get state and action space
        state_space = self.red_team.get_state_space()
        conseq_space = self.red_team.get_consequence_state_space()
        action_space = self.red_team.get_action_space()

        # generate random scenario
        red_team_conditions, red_team_consequences = self.get_random_red_teamed_scenario(state_space, conseq_space)

        # get action from user input and resolve conflicts (if necessary)
        output = UIAction.get_action_from_user_and_resolve_conflicts(self.red_team, red_team_conditions, red_team_consequences, action_space)
        # unpack
        self.continue_data_generation, abort, action = output
        # check if aborting this data point
        if abort:
            return

        # get consequences from user input and resolve conflicts (if necessary)
        output = UIConseq.get_consequences_from_user_and_resolve_conflicts(self.red_team, red_team_conditions, red_team_consequences, action, conseq_space)
        # unpack
        self.continue_data_generation, abort, conseqs = output
        # check if aborting this data point
        if abort:
            return

        # create policy data point
        pol_point = RiskMitigatingPolicyDataPoint(conditions=red_team_conditions,
                                                  consequences_before_action=red_team_consequences,
                                                  action=action,
                                                  consequences_after_action=conseqs)

        # update policy
        CLP.print_update_policy_message(red_team_conditions, red_team_consequences, action, conseqs)
        self.red_team.update_policy(pol_point)

        # check if policy needs to be written to file
        if (self.get_points_generated() != 0) and ((self.get_points_generated() % self.save_new_policy_points) == 0):
            self.write_policy_to_file()

        return

    def __generate_new_counter_factual_data_point(self):
        rospy.logwarn("[Red Team Data Extension] FUNCTION NOT IMPLEMENTED")
        # TODO IMPLEMENTATION

    ###########################
    ### SAVE POLICY TO FILE ###
    ###########################

    def write_policy_to_file(self):
        print("*** Writing policy to file...")
        if not self.cf_mode:
            self.red_team.write_policy_to_file()
        else:
            self.red_team.write_counter_factual_policy_to_file()
        print("*** Policy saved!")
        return



#########################
### RUN NODE FUNCTION ###
#########################

def run_red_team_data_extension_node():
    # set node name
    node_name = "RedTeamDataExtension"
    param_prefix = "/" + node_name + "/"

    # get ROS parameters
    robot_name = rospy.get_param(param_prefix + 'robot', "val")
    env_name = rospy.get_param(param_prefix + 'environment', "lunar_habitat")
    num_points = rospy.get_param(param_prefix + 'num_points', 10)
    max_conds_per_point = rospy.get_param(param_prefix + 'max_conds', -1)
    cf_mode = rospy.get_param(param_prefix + 'counter_factual', False)

    # initialize node
    rospy.init_node(node_name)

    # create red team node
    rospy.loginfo("[Red Team Data Extension] Creating human-robot red team data extension node...")
    red_team = RedTeamDataExtension(robot=robot_name, environment=env_name,
                                    num_points=num_points,
                                    max_conds=max_conds_per_point,
                                    counter_factual_mode=cf_mode)
    rospy.loginfo("[Red Team Data Extension] Initializing human-robot red team data extension node...")
    red_team.initialize_red_team()

    # verify initialization and policy
    if red_team.check_initialized() and red_team.check_valid_policy():
        rospy.loginfo("[Red Team Data Extension] Successfully initialized human-robot red team data extension node!")
        rospy.loginfo("[Red Team Data Extension] Initial human-robot red team data is valid!")
    else:
        if not red_team.check_initialized():
            rospy.logerr("[Red Team Data Extension] Could not initialize human-robot red team data extension node")
        if not red_team.check_valid_policy():
            rospy.logerr("[Red Team Data Extension] Initial human-robot red team data is invalid")
        # exit with error
        sys.exit(1)
    print()

    # initialize loop rate
    rate = rospy.Rate(10) # Hz

    # run node
    while not rospy.is_shutdown() and \
          not red_team.check_points_generated() and \
          red_team.check_continue_data_generation():
        rospy.loginfo("[Red Team Data Extension] Generating new red teamed %s data point...", red_team.get_mode_name())
        print()
        red_team.generate_new_data_point()
        rate.sleep()

    # check stopping conditions
    if red_team.check_points_generated():
        rospy.loginfo("[Red Team Data Extension] Completed generating %d new %s data points through human-robot red teaming! GO TEAM!",
                      red_team.get_points_generated(), red_team.get_mode_name())
    else:
        rospy.loginfo("[Red Team Data Extension] Quitting after generating %d new %s data points through human-robot red teaming. RedTeamwork makes the dream work!",
                      red_team.get_points_generated(), red_team.get_mode_name())

    # write final policy to file
    red_team.write_policy_to_file()

    rospy.loginfo("[Red Team Data Extension] Node stopped, all done!")
    # exit with success
    sys.exit(0)

#####################
### MAIN FUNCTION ###
#####################

if __name__ == '__main__':
    # debugging
    # param_names = rospy.get_param_names()
    # for name in param_names:
    #     print("***** DEBUG: got param {}".format(name))

    run_red_team_data_extension_node()
