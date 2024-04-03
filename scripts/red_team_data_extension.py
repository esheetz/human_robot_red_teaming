#!/usr/bin/env python3
"""
Red Team Data Extension Node
Emily Sheetz, NSTGRO VTE 2024
"""

import rospy

import os, sys
import random

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
    def __init__(self, robot="val", environment="lunar_habitat", num_points=10, max_conds=-1):
        # set internal parameters
        self.robot_name = robot
        self.environment_name = environment
        self.num_red_team_points = num_points
        self.max_conds_per_point = max_conds

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
        return self.red_team.get_num_red_team_policy_data() - self.num_starting_points

    def check_points_generated(self):
        return (self.get_points_generated() == self.num_red_team_points)

    def check_continue_data_generation(self):
        return self.continue_data_generation

    ######################
    ### INITIALIZATION ###
    ######################

    def initialize_red_team(self):
        # initialize
        self.red_team.initialize()

        # if initialization successful, prep for data generation
        if self.red_team.initialized and self.red_team.valid_policy:
            self.num_starting_points = self.red_team.get_num_red_team_policy_data()
            self.continue_data_generation = True
        return

    ####################################
    ### RED TEAM SCENARIO GENERATION ###
    ####################################

    def get_random_red_teamed_scenario(self, state_space):
        # set bounds for number of conditions to be generated
        lower_bound = 1
        upper_bound = self.max_conds_per_point if (self.max_conds_per_point != -1) else len(state_space)

        # get number of conditions to generate for this scenario
        k = random.randint(lower_bound, upper_bound)

        # get random conditions
        scenario = random.sample(state_space, k)

        # make sure conditions are in sorted order
        scenario.sort()

        return scenario

    #############################
    ### DATA POINT GENERATION ###
    #############################

    def generate_new_data_point(self):
        # get state and action space
        state_space = self.red_team.get_state_space()
        action_space = self.red_team.get_action_space()

        # generate random scenario
        red_team_conditions = self.get_random_red_teamed_scenario(state_space)

        # get scenario input from user
        action_idx = self.get_scenario_input(red_team_conditions, action_space)

        # verify action is not None
        if action_idx is None:
            # no valid action received, check whether skipping scenario or quitting data generation
            if self.continue_data_generation:
                # skipping scenario
                self.print_skip_message()
                return
            else:
                # quitting
                return

        # get action
        action = action_space[action_idx]

        # create policy data point
        pol_point = RiskMitigatingPolicyDataPoint(conditions=red_team_conditions, action=action)

        # check for duplicates already in policy
        conflict, point_act, pol_act = pol_point.check_and_get_conflicting_data_point(self.red_team.policy_data)
        if conflict:
            # report conflict
            self.print_action_conflict_message(red_team_conditions, point_act, pol_act)

            # get scenario input from user
            action_idx = self.get_scenario_input(red_team_conditions, action_space)

            # verify action is not None
            if action_idx is None:
                # no valid action received, check whether skipping scenario or quitting data generation
                if self.continue_data_generation:
                    # skipping scenario
                    self.print_skip_message()
                    return
                else:
                    # quitting
                    return

            # get action
            action = action_space[action_idx]

            # create policy data point
            pol_point = RiskMitigatingPolicyDataPoint(conditions=red_team_conditions, action=action)

        # update policy
        self.print_update_policy_message(red_team_conditions, action)
        self.red_team.update_policy(pol_point)

        # check if policy needs to be written to file
        if (self.get_points_generated() != 0) and ((self.get_points_generated() % self.save_new_policy_points) == 0):
            self.write_policy_to_file()

        return

    ################
    ### PRINTING ###
    ################

    def print_red_teamed_scenario(self, scenario):
        print("    What should the robot do when the following risk conditions exist in the current state?")
        print("        Conditions:")
        for condition_name in scenario:
            # get condition from state space
            condition = self.red_team.get_risky_condition_with_name(condition_name)
            print("          - " + condition_name + "    [ " +
                  "L=" + str(condition.get_likelihood_level()) +
                  ", C=" + str(condition.get_consequence_class()) +
                  ", Risk=" + str(condition.get_risk_score()) +
                  " (" + condition.get_risk_score_name() + ") ]")
        print()
        return

    def print_actions(self, action_space):
        print("    Please select the number of the appropriate action to take in this state:")
        print("        Robot's risk mitigating action space:")
        for i in range(len(action_space)):
            print("          " + str(i) + ". " + action_space[i])
        print("          " + str(len(action_space)) + ". [SKIP THIS SCENARIO]")
        print("    [type 'exit()' to quit]")
        print()
        return

    def print_skip_message(self):
        print("    *** Skipping this red teamed scenario, no new data points generated...")
        print()
        return

    def print_action_conflict_message(self, red_team_conditions, point_act, pol_act):
        print("    *** ERROR: this scenario is already in red teamed policy with different action")
        print("            Conditions:", red_team_conditions)
        print("            Just entered action:", point_act)
        print("            Action in stored policy:", pol_act)
        print("    Let's resolve this conflict now.")
        print("    NOTE: your next selection will overwrite the stored policy value.")
        print()
        return

    def print_update_policy_message(self, red_team_conditions, action):
        print("    *** Great! Updating policy!")
        print("            Conditions:", red_team_conditions)
        print("            Action:", action)
        print()
        return

    #############################
    ### USER INPUT PROCESSING ###
    #############################

    def get_scenario_input(self, red_team_conditions, action_space):
        # initialize loop flag and action index
        got_valid_action = False
        action_idx = None

        # keep asking until valid input received
        while not got_valid_action:
            # print scenario
            self.print_red_teamed_scenario(red_team_conditions)

            # print possible actions
            self.print_actions(action_space)

            # ask for user input
            quit, skip, action_idx = self.get_user_input(action_space)

            # check result
            if quit:
                self.continue_data_generation = False
                return None
            if skip:
                return None
            if action_idx is None:
                print("    Please try again.")
                print()
                continue

            # valid action received
            break

        # return action index
        return action_idx

    def get_user_input(self, action_space):
        # prompt user for input
        val = self.prompt_user_input()

        # check for quit
        if val == "exit()":
            return True, None, None

        # validate action index
        skip, action_idx = self.validate_action_index(val, action_space)

        return False, skip, action_idx

    def prompt_user_input(self):
        val = input("    Action number: ")
        print()
        print("====================================================================================================")
        print()
        return val

    def validate_action_index(self, val, action_space):
        # initialize value
        int_val = None

        # try to convert to int
        try:
            int_val = int(val)
        except:
            print("    *** INVALID INPUT: " + val + " cannot be converted to an integer.")
            print()
            return None, None

        # check indices
        if (int_val < 0) or (int_val > len(action_space)):
            print("    *** INVALID INPUT: received " + val + ", but must be in range [0," + str(len(action_space)) + "]")
            print()
            return None, None
        elif int_val == len(action_space):
            return True, None
        else:
            return False, int_val

    ###########################
    ### SAVE POLICY TO FILE ###
    ###########################

    def write_policy_to_file(self):
        print("*** Writing policy to file...")
        self.red_team.write_policy_to_file()
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

    # initialize node
    rospy.init_node(node_name)

    # create red team node
    rospy.loginfo("[Red Team Data Extension] Creating human-robot red team data extension node...")
    red_team = RedTeamDataExtension(robot=robot_name, environment=env_name,
                                    num_points=num_points,
                                    max_conds=max_conds_per_point)
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
        rospy.loginfo("[Red Team Data Extension] Generating new red teamed data point...")
        print()
        red_team.generate_new_data_point()
        rate.sleep()

    # check stopping conditions
    if red_team.check_points_generated():
        rospy.loginfo("[Red Team Data Extension] Completed generating %d new data points through human-robot red teaming! GO TEAM!",
                      red_team.get_points_generated())
    else:
        rospy.loginfo("[Red Team Data Extension] Quitting after generating %d new data points through human-robot red teaming. RedTeamwork makes the dream work!",
                      red_team.get_points_generated())

    # write final policy to file
    red_team.write_policy_to_file()

    rospy.loginfo("[Red Team Data Extension] Node stopped, all done!")
    # exit with success
    sys.exit(0)

#####################
### MAIN FUNCTION ###
#####################

if __name__ == '__main__':
    run_red_team_data_extension_node()
