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
        # get state and action space
        state_space = self.red_team.get_state_space()
        conseq_space = self.red_team.get_consequence_state_space()
        action_space = self.red_team.get_action_space()

        # generate random scenario
        red_team_conditions, red_team_consequences = self.get_random_red_teamed_scenario(state_space, conseq_space)

        # get scenario input from user
        action_idx = self.get_scenario_input(red_team_conditions, red_team_consequences, action_space)

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

        # get consequences after action input from user
        conseq_idxs = self.get_consequence_input(red_team_conditions, red_team_consequences, action, conseq_space)

        # verify consequences are not None
        if conseq_idxs is None:
            # no valid consequences received, check whether skipping scenario or quitting data generation
            if self.continue_data_generation:
                # skipping scenario
                self.print_skip_message()
                return
            else:
                # quitting
                return

        # get consequences
        conseqs = [conseq_space[i] for i in conseq_idxs]

        # create policy data point
        pol_point = RiskMitigatingPolicyDataPoint(conditions=red_team_conditions,
                                                  consequences_before_action=red_team_consequences,
                                                  action=action,
                                                  consequences_after_action=conseqs)

        # check for duplicates already in policy
        conflict, point_act, pol_act = pol_point.check_and_get_conflicting_data_point(self.red_team.policy_data)
        if conflict:
            # report conflict
            self.print_action_conflict_message(red_team_conditions, point_act, pol_act)

            # get scenario input from user
            action_idx = self.get_scenario_input(red_team_conditions, red_team_consequences, action_space)

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

            # get consequences after action input from user
            conseq_idxs = self.get_consequence_input(red_team_conditions, red_team_consequences, action, conseq_space)

            # verify consequences are not None
            if conseq_idxs is None:
                # no valid consequences received, check whether skipping scenario or quitting data generation
                if self.continue_data_generation:
                    # skipping scenario
                    self.print_skip_message()
                    return
                else:
                    # quitting
                    return

            # get consequences
            conseqs = [conseq_space[i] for i in conseq_idxs]

            # create policy data point
            pol_point = RiskMitigatingPolicyDataPoint(conditions=red_team_conditions,
                                                      consequences_before_action=red_team_consequences,
                                                      action=action,
                                                      consequences_after_action=conseqs)

        # update policy
        self.print_update_policy_message(red_team_conditions, red_team_consequences, action, conseqs)
        self.red_team.update_policy(pol_point)

        # check if policy needs to be written to file
        if (self.get_points_generated() != 0) and ((self.get_points_generated() % self.save_new_policy_points) == 0):
            self.write_policy_to_file()

        return

    ################
    ### PRINTING ###
    ################

    def print_red_teamed_scenario(self, scenario, scenario_consequences):
        print("    What should the robot do when the following RISK CONDITIONS (with possible future CONSEQUENCES) exist in the current state?")
        print("        RISK CONDITIONS:")
        for condition_name in scenario:
            # get condition from state space
            condition = self.red_team.get_risky_condition_with_name(condition_name)
            print("          - " + condition_name + "    [ " +
                  "L=" + str(condition.get_likelihood_level()) +
                  ", C=" + str(condition.get_consequence_class()) +
                  ", Risk=" + str(condition.get_matrix_risk_score()) +
                  " (" + condition.get_matrix_risk_score_name() + ") ]")
        print("        FUTURE CONSEQUENCES:")
        for consequence_name in scenario_consequences:
            print("          - " + consequence_name)
        print()
        return

    def print_action_and_consequences(self, scenario_consequences, action):
        print("    Given the possible future CONSEQUENCES, after the robot takes the RISK MITIGATING ACTION, what future CONSEQUENCES can still occur?")
        print("        CONSEQUENCES BEFORE ACTION:")
        for conseq_name in scenario_consequences:
            print("          - " + conseq_name)
        print("        RISK MITIGATING ACTION: " + action)
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

    def print_consequences(self, conseq_space):
        print("    Please select the consequences possible after the action is taken (as a comma separated list of consequence numbers):")
        print("        Consequence state space:")
        for i in range(len(conseq_space)):
            print("          " + str(i) + ". " + conseq_space[i])
        print("          " + str(len(conseq_space)) + ". [SKIP THIS SCENARIO]")
        print("    [type 'exit()' to quit]")
        print()
        return

    def print_try_again_message(self):
        print("    Please try again.")
        print()
        return

    def print_separator(self):
        print()
        print("====================================================================================================")
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

    def print_update_policy_message(self, red_team_conditions, red_team_consequences, action, conseqs):
        print("    *** Great! Updating policy!")
        print("            Conditions:", red_team_conditions)
        print("            Consequences before action:", red_team_consequences)
        print("            Action:", action)
        print("            Consequences after action:", conseqs)
        print()
        return

    ############################################################
    ### USER INPUT PROCESSING - SELECT ACTION GIVEN SCENARIO ###
    ############################################################

    def get_scenario_input(self, red_team_conditions, red_team_consequences, action_space):
        # initialize loop flag and action index
        got_valid_action = False
        action_idx = None

        # keep asking until valid input received
        while not got_valid_action:
            # print scenario
            self.print_red_teamed_scenario(red_team_conditions, red_team_consequences)

            # print possible actions
            self.print_actions(action_space)

            # ask for user input
            quit, skip, action_idx = self.get_user_input_action(action_space)

            # check result
            if quit:
                self.continue_data_generation = False
                return None
            if skip:
                return None
            if action_idx is None:
                self.print_try_again_message()
                continue

            # valid action received
            break

        # return action index
        return action_idx

    def get_user_input_action(self, action_space):
        # prompt user for input
        val = self.prompt_user_input_action()

        # check for quit
        if val == "exit()":
            return True, None, None

        # validate action index
        skip, action_idx = self.validate_action_index(val, action_space)

        return False, skip, action_idx

    def prompt_user_input_action(self):
        val = input("    Action number: ")
        self.print_separator()
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

    ################################################################
    ### USER INPUT PROCESSING - SELECT CONSEQUENCES GIVEN ACTION ###
    ################################################################

    def get_consequence_input(self, red_team_conditions, red_team_consequences, action, conseq_space):
        # initialize loop flag and consequence indices
        got_valid_consequences = False
        conseq_idxs = None

        # keep asking until valid input received
        while not got_valid_consequences:
            # print scenario
            self.print_action_and_consequences(red_team_consequences, action)

            # print possible consequences
            self.print_consequences(conseq_space)

            # ask for user input
            quit, skip, action_idx = self.get_user_input_consequences(conseq_space)

            # check result
            if quit:
                self.continue_data_generation = False
                return None
            if skip:
                return None
            if action_idx is None:
                self.print_try_again_message()
                continue

            # valid consequences received
            break

        # return consequence indices
        return conseq_idxs

    def get_user_input_consequences(self, conseq_space):
        # prompt user for input
        val = self.prompt_user_input_consequences()

        # check for quit
        if val == "exit()":
            return True, None, None

        # validate consequence indices
        skip, conseq_idxs = self.validate_consequence_indices(val, conseq_space)

        return False, skip, conseq_idxs

    def prompt_user_input_consequences(self):
        val = input("    Consequence numbers: ")
        self.print_separator()
        return val

    def validate_consequence_indices(self, val, conseq_space):
        # initialize value
        int_vals = None

        # try to convert comma separated string to list of ints
        try:
            int_vals = [int(i) for i in val.split(',')]
        except:
            print("    *** INVALID INPUT: " + val + " cannot be converted to a list of integers.")
            print()
            return None, None

        # check indices
        for int_val in int_vals:
            if (int_val < 0) or (int_val > len(conseq_space)):
                print("    *** INVALID INPUT: received " + str(int_val) + ", but must be in range [0," + str(len(conseq_space)) + "]")
                print()
                return None, None
            elif int_val == len(conseq_space):
                return True, None
            else:
                return False, int_vals

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
