#!/usr/bin/env python3
"""
Red Team Data Extension Node
Emily Sheetz, NSTGRO VTE 2024
"""

import rospy

import os, sys
import random, math

from yaml_formatting_checks import YAMLChecks

# import state space, consequence space, action space, and policy data point classes
from risky_condition import RiskyCondition
from consequence_state import ConsequenceState
from risk_mitigating_action import RiskMitigatingAction
from risk_mitigating_policy_data_point import RiskMitigatingPolicyDataPoint
from counter_factual_policy_data_point import CounterFactualPolicyDataPoint

# import state space, consequence space, action space, and policy data readers
from risky_condition_reader import RiskyConditionReader
from consequence_state_reader import ConsequenceStateReader
from risk_mitigating_action_reader import RiskMitigatingActionReader
from risk_mitigating_policy_data_reader import RiskMitigatingPolicyDataReader
from counter_factual_policy_data_reader import CounterFactualPolicyDataReader

# import red team
from red_team_policy import RedTeamPolicy

# import command line tools
from red_team_command_line_tools import RedTeamCommandLinePrinting as CLP
from red_team_command_line_tools import UserInputActionProcessing as UIAction
from red_team_command_line_tools import UserInputConsequenceProcessing as UIConseq

# import services for automatic data generation
from safety_aware_reasoning.srv import RiskyScenarioDataGeneration, RiskyScenarioDataGenerationRequest, RiskyScenarioDataGenerationResponse
from safety_aware_reasoning.srv import CounterFactualDataGeneration, CounterFactualDataGenerationRequest, CounterFactualDataGenerationResponse

class RedTeamDataExtension:
    def __init__(self, robot="val", environment="lunar_habitat",
                       num_points=10, max_conds=-1, counter_factual_mode=False,
                       auto_gen_data=True,
                       rs_auto_data_gen_service_name="", cf_auto_data_gen_service_name=""):
        # set internal parameters
        self.robot_name = robot
        self.environment_name = environment
        self.num_red_team_points = num_points
        self.max_conds_per_point = max_conds
        self.cf_mode = counter_factual_mode
        self.auto_gen_data = auto_gen_data
        self.rs_auto_data_gen_service_name = rs_auto_data_gen_service_name
        self.cf_auto_data_gen_service_name = cf_auto_data_gen_service_name

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

    def get_total_data_points(self):
        if not self.cf_mode:
            return self.red_team.get_num_red_team_policy_data()
        else:
            return self.red_team.get_num_counter_factual_policy_data()

    def get_num_possible_data_points(self):
        if not self.cf_mode:
            return self.get_num_possible_risky_data_points()
        else:
            return self.get_num_possible_cf_data_points()

    def get_num_possible_risky_data_points(self):
        # get total number of risky conditions
        n = self.red_team.state_space_reader.get_num_risky_conditions()

        # count total number of combinations
        total = 0
        for i in range(1,n+1):
            total += math.comb(n,i)

        return total

    def get_num_possible_cf_data_points(self):
        # get total number of risky data points
        p = self.get_num_possible_risky_data_points()
        # get total number of actions
        a = self.red_team.action_space_reader.get_num_risk_mitigating_actions()

        # compute total
        total = p * a

        return total

    def check_possible_points_generated(self):
        total_points = self.get_num_possible_data_points()
        if not self.cf_mode:
            return self.red_team.get_num_red_team_policy_data() >= total_points
        else:
            return self.red_team.get_num_counter_factual_policy_data() >= total_points

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

        # create service clients
        self.initialize_auto_data_gen_service_clients()

        return

    def initialize_auto_data_gen_service_clients(self):
        if not self.auto_gen_data:
            rospy.loginfo("[Red Team Data Extension] Generating data from user input, not initializing service clients")
            return

        # wait until service servers have started up and started listening for requests, then create service client
        rospy.loginfo("[Red Team Data Extension] Waiting for service server %s...", self.rs_auto_data_gen_service_name)
        rospy.wait_for_service(self.rs_auto_data_gen_service_name)
        rospy.loginfo("[Red Team Data Extension] ROS service %s is ready!", self.rs_auto_data_gen_service_name)
        self.rs_auto_data_gen_client = rospy.ServiceProxy(self.rs_auto_data_gen_service_name, RiskyScenarioDataGeneration)

        rospy.loginfo("[Red Team Data Extension] Waiting for service server %s...", self.cf_auto_data_gen_service_name)
        rospy.wait_for_service(self.cf_auto_data_gen_service_name)
        rospy.loginfo("[Red Team Data Extension] ROS service %s is ready!", self.cf_auto_data_gen_service_name)
        self.cf_auto_data_gen_client = rospy.ServiceProxy(self.cf_auto_data_gen_service_name, CounterFactualDataGeneration)

        rospy.loginfo("[Red Team Data Extension] ALL SERVICES READY!")

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

    ###########################################
    ### COUNTER FACTUAL SCENARIO GENERATION ###
    ###########################################

    def get_random_counter_factual_scenario_action(self, state_space, consequence_space, action_space):
        # get random key from red-teamed policy dictionary
        key = random.choice(list(self.red_team.policy_data.keys()))

        # get factual policy point
        pol_point = self.red_team.policy_data[key]

        # get conditions, consequences, and action
        conditions = pol_point.get_policy_data_point_condition_names()
        consequences, f_conseqs = pol_point.get_policy_data_point_consequence_names()
        f_action = pol_point.get_policy_data_point_action_name()

        # create counter factual action space by removing factual action
        cf_action_space = [act for act in action_space if act != f_action]

        # get random counter factual action
        cf_action = random.choice(cf_action_space)

        return conditions, consequences, f_action, f_conseqs, cf_action

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
        state_space = sorted(state_space)
        conseq_space = self.red_team.get_consequence_state_space()
        conseq_space = sorted(conseq_space)
        action_space = self.red_team.get_action_space()
        action_space = sorted(action_space)

        # generate random scenario
        red_team_conditions, red_team_consequences = self.get_random_red_teamed_scenario(state_space, conseq_space)

        # check if auto-generating data
        if self.auto_gen_data:
            succ = self.__auto_generate_new_risky_scenario_data_point(red_team_conditions, red_team_consequences)
            # check if successfully auto-generated data point; otherwise, continue with data generation
            if succ:
                return

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
        # get state and action space
        state_space = self.red_team.get_state_space()
        state_space = sorted(state_space)
        conseq_space = self.red_team.get_consequence_state_space()
        conseq_space = sorted(conseq_space)
        action_space = self.red_team.get_action_space()
        action_space = sorted(action_space)

        # get random counter factual scenario from policy
        output = self.get_random_counter_factual_scenario_action(state_space, conseq_space, action_space)
        # unpack
        conditions, consequences, f_action, f_conseqs, cf_action = output

        # check if auto-generating data
        if self.auto_gen_data:
            succ = self.__auto_generate_new_counter_factual_data_point(conditions, consequences, f_action, f_conseqs, cf_action)
            # check if successfully auto-generated data point; otherwise, continue with data generation
            if succ:
                return

        # get consequences from user input
        output = UIConseq.get_counter_factual_consequences_from_user(self.red_team, conditions, consequences, cf_action, conseq_space)
        # unpack
        self.continue_data_generation, abort, cf_conseqs = output
        # check if aborting this data point
        if abort:
            return

        # create policy data point
        pol_point = CounterFactualPolicyDataPoint(conditions=conditions,
                                                  consequences_before_action=consequences,
                                                  action=cf_action,
                                                  consequences_after_action=cf_conseqs)

        # update policy
        CLP.print_update_policy_message(conditions, consequences, cf_action, cf_conseqs)
        self.red_team.update_counter_factual_policy(pol_point)

        # check if policy needs to be written to file
        if (self.get_points_generated() != 0) and ((self.get_points_generated() % self.save_new_policy_points) == 0):
            self.write_policy_to_file()

        return

    def __auto_generate_new_risky_scenario_data_point(self, condition_names,
                                                            pre_action_consequence_names):
        # initialize result
        res = RiskyScenarioDataGenerationResponse()

        # try service call
        try:
            res = self.rs_auto_data_gen_client(condition_names,
                                               pre_action_consequence_names)
        except rospy.ServiceException as e:
            rospy.logwarn("[Red Team Data Extension] Data point generation service call failed: %s", e)
            rospy.loginfo("[Red Team Data Extension] Requesting input from user")
            return False

        # got result! check success
        if not res.success:
            rospy.logwarn("[Red Team Data Extension] Automatic generation of data point failed")
            rospy.loginfo("[Red Team Data Extension] Requesting input from user")
            return False

        # create policy data point
        pol_point = RiskMitigatingPolicyDataPoint(conditions=condition_names,
                                                  consequences_before_action=pre_action_consequence_names,
                                                  action=res.action_name,
                                                  consequences_after_action=res.post_action_consequence_names)

        # check for conflicts
        if (pol_point.check_conflicting_data_point_action(self.red_team.policy_data) or
            pol_point.check_conflicting_data_point_consequences(self.red_team.policy_data)):
            rospy.logwarn("[Red Team Data Extension] Automatic generation of data point resulted in conflicts with policy")
            rospy.loginfo("[Red Team Data Extension] Requesting input from user")
            return False

        # update policy
        rospy.loginfo("[Red Team Data Extension] Automatically generated new %s data point!", self.get_mode_name())
        CLP.print_update_policy_message(condition_names, pre_action_consequence_names, res.action_name, res.post_action_consequence_names)
        self.red_team.update_policy(pol_point)

        # check if policy needs to be written to file
        if (self.get_points_generated() != 0) and ((self.get_points_generated() % self.save_new_policy_points) == 0):
            self.write_policy_to_file()

        return True

    def __auto_generate_new_counter_factual_data_point(self, condition_names,
                                                             pre_action_consequence_names,
                                                             factual_action_name,
                                                             factual_post_action_consequence_names,
                                                             counter_factual_action_name):
        # initialize result
        res = CounterFactualDataGenerationResponse()

        # try service call
        try:
            res = self.cf_auto_data_gen_client(condition_names,
                                               pre_action_consequence_names,
                                               factual_action_name,
                                               factual_post_action_consequence_names,
                                               counter_factual_action_name)
        except rospy.ServiceException as e:
            rospy.logwarn("[Red Team Data Extension] Data point generation service call failed: %s", e)
            rospy.loginfo("[Red Team Data Extension] Requesting input from user")
            return False

        # got result! check success
        if not res.success:
            rospy.logwarn("[Red Team Data Extension] Automatic generation of data point failed")
            rospy.loginfo("[Red Team Data Extension] Requesting input from user")
            return False

        # create policy data point
        pol_point = CounterFactualPolicyDataPoint(conditions=condition_names,
                                                  consequences_before_action=pre_action_consequence_names,
                                                  action=counter_factual_action_name,
                                                  consequences_after_action=res.post_action_consequence_names)

        # update policy
        rospy.loginfo("[Red Team Data Extension] Automatically generated new %s data point!", self.get_mode_name())
        CLP.print_update_policy_message(condition_names, pre_action_consequence_names, counter_factual_action_name, res.post_action_consequence_names)
        self.red_team.update_counter_factual_policy(pol_point)

        # check if policy needs to be written to file
        if (self.get_points_generated() != 0) and ((self.get_points_generated() % self.save_new_policy_points) == 0):
            self.write_policy_to_file()

        return True

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
    auto_gen_data = rospy.get_param(param_prefix + 'auto_gen_data', True)
    rs_auto_data_gen_service_name = rospy.get_param(param_prefix + 'auto_data_gen_service', "")
    cf_auto_data_gen_service_name = rospy.get_param(param_prefix + 'auto_cf_data_gen_service', "")

    # initialize node
    rospy.init_node(node_name)

    # create red team node
    rospy.loginfo("[Red Team Data Extension] Creating human-robot red team data extension node...")
    red_team = RedTeamDataExtension(robot=robot_name, environment=env_name,
                                    num_points=num_points,
                                    max_conds=max_conds_per_point,
                                    counter_factual_mode=cf_mode,
                                    auto_gen_data=auto_gen_data,
                                    rs_auto_data_gen_service_name=rs_auto_data_gen_service_name,
                                    cf_auto_data_gen_service_name=cf_auto_data_gen_service_name)
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
          not red_team.check_possible_points_generated() and \
          red_team.check_continue_data_generation():
        rospy.loginfo("[Red Team Data Extension] Generating new red teamed %s data point...", red_team.get_mode_name())
        print()
        red_team.generate_new_data_point()
        rate.sleep()

    # check stopping conditions
    rospy.loginfo("[Red Team Data Extension] Total policy points %d of %d possible data points that could be generated",
                  red_team.get_total_data_points(), red_team.get_num_possible_data_points())
    if red_team.check_possible_points_generated():
        rospy.loginfo("[Red Team Data Extension] Red teamed data is already completed. There are %d possible data points and %d have been generated through human-robot red teaming!",
                      red_team.get_num_possible_data_points(), red_team.get_total_data_points())
    elif red_team.check_points_generated():
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
