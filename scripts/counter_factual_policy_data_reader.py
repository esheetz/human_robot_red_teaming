#!/usr/bin/env python3
"""
Counter Factual Policy Data Reader
    includes class CounterFactualPolicyDataReader, which can be used in other nodes
    optionally can be run as a standalone script to test parsing of counter factual policy data
Emily Sheetz, NSTGRO VTE 2024
"""

import rospy

import os
import yaml

from yaml_formatting_checks import YAMLPolicyDataChecks as YAMLChecks

from counter_factual_policy_data_point import CounterFactualPolicyDataPoint

class CounterFactualPolicyDataReader:
    def __init__(self, robot="val", environment="lunar_habitat"):
        # set internal parameters
        self.robot_name = robot
        self.environment_name = environment

        # get path of this script
        script_path = os.path.abspath(os.path.dirname( __file__ ))

        # set counter factual policy file name and path
        self.counter_factual_policy_data_file = "counter_factual_policy_data.yaml"
        self.counter_factual_policy_data_full_path = script_path + "/../data/" + self.robot_name + "/" + self.counter_factual_policy_data_file
        self.policy_file_exists = False

        # initialize list of counter-factual policy points
        self.counter_factual_policy = []

        # initialize flag for valid policy
        self.valid_policy = False

    #######################
    ### GETTERS/SETTERS ###
    #######################

    def get_robot_name(self):
        return self.robot_name

    def get_environment_name(self):
        return self.environment_name

    def get_counter_factual_policy_data_file_path(self):
        return self.counter_factual_policy_data_full_path

    def get_counter_factual_policy_data(self):
        return self.counter_factual_policy

    def get_num_counter_factual_policy_data(self):
        return len(self.counter_factual_policy)

    ###########################################
    ### PROCESS COUNTER FACTUAL POLICY DATA ###
    ###########################################

    def process_counter_factual_policy_data(self):
        # clear out counter factual policy list
        self.counter_factual_policy = []

        # verify YAML file exists
        valid_path = YAMLChecks.check_yaml_existence(self.counter_factual_policy_data_full_path)
        if not valid_path:
            print("WARN: counter factual policy data file " + self.counter_factual_policy_data_full_path + " does not exist; initializing to empty policy")
            self.policy_file_exists = False
            self.valid_policy = True
            return

        # open YAML file and load dict
        self.policy_file_exists = True
        fo = open(self.counter_factual_policy_data_full_path)
        yaml_dict = yaml.load(fo, Loader=yaml.FullLoader)

        # error check YAML file formatting
        valid_yaml = YAMLChecks.check_policy_data_yaml_formatting(yaml_dict, self.environment_name, "counter factual policy data")
        if not valid_yaml:
            print("ERROR: counter factual policy data file " + self.counter_factual_policy_data_full_path + " is poorly formatted")
            self.valid_policy = False
            return

        # initialize valid policy flag
        self.valid_policy = True

        # get list of policy data for environment
        policy = yaml_dict[self.environment_name]['policy_data']

        # process each policy data
        for i in range(len(policy)):
            # get policy data
            pol = policy[i]

            # check valid values for policy
            valid_policy = YAMLChecks.check_valid_policy_data_values(pol, i, len(policy))
            self.valid_policy = self.valid_policy and valid_policy

            # create policy
            cf_pol = CounterFactualPolicyDataPoint(conditions=pol['conditions'],
                                                   consequences_before_action=pol['consequences_before_action'],
                                                   action=pol['action'],
                                                   consequences_after_action=pol['consequences_after_action'])

            # add policy data to list
            self.counter_factual_policy.append(cf_pol)

        # close file
        fo.close()

        return

    def check_valid_policy(self):
        return self.valid_policy

    #######################################
    ### COUNTER FACTUAL POLICY PRINTING ###
    #######################################

    def print_counter_factual_policy_data(self):
        # compute number of policy data points
        num_pols = len(self.counter_factual_policy)

        # print starting message
        print()
        print("Read " + str(num_pols) + " counter factual policy data points for robot " + self.robot_name.upper() + " in " + self.environment_name.upper() + " environment")

        i = 0
        for pol_data_point in self.counter_factual_policy:
            # get conditions and action
            conds = pol_data_point.get_policy_data_point_condition_names()
            act = pol_data_point.get_policy_data_point_action_name()
            conseq_bef, conseq_aft = pol_data_point.get_policy_data_point_consequence_names()
            # print info
            print("    Policy Data Point " + str(i) + " of " + str(num_pols) + ":")
            print("        conditions:")
            for cond in conds:
                print("          - " + cond)
            if len(conseq_bef) == 0:
                print("        consequences before action: NONE")
            else:
                print("        consequences before action:")
                for conseq in conseq_bef:
                    print("          - " + conseq)
            print("        action: " + act)
            if len(conseq_aft) == 0:
                print("        consequences after action: NONE")
            else:
                print("        consequences after action:")
                for conseq in conseq_aft:
                    print("          - " + conseq)
            i += 1
        print()

        return



#####################
### MAIN FUNCTION ###
#####################

if __name__ == '__main__':
    # set node name
    node_name = "CounterFactualPolicyDataReader"
    param_prefix = "/" + node_name + "/"

    # get ROS parameters
    robot_name = rospy.get_param(param_prefix + 'robot', "val")
    env_name = rospy.get_param(param_prefix + 'environment', "lunar_habitat")

    # initialize node
    rospy.init_node(node_name)

    # create counter factual policy data reader
    rospy.loginfo("[Counter Factual Policy Data Reader] Creating counter factual policy data reader...")
    policy_reader = CounterFactualPolicyDataReader(robot=robot_name, environment=env_name)

    # read counter factual policy data
    rospy.loginfo("[Counter Factual Policy Data Reader] Reading counter factual policy data for robot %s in %s environment...",
                  policy_reader.get_robot_name(), policy_reader.get_environment_name())
    rospy.loginfo("[Counter Factual Policy Data Reader] Trying to read counter factual policy data file '%s'...",
                  policy_reader.get_counter_factual_policy_data_file_path())
    policy_reader.process_counter_factual_policy_data()

    # check if policy is valid
    if policy_reader.check_valid_policy():
        rospy.loginfo("[Counter Factual Policy Data Reader] Counter factual policy data processed successfully!")
        rospy.loginfo("[Counter Factual Policy Data Reader] Processed %d counter factual policy data points",
                      policy_reader.get_num_counter_factual_policy_data())
    else:
        rospy.logwarn("[Counter Factual Policy Data Reader] Counter factual policy data not processed successfully. Please review errors to fix.")

    # print counter factual policy data
    policy_reader.print_counter_factual_policy_data()

    rospy.loginfo("[Counter Factual Policy Data Reader] Node stopped, all done!")
