#!/usr/bin/env python3
"""
Risk Mitigating Policy Data Reader
    includes class RiskMitigatingPolicyDataReader, which can be used in other nodes
    optionally can be run as a standalone script to test parsing of risk mitigating policy data
Emily Sheetz, NSTGRO VTE 2024
"""

import rospy

import os
import yaml

from yaml_formatting_checks import YAMLPolicyDataChecks as YAMLChecks

from risk_mitigating_policy_data_point import RiskMitigatingPolicyDataPoint

class RiskMitigatingPolicyDataReader:
    def __init__(self, robot="val", environment="lunar_habitat", human_gen_data=True):
        # set internal parameters
        self.robot_name = robot
        self.environment_name = environment

        # get path of this script
        script_path = os.path.abspath(os.path.dirname( __file__ ))

        # set risk mitigating policy file name and path
        if human_gen_data:
            # read human-generated data in config/ directory
            self.risk_mitigating_policy_data_file = "risk_mitigating_policy_data.yaml"
            self.risk_mitigating_policy_data_full_path = script_path + "/../config/" + self.robot_name + "/" + self.risk_mitigating_policy_data_file
        else:
            # read human-robot red team generated data in data/ directory
            self.risk_mitigating_policy_data_file = "red_teamed_risk_mitigating_policy_data.yaml"
            self.risk_mitigating_policy_data_full_path = script_path + "/../data/" + self.robot_name + "/" + self.risk_mitigating_policy_data_file

        # initialize dictionary of conditions to actions
        self.risk_mitigating_policy = {}

        # initialize flag for valid policy
        self.valid_policy = False

    #######################
    ### GETTERS/SETTERS ###
    #######################

    def get_robot_name(self):
        return self.robot_name

    def get_environment_name(self):
        return self.environment_name

    def get_risk_mitigating_policy_data_file_path(self):
        return self.risk_mitigating_policy_data_full_path

    def get_risk_mitigating_policy_data(self):
        return self.risk_mitigating_policy

    def get_num_risk_mitigating_policy_data(self):
        return len(self.risk_mitigating_policy.keys())

    ###########################################
    ### PROCESS RISK MITIGATING POLICY DATA ###
    ###########################################

    def process_risk_mitigating_policy_data(self):
        # clear out risk mitigating policy dictionary
        self.risk_mitigating_policy = {}

        # verify YAML file exists
        valid_path = YAMLChecks.check_yaml_existence(self.risk_mitigating_policy_data_full_path)
        if not valid_path:
            print("ERROR: risk mitigating policy data file " + self.risk_mitigating_policy_data_full_path + " does not exist")
            self.valid_policy = False
            return

        # open YAML file and load dict
        fo = open(self.risk_mitigating_policy_data_full_path)
        yaml_dict = yaml.load(fo, Loader=yaml.FullLoader)

        # error check YAML file formatting
        valid_yaml = YAMLChecks.check_risk_mitigating_policy_data_yaml_formatting(yaml_dict, self.environment_name)
        if not valid_yaml:
            print("ERROR: risk mitigating policy data file " + self.risk_mitigating_policy_data_full_path + " is poorly formatted")
            self.valid_policy = False
            return

        # initialize valid policy flag
        self.valid_policy = True

        # get list of policy data data for environment
        policy = yaml_dict[self.environment_name]['policy_data']

        # process each policy data
        for i in range(len(policy)):
            # get policy data
            pol = policy[i]

            # check valid values for policy
            valid_policy = YAMLChecks.check_valid_risk_mitigating_policy_data_values(pol, i, len(policy))
            self.valid_policy = self.valid_policy and valid_policy

            # create policy
            risk_pol = RiskMitigatingPolicyDataPoint(conditions=pol['conditions'],
                                                     consequences_before_action=pol['consequences_before_action'],
                                                     action=pol['action'],
                                                     consequences_after_action=pol['consequences_after_action'])
            risk_conds = risk_pol.get_policy_data_point_condition_names()
            risk_act = risk_pol.get_policy_data_point_action_name()
            risk_conseq_bef, risk_conseq_aft = risk_pol.get_policy_data_point_consequence_names()

            # check for conflicting data point already in policy
            act_conflict, risk_act, policy_risk_act = risk_pol.check_and_get_conflicting_data_point_action(self.risk_mitigating_policy)
            conseq_conflict, risk_conseq, policy_risk_conseq = risk_pol.check_and_get_conflicting_data_point_consequences(self.risk_mitigating_policy)
            if act_conflict or conseq_conflict:
                print("ERROR: found duplicate conditions with different actions/consequences; please resolve conflict in risk mitigating policy data file " + self.risk_mitigating_policy_data_full_path)
                print("    Conditions:", risk_conds)
                print("    Action:", policy_risk_act, "    Consequences:", policy_risk_conseq)
                print("    Found new action:", risk_act, "    New consequences:", risk_conseq)
                self.valid_policy = False
                # do not add duplicate conditions into dictionary
                continue

            # add policy data to dictionary
            self.risk_mitigating_policy[risk_pol.get_policy_data_point_dictionary_key()] = risk_pol

        # close file
        fo.close()

        return

    def check_valid_policy(self):
        return self.valid_policy

    #######################################
    ### RISK MITIGATING POLICY PRINTING ###
    #######################################

    def print_risk_mitigating_policy_data(self):
        # compute number of policy data points
        num_pols = len(self.risk_mitigating_policy.keys())

        # print starting message
        print()
        print("Read " + str(num_pols) + " risk mitigating policy data points for robot " + self.robot_name.upper() + " in " + self.environment_name.upper() + " environment")

        i = 0
        for pol_conds in self.risk_mitigating_policy.keys():
            # get policy data point
            pol_data_point = self.risk_mitigating_policy[pol_conds]
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
    node_name = "RiskMitigatingPolicyDataReader"
    param_prefix = "/" + node_name + "/"

    # get ROS parameters
    robot_name = rospy.get_param(param_prefix + 'robot', "val")
    env_name = rospy.get_param(param_prefix + 'environment', "lunar_habitat")
    human_gen_data = rospy.get_param(param_prefix + 'human_generated_data', True)

    # initialize node
    rospy.init_node(node_name)

    # create risk mitigating policy data reader
    rospy.loginfo("[Risk Mitigating Policy Data Reader] Creating risk mitigating policy data reader...")
    policy_reader = RiskMitigatingPolicyDataReader(robot=robot_name, environment=env_name, human_gen_data=human_gen_data)

    # read risk mitigating policy data
    rospy.loginfo("[Risk Mitigating Policy Data Reader] Reading risk mitigating policy data for robot %s in %s environment...",
                  policy_reader.get_robot_name(), policy_reader.get_environment_name())
    rospy.loginfo("[Risk Mitigating Policy Data Reader] Trying to read risk mitigating policy data file '%s'...",
                  policy_reader.get_risk_mitigating_policy_data_file_path())
    policy_reader.process_risk_mitigating_policy_data()

    # check if policy is valid
    if policy_reader.check_valid_policy():
        rospy.loginfo("[Risk Mitigating Policy Data Reader] Risk mitigating policy data processed successfully!")
        rospy.loginfo("[Risk Mitigating Policy Data Reader] Processed %d risk mitigating policy data points",
                      policy_reader.get_num_risk_mitigating_policy_data())
    else:
        rospy.logwarn("[Risk Mitigating Policy Data Reader] Risk mitigating policy data not processed successfully. Please review errors to fix.")

    # print risk mitigating policy data
    policy_reader.print_risk_mitigating_policy_data()

    rospy.loginfo("[Risk Mitigating Policy Data Reader] Node stopped, all done!")
