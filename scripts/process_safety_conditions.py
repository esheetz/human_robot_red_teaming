#!/usr/bin/env python3
"""
Process Safety Conditions
    includes class SafetyConditionReader, which can be used in other nodes
    optionally can be run as a standalone script to test parsing of safety conditions
Emily Sheetz, NSTGRO VTE 2024
"""

import rospy

import os
import yaml

from likelihood_consequence_risk import LikelihoodLevels, ConsequenceClasses
from safety_condition import SafetyCondition

class SafetyConditionReader:
    def __init__(self, robot="val", environment="lunar_habitat"):
        # set internal parameters
        self.robot_name = robot
        self.environment_name = environment

        # get path of this script
        script_path = os.path.abspath(os.path.dirname( __file__ ))

        # set safety condition file name and path
        self.safety_condition_file = "safety_conditions.yaml"
        self.safety_condition_full_path = script_path + "/../config/" + self.robot_name + "/safety_conditions.yaml"

        # initialize list of safety conditions
        self.safety_conditions = []

        # initialize flag for valid conditions
        self.valid_conditions = False

    #######################
    ### GETTERS/SETTERS ###
    #######################

    def get_robot_name(self):
        return self.robot_name

    def get_environment_name(self):
        return self.environment_name

    def get_safety_condition_file_path(self):
        return self.safety_condition_full_path

    def get_safety_conditions(self):
        return self.safety_conditions

    def get_num_safety_conditions(self):
        return len(self.safety_conditions)

    #################################
    ### PROCESS SAFETY CONDITIONS ###
    #################################

    def process_safety_conditions(self):
        # clear out safety conditions list
        self.safety_conditions = []

        # verify YAML file exists
        valid_path = self.error_check_yaml_existence()
        if not valid_path:
            print("ERROR: safety condition file " + self.safety_condition_full_path + " does not exist")
            self.valid_conditions = False
            return

        # open YAML file and load dict
        fo = open(self.safety_condition_full_path)
        yaml_dict = yaml.load(fo, Loader=yaml.FullLoader)

        # error check YAML file formatting
        valid_yaml = self.error_check_yaml_formatting(yaml_dict)
        if not valid_yaml:
            print("ERROR: safety condition file " + self.safety_condition_full_path + " is poorly formatted")
            self.valid_conditions = False
            return

        # initialize valid conditions flag
        self.valid_conditions = True

        # get list of conditions for environment
        conditions = yaml_dict[self.environment_name]['conditions']

        # process each condition
        for i in range(len(conditions)):
            # get condition
            cond = conditions[i]

            # check valid values for safety condition
            valid_condition = self.error_check_valid_condition_values(cond, i, len(conditions))
            self.valid_conditions = self.valid_conditions and valid_condition

            # create safety condition
            safety_cond = SafetyCondition(name=cond['name'],
                                          likelihood=cond['likelihood'],
                                          consequence=cond['consequence'])

            # add safety condition to list
            self.safety_conditions.append(safety_cond)

        return

    def check_valid_conditions(self):
        return self.valid_conditions

    #######################################
    ### SAFETY CONDITION ERROR CHECKING ###
    #######################################

    def error_check_yaml_existence(self):
        # check if YAML file exists and is a file
        return (os.path.exists(self.safety_condition_full_path) and
                os.path.isfile(self.safety_condition_full_path))

    def error_check_yaml_formatting(self, yaml_dict):
        # check if environment exists
        if self.environment_name not in yaml_dict.keys():
            print("ERROR: environment " + self.environment_name + " does not exist in safety conditions file")
            return False

        # check for list of conditions
        if 'conditions' not in yaml_dict[self.environment_name].keys():
            print("ERROR: environment " + self.environment_name + " has no safety conditions defined under key 'conditions'")
            return False

        # get number of conditions
        num_conds = len(yaml_dict[self.environment_name]['conditions'])

        # initialize valid condition flag
        valid_conditions = True

        # check each condition in list
        for i in range(num_conds):
            # get condition
            condition = yaml_dict[self.environment_name]['conditions'][i]

            # check for name
            if 'name' not in condition.keys():
                print("ERROR: safety condition " + str(i) + " of " + str(num_conds) + " does not have a name")
                valid_conditions = False

            # check for likelihood
            if 'likelihood' not in condition.keys():
                print("ERROR: safety condition " + str(i) + " of " + str(num_conds) + " does not have a likelihood value")
                valid_conditions = False

            # check for consequence
            if 'consequence' not in condition.keys():
                print("ERROR: safety condition " + str(i) + " of " + str(num_conds) + " does not have a consequence value")
                valid_conditions = False

        return valid_conditions

    def error_check_valid_condition_values(self, cond, i, num_conds):
        # initialize valid values flag
        valid_values = True

        # check for valid name
        if not type(cond['name']) == str:
            print("WARN: non-string name for safety condition " + str(i) + " of " + str(num_conds))
            valid_values = False

        # check for valid likelihood and consequence scores
        if not LikelihoodLevels.valid_value(cond['likelihood']):
            print("WARN: invalid likelihood value for safety condition " + str(i) + " of " + str(num_conds))
            valid_values = False

        if not ConsequenceClasses.valid_value(cond['consequence']):
            print("WARN: invalid consequence value for safety condition " + str(i) + " of " + str(num_conds))
            valid_values = False

        return valid_values

    #################################
    ### SAFETY CONDITION PRINTING ###
    #################################

    def print_safety_conditions(self):
        # compute number of conditions
        num_conds = len(self.safety_conditions)

        # print starting message
        print()
        print("Read " + str(num_conds) + " safety conditions for robot " + self.robot_name.upper() + " in " + self.environment_name.upper() + " environment")        

        for i in range(num_conds):
            # get condition
            cond = self.safety_conditions[i]
            # print condition info
            print("    Condition " + str(i) + " of " + str(num_conds) + ":")
            print("        name: " + cond.get_condition_name())
            print("        likelihood: " + str(cond.get_likelihood_level()) + " (" + cond.get_likelihood_level_name() + ")")
            print("        consequence: " + str(cond.get_consequence_class()) + " (" + cond.get_consequence_class_name() + ")")
            print("        RISK LEVEL: " + cond.get_risk_score_name().upper())
            print("            risk score: " + str(cond.get_risk_score()))
            print("            safety score: " + str(cond.get_safety_score()))
        print()

        return



#####################
### MAIN FUNCTION ###
#####################

if __name__ == '__main__':
    # set node name
    node_name = "ProcessSafetyConditions"
    param_prefix = "/" + node_name + "/"

    # get ROS parameters
    robot_name = rospy.get_param(param_prefix + 'robot', "val")
    env_name = rospy.get_param(param_prefix + 'environment', "lunar_habitat")

    # initialize node
    rospy.init_node("ProcessSafetyConditions")

    # create safety condition reader with default values
    rospy.loginfo("[Process Safety Conditions] Creating safety condition reader...")
    safety_reader = SafetyConditionReader(robot=robot_name, environment=env_name)

    # read safety conditions
    rospy.loginfo("[Process Safety Conditions] Reading safety conditions for robot %s in %s environment...",
                  safety_reader.get_robot_name(), safety_reader.get_environment_name())
    rospy.loginfo("[Process Safety Conditions] Trying to read safety condition file '%s'...",
                  safety_reader.get_safety_condition_file_path())
    safety_reader.process_safety_conditions()

    # check if conditions are valid
    if safety_reader.check_valid_conditions():
        rospy.loginfo("[Process Safety Conditions] Safety conditions processed successfully!")
        rospy.loginfo("[Process Safety Conditions] Processed %d safety conditions",
                      safety_reader.get_num_safety_conditions())
    else:
        rospy.logwarn("[Process Safety Conditions] Safety conditions not processed successfully. Please review errors to fix.")

    # print safety conditions
    safety_reader.print_safety_conditions()

    rospy.loginfo("[Process Safety Conditions] Node stopped, all done!")
