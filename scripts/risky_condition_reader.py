#!/usr/bin/env python3
"""
Risky Condition Reader
    includes class RiskyConditionReader, which can be used in other nodes
    optionally can be run as a standalone script to test parsing of risky conditions
Emily Sheetz, NSTGRO VTE 2024
"""

import rospy

import os
import yaml

from likelihood_consequence_risk import LikelihoodLevels, ConsequenceClasses
from risky_condition import RiskyCondition

class RiskyConditionReader:
    def __init__(self, robot="val", environment="lunar_habitat"):
        # set internal parameters
        self.robot_name = robot
        self.environment_name = environment

        # get path of this script
        script_path = os.path.abspath(os.path.dirname( __file__ ))

        # set risky condition file name and path
        self.risky_condition_file = "risky_conditions.yaml"
        self.risky_condition_full_path = script_path + "/../config/" + self.robot_name + "/" + self.risky_condition_file

        # initialize list of risky conditions
        self.risky_conditions = []

        # initialize flag for valid conditions
        self.valid_conditions = False

    #######################
    ### GETTERS/SETTERS ###
    #######################

    def get_robot_name(self):
        return self.robot_name

    def get_environment_name(self):
        return self.environment_name

    def get_risky_condition_file_path(self):
        return self.risky_condition_full_path

    def get_risky_conditions(self):
        return self.risky_conditions

    def get_num_risky_conditions(self):
        return len(self.risky_conditions)

    ################################
    ### PROCESS RISKY CONDITIONS ###
    ################################

    def process_risky_conditions(self):
        # clear out risky conditions list
        self.risky_conditions = []

        # verify YAML file exists
        valid_path = self.error_check_yaml_existence()
        if not valid_path:
            print("ERROR: risky condition file " + self.risky_condition_full_path + " does not exist")
            self.valid_conditions = False
            return

        # open YAML file and load dict
        fo = open(self.risky_condition_full_path)
        yaml_dict = yaml.load(fo, Loader=yaml.FullLoader)

        # error check YAML file formatting
        valid_yaml = self.error_check_yaml_formatting(yaml_dict)
        if not valid_yaml:
            print("ERROR: risky condition file " + self.risky_condition_full_path + " is poorly formatted")
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

            # check valid values for risky condition
            valid_condition = self.error_check_valid_condition_values(cond, i, len(conditions))
            self.valid_conditions = self.valid_conditions and valid_condition

            # create risky condition
            risky_cond = RiskyCondition(name=cond['name'],
                                        likelihood=cond['likelihood'],
                                        consequence=cond['consequence'])

            # add risky condition to list
            self.risky_conditions.append(risky_cond)

        return

    def check_valid_conditions(self):
        return self.valid_conditions

    ######################################
    ### RISKY CONDITION ERROR CHECKING ###
    ######################################

    def error_check_yaml_existence(self):
        # check if YAML file exists and is a file
        return (os.path.exists(self.risky_condition_full_path) and
                os.path.isfile(self.risky_condition_full_path))

    def error_check_yaml_formatting(self, yaml_dict):
        # check if environment exists
        if self.environment_name not in yaml_dict.keys():
            print("ERROR: environment " + self.environment_name + " does not exist in risky conditions file")
            return False

        # check for list of conditions
        if 'conditions' not in yaml_dict[self.environment_name].keys():
            print("ERROR: environment " + self.environment_name + " has no risky conditions defined under key 'conditions'")
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
                print("ERROR: risky condition " + str(i) + " of " + str(num_conds) + " does not have a name")
                valid_conditions = False

            # check for likelihood
            if 'likelihood' not in condition.keys():
                print("ERROR: risky condition " + str(i) + " of " + str(num_conds) + " does not have a likelihood value")
                valid_conditions = False

            # check for consequence
            if 'consequence' not in condition.keys():
                print("ERROR: risky condition " + str(i) + " of " + str(num_conds) + " does not have a consequence value")
                valid_conditions = False

        return valid_conditions

    def error_check_valid_condition_values(self, cond, i, num_conds):
        # initialize valid values flag
        valid_values = True

        # check for valid name
        if not type(cond['name']) == str:
            print("WARN: non-string name for risky condition " + str(i) + " of " + str(num_conds))
            valid_values = False

        # check for valid likelihood and consequence scores
        if not LikelihoodLevels.valid_value(cond['likelihood']):
            print("WARN: invalid likelihood value for risky condition " + str(i) + " of " + str(num_conds))
            valid_values = False

        if not ConsequenceClasses.valid_value(cond['consequence']):
            print("WARN: invalid consequence value for risky condition " + str(i) + " of " + str(num_conds))
            valid_values = False

        return valid_values

    ################################
    ### RISKY CONDITION PRINTING ###
    ################################

    def print_risky_conditions(self):
        # compute number of conditions
        num_conds = len(self.risky_conditions)

        # print starting message
        print()
        print("Read " + str(num_conds) + " risky conditions for robot " + self.robot_name.upper() + " in " + self.environment_name.upper() + " environment")        

        for i in range(num_conds):
            # get condition
            cond = self.risky_conditions[i]
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
    node_name = "RiskyConditionReader"
    param_prefix = "/" + node_name + "/"

    # get ROS parameters
    robot_name = rospy.get_param(param_prefix + 'robot', "val")
    env_name = rospy.get_param(param_prefix + 'environment', "lunar_habitat")

    # initialize node
    rospy.init_node(node_name)

    # create risky condition reader with default values
    rospy.loginfo("[Risky Condition Reader] Creating risky condition reader...")
    risky_reader = RiskyConditionReader(robot=robot_name, environment=env_name)

    # read risky conditions
    rospy.loginfo("[Risky Condition Reader] Reading risky conditions for robot %s in %s environment...",
                  risky_reader.get_robot_name(), risky_reader.get_environment_name())
    rospy.loginfo("[Risky Condition Reader] Trying to read risky condition file '%s'...",
                  risky_reader.get_risky_condition_file_path())
    risky_reader.process_risky_conditions()

    # check if conditions are valid
    if risky_reader.check_valid_conditions():
        rospy.loginfo("[Risky Condition Reader] Risky conditions processed successfully!")
        rospy.loginfo("[Risky Condition Reader] Processed %d risky conditions",
                      risky_reader.get_num_risky_conditions())
    else:
        rospy.logwarn("[Risky Condition Reader] Risky conditions not processed successfully. Please review errors to fix.")

    # print risky conditions
    risky_reader.print_risky_conditions()

    rospy.loginfo("[Risky Condition Reader] Node stopped, all done!")
