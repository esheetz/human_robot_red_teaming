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

from yaml_formatting_checks import YAMLStateSpaceChecks as YAMLChecks

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

    def get_risky_condition_names(self):
        return [cond.get_condition_name() for cond in self.risky_conditions]

    def get_risky_condition_with_name(self, condition_name):
        # look through risky conditions
        for cond in self.risky_conditions:
            # check name
            if cond.get_condition_name() == condition_name:
                return cond
        # if we get here, no condition in list has given name
        return None

    def get_num_risky_conditions(self):
        return len(self.risky_conditions)

    ################################
    ### PROCESS RISKY CONDITIONS ###
    ################################

    def process_risky_conditions(self):
        # clear out risky conditions list
        self.risky_conditions = []

        # verify YAML file exists
        valid_path = YAMLChecks.check_yaml_existence(self.risky_condition_full_path)
        if not valid_path:
            print("ERROR: risky condition file " + self.risky_condition_full_path + " does not exist")
            self.valid_conditions = False
            return

        # open YAML file and load dict
        fo = open(self.risky_condition_full_path)
        yaml_dict = yaml.load(fo, Loader=yaml.FullLoader)

        # error check YAML file formatting
        valid_yaml = YAMLChecks.check_risky_condition_yaml_formatting(yaml_dict, self.environment_name)
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
            valid_condition = YAMLChecks.check_valid_risky_condition_values(cond, i, len(conditions))
            self.valid_conditions = self.valid_conditions and valid_condition

            # create risky condition
            risky_cond = RiskyCondition(name=cond['name'],
                                        likelihood=cond['likelihood'],
                                        consequence=cond['consequence'],
                                        consequence_states=cond['consequence_states'])

            # add risky condition to list
            self.risky_conditions.append(risky_cond)

        # close file
        fo.close()

        return

    def check_valid_conditions(self):
        return self.valid_conditions

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
            # get consequence states
            conseq_states = cond.get_consequence_states()
            # print condition info
            print("    Condition " + str(i) + " of " + str(num_conds) + ":")
            print("        name: " + cond.get_condition_name())
            print("        likelihood: " + str(cond.get_likelihood_level()) + " (" + cond.get_likelihood_level_name() + ")")
            print("        consequence: " + str(cond.get_consequence_class()) + " (" + cond.get_consequence_class_name() + ")")
            print("        predicted future consequence states:")
            for state in conseq_states:
                print("            - " + state)
            print("        RISK ASSESSMENT MATRIX (RAM) RISK LEVEL: " + cond.get_matrix_risk_score_name().upper())
            print("            RAM risk score: " + str(cond.get_matrix_risk_score()))
            print("            RAM safety score: " + str(cond.get_matrix_safety_score()))
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

    # create risky condition reader
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
