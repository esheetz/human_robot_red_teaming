#!/usr/bin/env python3
"""
Risk Mitigating Action Reader
    includes class RiskMitigatingActionReader, which can be used in other nodes
    optionally can be run as a standalone script to test parsing of risk mitigating actions
Emily Sheetz, NSTGRO VTE 2024
"""

import rospy

import os
import yaml

from yaml_formatting_checks import YAMLActionSpaceChecks as YAMLChecks

from risk_mitigating_action import RiskMitigatingAction

class RiskMitigatingActionReader:
    def __init__(self, robot="val", environment="lunar_habitat"):
        # set internal parameters
        self.robot_name = robot
        self.environment_name = environment

        # get path of this script
        script_path = os.path.abspath(os.path.dirname( __file__ ))

        # set risk mitigating action file name and path
        self.risk_mitigating_action_file = "risk_mitigating_actions.yaml"
        self.risk_mitigating_action_full_path = script_path + "/../config/" + self.robot_name + "/" + self.risk_mitigating_action_file

        # initialize list of risk mitigating actions
        self.risk_mitigating_actions = []

        # initialize flag for valid actions
        self.valid_actions = False

    #######################
    ### GETTERS/SETTERS ###
    #######################

    def get_robot_name(self):
        return self.robot_name

    def get_environment_name(self):
        return self.environment_name

    def get_risk_mitigating_action_file_path(self):
        return self.risk_mitigating_action_full_path

    def get_risk_mitigating_actions(self):
        return self.risk_mitigating_actions

    def get_num_risk_mitigating_actions(self):
        return len(self.risk_mitigating_actions)

    #######################################
    ### PROCESS RISK MITIGATING ACTIONS ###
    #######################################

    def process_risk_mitigating_actions(self):
        # clear out risk mitigating actions list
        self.risk_mitigating_actions = []

        # verify YAML file exists
        valid_path = YAMLChecks.check_yaml_existence(self.risk_mitigating_action_full_path)
        if not valid_path:
            print("ERROR: risk mitigating action file " + self.risk_mitigating_action_full_path + " does not exist")
            self.valid_actions = False
            return

        # open YAML file and load dict
        fo = open(self.risk_mitigating_action_full_path)
        yaml_dict = yaml.load(fo, Loader=yaml.FullLoader)

        # error check YAML file formatting
        valid_yaml = YAMLChecks.check_risk_mitigating_action_yaml_formatting(yaml_dict, self.environment_name)
        if not valid_yaml:
            print("ERROR: risk mitigating action file " + self.risk_mitigating_action_full_path + " is poorly formatted")
            self.valid_actions = False
            return

        # initialize valid actions flag
        self.valid_actions = True

        # get list of actions for environment
        actions = yaml_dict[self.environment_name]['actions']

        # process each action
        for i in range(len(actions)):
            # get action
            act = actions[i]

            # check valid values for risk mitigating action
            valid_action = YAMLChecks.check_valid_risk_mitigating_action_values(act, i, len(actions))
            self.valid_actions = self.valid_actions and valid_action

            # create risk mitigating action
            risk_act = RiskMitigatingAction(name=act['name'])

            # add risk mitigating action to list
            self.risk_mitigating_actions.append(risk_act)

        # close file
        fo.close()

        return

    def check_valid_actions(self):
        return self.valid_actions

    #######################################
    ### RISK MITIGATING ACTION PRINTING ###
    #######################################

    def print_risk_mitigating_actions(self):
        # compute number of actions
        num_acts = len(self.risk_mitigating_actions)

        # print starting message
        print()
        print("Read " + str(num_acts) + " risk mitigating actions for robot " + self.robot_name.upper() + " in " + self.environment_name.upper() + " environment")

        for i in range(num_acts):
            # get action
            act = self.risk_mitigating_actions[i]
            # print action info
            print("    Action " + str(i) + " of " + str(num_acts) + ":")
            print("        name: " + act.get_action_name())
        print()

        return



#####################
### MAIN FUNCTION ###
#####################

if __name__ == '__main__':
    # set node name
    node_name = "RiskMitigatingActionReader"
    param_prefix = "/" + node_name + "/"

    # get ROS parameters
    robot_name = rospy.get_param(param_prefix + 'robot', "val")
    env_name = rospy.get_param(param_prefix + 'environment', "lunar_habitat")

    # initialize node
    rospy.init_node(node_name)

    # create risk mitigating action reader
    rospy.loginfo("[Risk Mitigating Action Reader] Creating risk mitigating action reader...")
    action_reader = RiskMitigatingActionReader(robot=robot_name, environment=env_name)

    # read risk mitigating actions
    rospy.loginfo("[Risk Mitigating Action Reader] Reading risk mitigating actions for robot %s in %s environment...",
                  action_reader.get_robot_name(), action_reader.get_environment_name())
    rospy.loginfo("[Risk Mitigating Action Reader] Trying to read risk mitigating action file '%s'...",
                  action_reader.get_risk_mitigating_action_file_path())
    action_reader.process_risk_mitigating_actions()

    # check if actions are valid
    if action_reader.check_valid_actions():
        rospy.loginfo("[Risk Mitigating Action Reader] Risk mitigating actions processed successfully!")
        rospy.loginfo("[Risk Mitigating Action Reader] Processed %d risk mitigating actions",
                      action_reader.get_num_risk_mitigating_actions())
    else:
        rospy.logwarn("[Risk Mitigating Action Reader] Risk mitigating actions not processed successfully. Please review errors to fix.")

    # print risk mitigating actions
    action_reader.print_risk_mitigating_actions()

    rospy.loginfo("[Risk Mitigating Action Reader] Node stopped, all done!")
