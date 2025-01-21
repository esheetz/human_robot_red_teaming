#!/usr/bin/env python3
"""
Consequence State Reader
    includes class ConsequenceStateSpaceReader, which can be used in other nodes
    optionally can be run as a standalone script to test parsing of consequence state space
Emily Sheetz, NSTGRO VTE 2024
"""

import rospy

import os
import yaml

from yaml_formatting_checks import YAMLStateSpaceChecks as YAMLChecks

from consequence_state import ConsequenceState

class ConsequenceStateReader:
    def __init__(self, robot="val", environment="lunar_habitat"):
        # set internal parameters
        self.robot_name = robot
        self.environment_name = environment

        # get path of this script
        script_path = os.path.abspath(os.path.dirname( __file__ ))

        # set consequence state file name and path
        self.consequence_state_file = "consequence_states.yaml"
        self.consequence_state_full_path = script_path + "/../config/" + self.robot_name + "/" + self.consequence_state_file

        # initialize list of consequence states
        self.consequence_states = []

        # initialize flag for valid consequence states
        self.valid_states = False

    #######################
    ### GETTERS/SETTERS ###
    #######################

    def get_robot_name(self):
        return self.robot_name

    def get_environment_name(self):
        return self.environment_name

    def get_consequence_state_file_path(self):
        return self.consequence_state_full_path

    def get_consequence_states(self):
        return self.consequence_states

    def get_consequence_state_names(self):
        return [state.get_consequence_name() for state in self.consequence_states]

    def get_consequence_state_with_name(self, state_name):
        # look through states
        for state in self.consequence_states:
            # check name
            if state.get_consequence_name() == state_name:
                return state
        # if we get here, no state in list has given name
        return None

    def get_num_consequence_states(self):
        return len(self.consequence_states)

    ##################################
    ### PROCESS CONSEQUENCE STATES ###
    ##################################

    def process_consequence_states(self):
        # clear out consequence states list
        self.consequence_states = []

        # verify YAML file exists
        valid_path = YAMLChecks.check_yaml_existence(self.consequence_state_full_path)
        if not valid_path:
            print("ERROR: consequence state file " + self.consequence_state_full_path + " does not exist")
            self.valid_states = False
            return

        # open YAML file and load dict
        fo = open(self.consequence_state_full_path)
        yaml_dict = yaml.load(fo, Loader=yaml.FullLoader)

        # error check YAML file formatting
        valid_yaml = YAMLChecks.check_consequence_state_yaml_formatting(yaml_dict, self.environment_name)
        if not valid_yaml:
            print("ERROR: consequence state file " + self.consequence_state_full_path + " is poorly formatted")
            self.valid_states = False
            return

        # initialize valid states flag
        self.valid_states = True

        # get list of states for environment
        states = yaml_dict[self.environment_name]['consequences']

        # process each state
        for i in range(len(states)):
            # get state
            state = states[i]

            # check valid values for consequence state
            valid_state = YAMLChecks.check_valid_consequence_state_values(state, i, len(states))
            self.valid_states = self.valid_states and valid_state

            # create consequence state
            conseq_state = ConsequenceState(name=state['name'])

            # add consequence state to list
            self.consequence_states.append(conseq_state)

        # close file
        fo.close()

        return

    def check_valid_states(self):
        return self.valid_states

    ##################################
    ### CONSEQUENCE STATE PRINTING ###
    ##################################

    def print_consequence_states(self):
        # compute number of consequence states
        num_states = len(self.consequence_states)

        # print starting message
        print()
        print("Read " + str(num_states) + " consequence states for robot " + self.robot_name.upper() + " in " + self.environment_name.upper() + " environment")

        for i in range(num_states):
            # get state
            state = self.consequence_states[i]
            # print state info
            print("    Consequence state " + str(i) + " of " + str(num_states) + ":")
            print("        name: " + state.get_consequence_name())
        print()

        return



#####################
### MAIN FUNCTION ###
#####################

if __name__ == '__main__':
    # set node name
    node_name = "ConsequenceStateReader"
    param_prefix = "/" + node_name + "/"

    # get ROS parameters
    robot_name = rospy.get_param(param_prefix + 'robot', "val")
    env_name = rospy.get_param(param_prefix + 'environment', "lunar_habitat")

    # initialize node
    rospy.init_node(node_name)

    # create consequence state reader
    rospy.loginfo("[Consequence State Reader] Creating consequence state reader...")
    conseq_reader = ConsequenceStateReader(robot=robot_name, environment=env_name)

    # read consequence state space
    rospy.loginfo("[Consequence State Reader] Reading consequence states for robot %s in %s environment...",
                  conseq_reader.get_robot_name(), conseq_reader.get_environment_name())
    rospy.loginfo("[Consequence State Reader] Trying to read consequence state file '%s'...",
                  conseq_reader.get_consequence_state_file_path())
    conseq_reader.process_consequence_states()

    # check if states are valid
    if conseq_reader.check_valid_states():
        rospy.loginfo("[Consequence State Reader] Consequence states processed successfully!")
        rospy.loginfo("[Consequence State Reader] Processed %d consequence staets",
                      conseq_reader.get_num_consequence_states())
    else:
        rospy.logwarn("[Consequence State Reader] Consequence states not processed successfully. Please review errors to fix.")

    # print consequence staets
    conseq_reader.print_consequence_states()

    rospy.loginfo("[Consequence State Reader] Node stopped, all done!")
