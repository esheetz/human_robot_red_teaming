"""
YAML State Action Space Checks Class
Emily Sheetz, NSTGRO VTE 2024
"""

import os

from likelihood_consequence_risk import LikelihoodLevels, ConsequenceClasses

class YAMLStateActionSpaceChecks:

    ###########################
    ### YAML FILE EXISTENCE ###
    ###########################

    @staticmethod
    def check_yaml_existence(yaml_file_path):
        # check if given YAML file exists and is a file
        return (os.path.exists(yaml_file_path) and
                os.path.isfile(yaml_file_path))

    #####################################
    ### STATE SPACE FORMATTING CHECKS ###
    #####################################

    @staticmethod
    def check_risky_condition_yaml_formatting(yaml_dict, env_name):
        # check if environment exists
        if env_name not in yaml_dict.keys():
            print("ERROR: environment " + env_name + " does not exist in risky conditions file")
            return False

        # check for list of conditions
        if 'conditions' not in yaml_dict[env_name].keys():
            print("ERROR: environment " + env_name + " has no risky conditions defined under key 'conditions'")
            return False

        # get number of conditions
        num_conds = len(yaml_dict[env_name]['conditions'])

        # initialize valid condition flag
        valid_conditions = True

        # check each condition in list
        for i in range(num_conds):
            # get condition
            condition = yaml_dict[env_name]['conditions'][i]

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

    @staticmethod
    def check_valid_risky_condition_values(cond_dict, i, num_conds):
        # initialize valid values flag
        valid_values = True

        # check for valid name
        if not type(cond_dict['name']) == str:
            print("WARN: non-string name for risky condition " + str(i) + " of " + str(num_conds))
            valid_values = False

        # check for valid likelihood and consequence scores
        if not LikelihoodLevels.valid_value(cond_dict['likelihood']):
            print("WARN: invalid likelihood value for risky condition " + str(i) + " of " + str(num_conds))
            valid_values = False

        if not ConsequenceClasses.valid_value(cond_dict['consequence']):
            print("WARN: invalid consequence value for risky condition " + str(i) + " of " + str(num_conds))
            valid_values = False

        return valid_values
