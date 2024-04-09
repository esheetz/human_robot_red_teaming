"""
YAML State Action Space Checks Class
Emily Sheetz, NSTGRO VTE 2024
"""

import os

from likelihood_consequence_risk import LikelihoodLevels, ConsequenceClasses

###################################
### YAML FILE FORMATTING CHECKS ###
###################################

class YAMLChecks:

    # CHECK YAML FILE EXISTENCE

    @staticmethod
    def check_yaml_existence(yaml_file_path):
        # check if given YAML file exists and is a file
        return (os.path.exists(yaml_file_path) and
                os.path.isfile(yaml_file_path))

    # CHECK NON-EMPTY YAML FILE

    @staticmethod
    def check_yaml_nonempty(yaml_file_path):
        # check if given YAML file exists
        if YAMLChecks.check_yaml_existence(yaml_file_path):
            # check if given YAML file is non-empty
            fo = open(yaml_file_path)
            lines = fo.readlines()
            num_lines = len(lines)
            fo.close()
            return (num_lines != 0)
        else:
            # file does not exist
            return False

    # CHECK YAML FORMATTING

    @staticmethod
    def check_yaml_formatting(file_nickname, yaml_dict, env_name, list_key, list_elem_keys):
        # check if environment exists
        if env_name not in yaml_dict.keys():
            print("ERROR: environment " + env_name + " does not exist in " + file_nickname + " file")
            return False

        # check for list key
        if list_key not in yaml_dict[env_name].keys():
            print("ERROR: environment " + env_name + " has no " + file_nickname + "s defined under key '" + list_key + "'")
            return False

        # get number of elements in list
        num_elems = len(yaml_dict[env_name][list_key])

        # initialize valid elements flag
        valid_elems = True

        # check each element in list
        for i in range(num_elems):
            # get element
            elem = yaml_dict[env_name][list_key][i]

            # check for each key in list element
            for elem_key in list_elem_keys:
                if elem_key not in elem.keys():
                    print("ERROR: " + file_nickname + " " + str(i) + " of " + str(num_elems) + " does not have key " + elem_key)
                    valid_elems = False

        return valid_elems



#####################################
### STATE SPACE FORMATTING CHECKS ###
#####################################

class YAMLStateSpaceChecks(YAMLChecks):

    @staticmethod
    def check_consequence_state_yaml_formatting(yaml_dict, env_name):
        return YAMLChecks.check_yaml_formatting(file_nickname="consequence state",
                                                yaml_dict=yaml_dict,
                                                env_name=env_name,
                                                list_key="consequences",
                                                list_elem_keys=["name"])

    @staticmethod
    def check_valid_consequence_state_values(state_dict, i, num_states):
        # initialize valid values flag
        valid_values = True

        # check for valid name
        if not type(state_dict['name']) == str:
            print("WARN: non-string name for consequence state " + str(i) + " of " + str(num_states))
            valid_values = False

        return valid_values

    @staticmethod
    def check_risky_condition_yaml_formatting(yaml_dict, env_name):
        return YAMLChecks.check_yaml_formatting(file_nickname="risky condition",
                                                yaml_dict=yaml_dict,
                                                env_name=env_name,
                                                list_key="conditions",
                                                list_elem_keys=["name","likelihood","consequence","consequence_states"])

    @staticmethod
    def check_valid_risky_condition_values(cond_dict, i, num_conds):
        # initialize valid values flag
        valid_values = True

        # check for valid name
        if not type(cond_dict['name']) == str:
            print("WARN: non-string name for risky condition " + str(i) + " of " + str(num_conds))
            valid_values = False

        # check for valid consequence states
        if not type(cond_dict['consequence_states']) == list:
            print("WARN: non-list consequence states for risky condition " + str(i) + " of " + str(num_conds))
            valid_values = False

        # check for valid likelihood and consequence scores
        if not LikelihoodLevels.valid_value(cond_dict['likelihood']):
            print("WARN: invalid likelihood value for risky condition " + str(i) + " of " + str(num_conds))
            valid_values = False

        if not ConsequenceClasses.valid_value(cond_dict['consequence']):
            print("WARN: invalid consequence value for risky condition " + str(i) + " of " + str(num_conds))
            valid_values = False

        return valid_values



######################################
### ACTION SPACE FORMATTING CHECKS ###
######################################

class YAMLActionSpaceChecks(YAMLChecks):

    @staticmethod
    def check_risk_mitigating_action_yaml_formatting(yaml_dict, env_name):
        return YAMLChecks.check_yaml_formatting(file_nickname="risk mitigating action",
                                                yaml_dict=yaml_dict,
                                                env_name=env_name,
                                                list_key="actions",
                                                list_elem_keys=["name"])

    @staticmethod
    def check_valid_risk_mitigating_action_values(act_dict, i, num_acts):
        # initialize valid values flag
        valid_values = True

        # check for valid name
        if not type(act_dict['name']) == str:
            print("WARN: non-string name for risk mitigating action " + str(i) + " of " + str(num_acts))
            valid_values = False

        return valid_values



#####################################
### POLICY DATA FORMATTING CHECKS ###
#####################################

class YAMLPolicyDataChecks(YAMLChecks):

    @staticmethod
    def check_policy_data_yaml_formatting(yaml_dict, env_name, file_nickname):
        return YAMLChecks.check_yaml_formatting(file_nickname=file_nickname,
                                                yaml_dict=yaml_dict,
                                                env_name=env_name,
                                                list_key="policy_data",
                                                list_elem_keys=["conditions", "consequences_before_action", "action", "consequences_after_action"])

    @staticmethod
    def check_valid_policy_data_values(pol_dict, i, num_pols):
        # initialize valid values flag
        valid_values = True

        # check for valid conditions
        if not type(pol_dict['conditions']) == list:
            print("WARN: non-list conditions for policy data " + str(i) + " of " + str(num_pols))
            valid_values = False

        # check for valid condition types
        for cond in pol_dict['conditions']:
            if not type(cond) == str:
                print("WARN: non-string condition for policy data " + str(i) + " of " + str(num_pols))
                valid_values = False

        # check for valid action
        if not type(pol_dict['action']) == str:
            print("WARN: non-string action for policy data " + str(i) + " of " + str(num_pols))
            valid_values = False

        # check for valid consequences
        if not type(pol_dict['consequences_before_action']) == list:
            print("WARN: non-list consequences before action for policy data " + str(i) + " of " + str(num_pols))
            valid_values = False
        if not type(pol_dict['consequences_after_action']) == list:
            print("WARN: non-list consequences after action for policy data " + str(i) + " of " + str(num_pols))
            valid_values = False

        # check for valid consequence types
        for conseq in pol_dict['consequences_before_action']:
            if not type(conseq) == str:
                print("WARN: non-string consequence before action for policy data " + str(i) + " of " + str(num_pols))
                valid_values = False
        for conseq in pol_dict['consequences_after_action']:
            if not type(conseq) == str:
                print("WARN: non-string consequence after action for policy data " + str(i) + " of " + str(num_pols))
                valid_values = False

        return valid_values

    @staticmethod
    def format_policy_as_yaml_list(policy_dict):
        # initialize list of policy data points
        policy_data_points = []

        # loop through given policy dictionary
        for conds in policy_dict:
            # get condition from dictionary
            policy_point = policy_dict[conds]

            # create dictionary for data point
            point_dict = {
                "conditions" : list(policy_point.get_policy_data_point_condition_names()),
                "consequences_before_action" : list(policy_point.get_policy_data_point_consequences_before_action_names()),
                "action" : policy_point.get_policy_data_point_action_name(),
                "consequences_after_action" : list(policy_point.get_policy_data_point_consequences_after_action_names())
            }

            # add data point to list
            policy_data_points.append(point_dict)

        return policy_data_points
