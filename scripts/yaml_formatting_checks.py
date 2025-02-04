"""
YAML Knowledge Base and Model Checks
"""

import os
import yaml

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
    def check_yaml_keys(yaml_dict, source_key=None, list_keys=None):
        # initialize sub-dict
        yaml_subdict = None

        # check if given source key
        if source_key is not None:
            # check if source key exists
            if source_key not in yaml_dict.keys():
                print("ERROR: source key " + source_key + " does not exist in file")
                return False
            else:
                # key exists
                yaml_subdict = yaml_dict[source_key]
        else:
            # no source key given
            yaml_subdict = yaml_dict

        # check if given list of keys
        if list_keys is not None:
            # check for given list of keys
            for k in list_keys:
                if k not in yaml_subdict.keys():
                    print("ERROR: key " + k + " does not exist in file")
                    return False

        return True

########################################
### KNOWLEDGE BASE FORMATTING CHECKS ###
########################################

class YAMLKBChecks(YAMLChecks):

    @staticmethod
    def check_kb_yaml_formatting(yaml_dict):
        return YAMLChecks.check_yaml_keys(yaml_dict,
                                          source_key="kb",
                                          list_keys=["avoid_states","avoid_actions","facts"])

    @staticmethod
    def check_valid_kb_values(kb_dict):
        # check for valid knowledge base type
        if not type(kb_dict) == dict:
            print("ERROR: knowledge base is not a dictionary")
            return False

        # check for valid states
        if not type(kb_dict["avoid_states"]) == list:
            print("ERROR: knowledge base avoid states are not a list")
            return False
        for i in kb_dict["avoid_states"]:
            if (not type(i) == str) and (not type(i) == list):
                print("ERROR: knowledge base avoid state is not a string or a list of mutex strings")
                return False

        # check for valid actions
        if not type(kb_dict["avoid_actions"]) == list:
            print("ERROR: knowledge base avoid actions are not a list")
            return False
        for i in kb_dict["avoid_actions"]:
            if not type(i) == dict:
                print("ERROR: knowledge base avoid action is not a dictionary")
                return False
            valid_act = YAMLChecks.check_yaml_keys(i,
                                                   source_key=None,
                                                   list_keys=["name","precond","postcond_add","postcond_sub"])
            if not valid_act:
                print("ERROR: knowledge base contains poorly formatted action")
                return False

        # check for valid facts
        if not type(kb_dict["facts"]) == list:
            print("ERROR: knowledbe base facts are not a list")
            return False
        for i in kb_dict["facts"]:
            if not type(i) == str:
                print("ERROR: knowledge base fact is not a string")
                return False

        return True

###############################
### MODEL FORMATTING CHECKS ###
###############################

class YAMLModelChecks(YAMLChecks):

    @staticmethod
    def check_model_yaml_formatting(yaml_dict):
        return YAMLChecks.check_yaml_keys(yaml_dict,
                                          source_key="model",
                                          list_keys=["states","actions","confidence_score"])

    def check_valid_model_values(model_dict):
        # check for valid model dict type
        if not type(model_dict) == dict:
            print("ERROR: model is not a dictionary")
            return False

        # check for valid states
        if not type(model_dict["states"]) == list:
            print("ERROR: model states are not a list")
            return False
        for i in model_dict["states"]:
            if (not type(i) == str) and (not type(i) == list):
                print("ERROR: model state is not a string or a list of mutex strings")
                return False

        # check for valid actions
        if not type(model_dict["actions"]) == list:
            print("ERROR: model actions are not a list")
            return False
        for i in model_dict["actions"]:
            if not type(i) == dict:
                print("ERROR: model action is not a dictionary")
                return False
            valid_act = YAMLChecks.check_yaml_keys(i,
                                                   source_key=None,
                                                   list_keys=["name","precond","postcond_add","postcond_sub"])
            if not valid_act:
                print("ERROR: model contains poorly formatted action")
                return False

        # check for valid confidence
        if not type(model_dict["confidence_score"]) == dict:
            print("ERROR: model confidence is not a dictionary")
            return False
        valid_conf = YAMLChecks.check_yaml_keys(model_dict["confidence_score"],
                                                source_key=None,
                                                list_keys=["successes","attempts"])
        if not valid_conf:
            print("ERROR: model confidence is poorly formatted")
            return False
        if not type(model_dict["confidence_score"]["successes"]) == int:
            print("ERROR: model confidence successes is not an integer")
            return False
        if not type(model_dict["confidence_score"]["attempts"]) == int:
            print("ERROR: model confidence attempts is not an integer")
            return False

        return True

#################################
### KNOWLEDGE BASE FORMATTING ###
#################################

class KBFormatting:

    @staticmethod
    def format_kb(yaml_file_path):
        # verify YAML file exists
        if not YAMLChecks.check_yaml_existence(yaml_file_path):
            print("ERROR: knowledge base file " + yaml_file_path + " does not exist")
            return False, []

        # open YAML file and load dict
        fo = open(yaml_file_path)
        yaml_dict = yaml.load(fo, Loader=yaml.FullLoader)
        fo.close()

        # error check YAML file formatting
        if not YAMLKBChecks.check_kb_yaml_formatting(yaml_dict):
            print("ERROR: knowledge base YAML file is poorly formatted")
            return False, []

        # get knowledge base
        kb_list = yaml_dict["kb"]

        # error check knowledge base formatting
        if not YAMLKBChecks.check_valid_kb_values(kb_list):
            print("ERROR: knowledge base is poorly formatted")
            return False, []

        # if we get here, everything is properly formatted
        # return knowledge base as list of strings
        return True, kb_list

########################
### MODEL FORMATTING ###
########################

class ModelFormatting:

    @staticmethod
    def format_model(yaml_file_path):
        # verify YAML file exists
        if not YAMLChecks.check_yaml_existence(yaml_file_path):
            print("ERROR: model file " + yaml_file_path + " does not exist")
            return False, {}

        # open YAML file and load dict
        fo = open(yaml_file_path)
        yaml_dict = yaml.load(fo, Loader=yaml.FullLoader)
        fo.close()

        # error check YAML file formatting
        if not YAMLModelChecks.check_model_yaml_formatting(yaml_dict):
            print("ERROR: model YAML file is poorly formatted")
            return False, {}

        # get model
        model_dict = yaml_dict["model"]

        # error check model formatting
        if not YAMLModelChecks.check_valid_model_values(model_dict):
            print("ERROR: model is poorly formatted")
            return False, {}

        # if we get here, everything is properly formatted
        # return model as dictionary of states and actions
        return True, model_dict
