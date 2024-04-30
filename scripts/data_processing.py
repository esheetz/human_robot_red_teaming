#!/usr/bin/env python3
"""
Data Processing Helper Classes
Emily Sheetz, NSTGRO VTE 2024
"""

import rospy

import os, yaml
from copy import deepcopy
import pandas as pd
import numpy as np

# data readers
from likelihood_consequence_risk import LikelihoodLevels, ConsequenceClasses
from risky_condition_reader import RiskyConditionReader
from consequence_state_reader import ConsequenceStateReader
from risk_mitigating_action_reader import RiskMitigatingActionReader
from risk_mitigating_policy_data_reader import RiskMitigatingPolicyDataReader

# logistic regression
from sklearn.model_selection import train_test_split
import statsmodels.formula.api as smf
from scipy.linalg import LinAlgError

# evaluation
from sklearn.metrics import confusion_matrix, ConfusionMatrixDisplay, accuracy_score

# helpers for exploring data relationships
from itertools import combinations
from sklearn.preprocessing import PolynomialFeatures

# save model
import pickle

##########################
### DATASET INFO CLASS ###
##########################

class DatasetInfo:
    """
    Stores information and variables related to safety-aware reasoning datasets
    """

    def __init__(self):
        self.initialize_files_directories()
        self.initialize_action_space_encodings()

    ######################
    ### INITIALIZATION ###
    ######################

    def initialize_files_directories(self):
        # get path of this script
        script_path = os.path.abspath(os.path.dirname( __file__ ))
        
        # data directory
        self.data_dir = script_path + "/../data/"

        # supported robots/environments
        self.supported_robots = ['clr','val','val_clr']
        self.supported_envs = ['household','lunar_habitat']

        # random risky scenario
        self.rrs_policy_file_name = "red_teamed_risk_mitigating_policy_data.yaml"
        self.rrs_dataset_file_name = "risky_scenario_policy_data.csv"
        # counter-factual action
        self.cfa_policy_file_name = "counter_factual_policy_data.yaml"
        self.cfa_dataset_file_name = "counter_factual_policy_data.csv"
        self.cfa_limited_dataset_file_name = "counter_factual_policy_data_limited.csv"
        self.cfa_match_factual_dataset_file_name = "counter_factual_policy_data_matches_factual.csv"

        # pre-processed data name
        self.data_file_name = "risk_mitigating_action_utility_data.csv"
        self.data_limited_file_name = "risk_mitigating_action_utility_data_limited.csv"
        self.data_match_factual_file_name = "risk_mitigating_action_utility_data_matches_factual.csv"

        # saved models directory
        self.models_dir = script_path + "/../saved_models/"

        # models file ending
        self.model_file_end = "_model.sav"

        return

    def initialize_action_space_encodings(self):
        # initialize dictionary of encodings
        self.action_space_encodings = {}

        # loop through robot/environment combinations
        for r in self.supported_robots:
            for e in self.supported_envs:
                # initialize dictionary
                key = self.create_action_encoding_dictionary_key(r, e)
                self.action_space_encodings[key] = {}

                # set dictionary
                self.action_space_encodings[key] = self.get_action_encoding_for_robot_env(r, e)

        return

    #########################
    ### GETTERS / SETTERS ###
    #########################

    def get_rrs_policy_full_path(self, robot):
        return self.data_dir + robot + "/" + self.rrs_policy_file_name

    def get_rrs_dataset_full_path(self, robot, env):
        path = self.data_dir + robot + "/" + env + "/"
        file_name = path + self.rrs_dataset_file_name
        return path, file_name

    def get_cfa_policy_full_path(self, robot):
        return self.data_dir + robot + "/" + self.cfa_policy_file_name

    def get_cfa_dataset_full_path(self, robot, env):
        path = self.data_dir + robot + "/" + env + "/"
        file_name = path + self.cfa_dataset_file_name
        return path, file_name

    def get_cfa_limited_dataset_full_path(self, robot, env):
        path = self.data_dir + robot + "/" + env + "/"
        file_name = path + self.cfa_limited_dataset_file_name
        return path, file_name

    def get_cfa_match_factual_dataset_full_path(self, robot, env):
        path = self.data_dir + robot + "/" + env + "/"
        file_name = path + self.cfa_match_factual_dataset_file_name
        return path, file_name

    def get_combined_dataset_full_path(self, robot, env, limited_cfa=False, weight=None):
        path = self.data_dir + robot + "/" + env + "/"
        file_name = None
        if limited_cfa:
            file_name_limited = path + self.data_limited_file_name
            file_name_match_factual = path + self.data_match_factual_file_name
            if weight is not None:
                file_name_limited = file_name_limited.replace(".csv","_weighted_{}x.csv".format(weight))
                file_name_match_factual = file_name_match_factual.replace(".csv","_weighted_{}x.csv".format(weight))
            file_name = (file_name_limited, file_name_match_factual)
        else:
            file_name = path + self.data_file_name
            if weight is not None:
                file_name = file_name.replace(".csv","_weighted_{}x.csv".format(weight))

        return path, file_name

    def get_action_encoding_for_robot_env(self, robot, env):
        # create reader and process data
        action_space_reader = RiskMitigatingActionReader(robot=robot, environment=env)
        action_space_reader.process_risk_mitigating_actions()

        # get action space names
        action_names = action_space_reader.get_risk_mitigating_action_names()
        
        # initialize dictionary
        action_space_encodings = {}

        # add action encodings to dictionary
        for i, item in enumerate(action_names):
            action_space_encodings[item] = i

        return action_space_encodings

    def get_action_autonomy_space_for_robot_env(self, robot, env):
        # create reader and process data
        action_space_reader = RiskMitigatingActionReader(robot=robot, environment=env)
        action_space_reader.process_risk_mitigating_actions()

        # initialize dictionary
        action_autonomy_space = {}

        # add action and autonomy level to dictionary
        for action in action_space_reader.get_risk_mitigating_actions():
            a_name = action.get_action_name()
            a_level = action.get_action_autonomy_level()
            # add entry to dictionary
            action_autonomy_space[a_name] = a_level

        return action_autonomy_space

    def get_consequence_space_for_robot_env(self, robot, env):
        # create reader and process data
        conseq_space_reader = ConsequenceStateReader(robot=robot, environment=env)
        conseq_space_reader.process_consequence_states()

        # initialize list
        conseq_space = []

        # add consequence name to list
        for conseq in conseq_space_reader.get_consequence_states():
            c_name = conseq.get_consequence_name()
            conseq_space.append(c_name)

        return conseq_space

    def get_risky_conditions_for_robot_env(self, robot, env):
        # create reader and process data
        state_space_reader = RiskyConditionReader(robot=robot, environment=env)
        state_space_reader.process_risky_conditions()

        # initialize dictionary
        state_space = {}

        # add risky condition information to dictionary
        for cond in state_space_reader.get_risky_conditions():
            c_name = cond.get_condition_name()
            c_likeli = cond.get_likelihood_level()
            c_conseq = cond.get_consequence_class()
            c_risk = cond.get_matrix_risk_score()
            c_safety = cond.get_matrix_safety_score()
            # add each entry to dictionary
            state_space[c_name] = {}
            state_space[c_name]['likelihood'] = c_likeli / LikelihoodLevels.get_max()
            state_space[c_name]['consequence'] = c_conseq / ConsequenceClasses.get_max()
            state_space[c_name]['risk'] = c_risk
            state_space[c_name]['safety'] = c_safety

        return state_space

    def get_policy_starter_for_robot_env(self, robot, env):
        # create reader and process data
        policy_starter_reader = RiskMitigatingPolicyDataReader(robot=robot, environment=env, human_gen_data=True)
        policy_starter_reader.process_risk_mitigating_policy_data()

        # initialize dictionary
        policy_points = {}

        # add policy starter information to dictionary
        for key,pol_point in policy_starter_reader.get_risk_mitigating_policy_data().items():
            p_conds = pol_point.get_policy_data_point_condition_names()
            p_action = pol_point.get_policy_data_point_action_name()
            p_conseqs_bef, p_conseqs_aft = pol_point.get_policy_data_point_consequence_names()
            # add each entry to dictionary
            policy_points[p_conds] = {}
            policy_points[p_conds]['conseqs_bef'] = p_conseqs_bef
            policy_points[p_conds]['action'] = p_action
            policy_points[p_conds]['conseqs_aft'] = p_conseqs_aft

        return policy_points

    ###############
    ### HELPERS ###
    ###############

    def create_action_encoding_dictionary_key(self, robot, env):
        key = robot + "_" + env
        return key



################################
### DATASET FORMATTING CLASS ###
################################

class DatasetColumns:
    """
    Stores formatting and column information about the dataset
    """

    def __init__(self, robot="val_clr", environment="lunar_habitat"):
        # set internal paramters
        self.robot_name = robot
        self.environment_name = environment

        # initialize data info
        self.info = DatasetInfo()

        # initialize relevant information
        self.action_encoding = self.info.get_action_encoding_for_robot_env(self.robot_name, self.environment_name)
        self.action_autonomy_space = self.info.get_action_autonomy_space_for_robot_env(self.robot_name, self.environment_name)
        self.consequence_space = self.info.get_consequence_space_for_robot_env(self.robot_name, self.environment_name)
        self.risky_conditions = self.info.get_risky_conditions_for_robot_env(self.robot_name, self.environment_name)

        # initialize column names and types
        self.initialize_column_names_types()

    ######################
    ### INITIALIZATION ###
    ######################

    def initialize_column_names_types(self):
        self.column_names = []
        self.column_types = {}

        # add column names for risky conditions
        for cond in self.risky_conditions.keys():
            col_name = self.get_col_name_for_condition(cond)
            self.column_names.append(col_name)
            self.column_types[col_name] = int
            col_name = self.get_col_name_for_condition_likelihood(cond)
            self.column_names.append(col_name)
            self.column_types[col_name] = float
            col_name = self.get_col_name_for_condition_consequence(cond)
            self.column_names.append(col_name)
            self.column_types[col_name] = float
            col_name = self.get_col_name_for_condition_risk(cond)
            self.column_names.append(col_name)
            self.column_types[col_name] = float
            col_name = self.get_col_name_for_condition_safety(cond)
            self.column_names.append(col_name)
            self.column_types[col_name] = float
            col_name = self.get_col_name_for_condition_autonomy(cond)
            self.column_names.append(col_name)
            self.column_types[col_name] = float

        # add column names for consequences pre-action
        for conseq in self.consequence_space:
            col_name = self.get_col_name_for_consequence(conseq, pre_action=True)
            self.column_names.append(col_name)
            self.column_types[col_name] = int

        # add columns for risk/safety of whole state
        col_name = self.get_col_name_for_state_conseq()
        self.column_names.append(col_name)
        self.column_types[col_name] = float
        col_name = self.get_col_name_for_state_risk()
        self.column_names.append(col_name)
        self.column_types[col_name] = float
        col_name = self.get_col_name_for_state_safety()
        self.column_names.append(col_name)
        self.column_types[col_name] = float
        col_name = self.get_col_name_for_state_autonomy()
        self.column_names.append(col_name)
        self.column_types[col_name] = float

        # add column names for consequences post-action
        for conseq in self.consequence_space:
            col_name = self.get_col_name_for_consequence(conseq, pre_action=False)
            self.column_names.append(col_name)
            self.column_types[col_name] = int

        # add columns for action and encoded action
        col_name = self.get_col_name_for_action()
        self.column_names.append(col_name)
        self.column_types[col_name] = str
        col_name = self.get_col_name_for_action_encoded()
        self.column_names.append(col_name)
        self.column_types[col_name] = int

        return

    ###########################
    ### COLUMN NAME HELPERS ###
    ###########################

    def get_col_name_for_condition(self, cond_name):
        return "COND_NAME_" + cond_name

    def get_col_name_for_condition_likelihood(self, cond_name):
        return "COND_LIKELI_" + cond_name

    def get_col_name_for_condition_consequence(self, cond_name):
        return "COND_CONSEQ_" + cond_name

    def get_col_name_for_condition_risk(self, cond_name):
        return "COND_RISK_" + cond_name

    def get_col_name_for_condition_safety(self, cond_name):
        return "COND_SAFETY_" + cond_name

    def get_col_name_for_condition_autonomy(self, cond_name):
        return "COND_AUTO_LEVEL_" + cond_name

    def get_col_name_for_consequence(self, conseq_name, pre_action=True):
        if pre_action:
            return "CONSEQ_PRE_ACT_" + conseq_name
        else:
            return "CONSEQ_POST_ACT_" + conseq_name

    def get_col_name_for_state_conseq(self):
        return "STATE_CONSEQ"

    def get_col_name_for_state_risk(self):
        return "STATE_RISK"

    def get_col_name_for_state_safety(self):
        return "STATE_SAFETY"

    def get_col_name_for_state_autonomy(self):
        return "STATE_AUTO_LEVEL"

    def get_col_name_for_action(self):
        return "RISK_MITIGATING_ACTION"

    def get_col_name_for_action_encoded(self):
        return "RISK_MITIGATING_ACTION_ENCODED"

    def check_col_name_for_condition(self, col_name):
        return "COND_NAME_" in col_name

    def check_col_name_for_condition_likelihood(self, col_name):
        return "COND_LIKELI_" in col_name

    def check_col_name_for_condition_consequence(self, col_name):
        return "COND_CONSEQ_" in col_name

    def check_col_name_for_condition_risk(self, col_name):
        return "COND_RISK_" in col_name

    def check_col_name_for_condition_safety(self, col_name):
        return "COND_SAFETY_" in col_name

    def check_col_name_for_condition_autonomy(self, col_name):
        return "COND_AUTO_LEVEL_" in col_name

    def check_col_name_for_consequence(self, col_name, pre_action=True):
        if pre_action:
            return "CONSEQ_PRE_ACT_" in col_name
        else:
            return "CONSEQ_POST_ACT_" in col_name

    def check_col_name_for_state_conseq(self, col_name):
        return "STATE_CONSEQ" == col_name

    def check_col_name_for_state_risk(self, col_name):
        return "STATE_RISK" == col_name

    def check_col_name_for_state_safety(self, col_name):
        return "STATE_SAFETY" == col_name

    def check_col_name_for_state_autonomy(self, col_name):
        return "STATE_AUTO_LEVEL" == col_name

    def check_col_name_for_action(self, col_name):
        return "RISK_MITIGATING_ACTION" == col_name

    def check_col_name_for_action_encoded(self, col_name):
        return "RISK_MITIGATING_ACTION_ENCODED" == col_name

    def get_condition_name_from_col_name(self, col_name):
        return col_name.replace("COND_NAME_","") \
                       .replace("COND_LIKELI_","") \
                       .replace("COND_CONSEQ_","") \
                       .replace("COND_RISK_","") \
                       .replace("COND_SAFETY_","") \
                       .replace("COND_AUTO_LEVEL_","")

    def get_consequence_name_from_col_name(self, col_name):
        return col_name.replace("CONSEQ_PRE_ACT_","").replace("CONSEQ_POST_ACT_","")

    def get_all_conseq_col_idxs(self, pre_action=True):
        if pre_action:
            return [i for i in range(len(self.column_names)) if "CONSEQ_PRE_ACT_" in self.column_names[i]]
        else:
            return [i for i in range(len(self.column_names)) if "CONSEQ_POST_ACT_" in self.column_names[i]]

    def get_all_conseq_col_names(self, pre_action=True):
        if pre_action:
            return [i for i in self.column_names if "CONSEQ_PRE_ACT_" in i]
        else:
            return [i for i in self.column_names if "CONSEQ_POST_ACT_" in i]

    def get_all_cond_conseq_col_idxs(self):
        return [i for i in range(len(self.column_names)) if "COND_CONSEQ_" in self.column_names[i]]

    def get_all_cond_conseq_col_names(self):
        return [i for i in self.column_names if "COND_CONSEQ_" in i]

    def get_all_cond_risk_col_idxs(self):
        return [i for i in range(len(self.column_names)) if "COND_RISK_" in self.column_names[i]]

    def get_all_cond_risk_col_names(self):
        return [i for i in self.column_names if "COND_RISK_" in i]

    def get_all_cond_auto_col_idxs(self):
        return [i for i in range(len(self.column_names)) if "COND_AUTO_LEVEL_" in self.column_names[i]]

    def get_all_cond_auto_col_names(self):
        return [i for i in self.column_names if "COND_AUTO_LEVEL_" in i]



################################
### DATA PREPROCESSING CLASS ###
################################

class DataPreprocessing:
    """
    Pre-processes red teamed data (YAML files) into CSV files
    """

    def __init__(self, robot="val_clr", environment="lunar_habitat"):
        # set internal paramters
        self.robot_name = robot
        self.environment_name = environment

        # initialize data info
        self.info = DatasetInfo()

        # initialize data formatting
        self.col_info = DatasetColumns(self.robot_name, self.environment_name)

        # initialize relevant information
        self.action_encoding = self.info.get_action_encoding_for_robot_env(self.robot_name, self.environment_name)
        self.action_autonomy_space = self.info.get_action_autonomy_space_for_robot_env(self.robot_name, self.environment_name)
        self.consequence_space = self.info.get_consequence_space_for_robot_env(self.robot_name, self.environment_name)
        self.risky_conditions = self.info.get_risky_conditions_for_robot_env(self.robot_name, self.environment_name)
        self.policy_starters = self.info.get_policy_starter_for_robot_env(self.robot_name, self.environment_name)

    ######################
    ### CREATE DATASET ###
    ######################

    def convert_yamls_to_dataset_csv(self):
        # create and save RRS and CFA datasets
        df_rrs = self.convert_rrs_yaml_to_dataset_csv()
        df_cfa = self.convert_cfa_yaml_to_dataset_csv()

        # create and save full combined dataset
        path, file_name = self.info.get_combined_dataset_full_path(self.robot_name, self.environment_name)
        df_full = self.create_combined_csv(df_rrs, df_cfa, path, file_name)

        # create and save limited CFA dataset
        df_cfa_limited, df_cfa_match_factual = self.convert_limited_cfa_to_dataset_csv(df_cfa)

        # create and save combined dataset from limited CFA datasets
        path, (file_name_limited, file_name_match_factual) = self.info.get_combined_dataset_full_path(self.robot_name, self.environment_name, limited_cfa=True)
        df = self.create_combined_csv(df_rrs, df_cfa_limited, path, file_name_limited)
        df = self.create_combined_csv(df_rrs, df_cfa_match_factual, path, file_name_match_factual)

        return df_rrs, df_cfa_match_factual

    def create_weighted_limited_datasets(self, df1, df2, max_df1_weight=9):
        # create initial combined dataset with 1:1 ratio
        df = pd.concat([df1, df2], ignore_index=True)

        # create additional weights
        for weight in range(2,max_df1_weight+1):
            # add additional copy of df1
            df = pd.concat([df1, df], ignore_index=True)
            # save data frame to file
            path, (_, weighted_file_name) = self.info.get_combined_dataset_full_path(self.robot_name, self.environment_name, limited_cfa=True, weight=weight)
            self.save_pandas_as_csv(df, path, weighted_file_name)

        return df

    def convert_rrs_yaml_to_dataset_csv(self):
        # create data frame
        df_rrs = self.convert_yaml_to_pandas(self.info.get_rrs_policy_full_path(self.robot_name))

        # save data frame to file
        path, file_name = self.info.get_rrs_dataset_full_path(self.robot_name, self.environment_name)
        self.save_pandas_as_csv(df_rrs, path, file_name)

        return df_rrs

    def convert_cfa_yaml_to_dataset_csv(self):
        # create data frame
        df_cfa = self.convert_yaml_to_pandas(self.info.get_cfa_policy_full_path(self.robot_name))

        # save data frame to file
        path, file_name = self.info.get_cfa_dataset_full_path(self.robot_name, self.environment_name)
        self.save_pandas_as_csv(df_cfa, path, file_name)

        return df_cfa

    def convert_limited_cfa_to_dataset_csv(self, df_cfa):
        # create data frames
        df_cfa_limited, df_cfa_match_factual = self.limit_cfa_dataset_to_improvement_examples(df_cfa)

        # save data frames to file
        path, file_name = self.info.get_cfa_limited_dataset_full_path(self.robot_name, self.environment_name)
        self.save_pandas_as_csv(df_cfa_limited, path, file_name)
        path, file_name = self.info.get_cfa_match_factual_dataset_full_path(self.robot_name, self.environment_name)
        self.save_pandas_as_csv(df_cfa_match_factual, path, file_name)

        return df_cfa_limited, df_cfa_match_factual

    def create_combined_csv(self, df1, df2, path, file_name):
        # create combined data frame
        df = pd.concat([df1, df2], ignore_index=True)

        # save data frame to file
        self.save_pandas_as_csv(df, path, file_name)

        return df

    def save_pandas_as_csv(self, df, csv_path, csv_file):
        # check if path exists:
        if not os.path.exists(csv_path):
            # create directory
            os.mkdir(csv_path)

        # save data frame to csv file
        df.to_csv(csv_file, index=False) # encoding='utf-8'
        return

    def limit_cfa_dataset_to_improvement_examples(self, df_cfa):
        # get indices of pre-action and post-action consequence columns
        pre_act_idxs = self.col_info.get_all_conseq_col_idxs(pre_action=True)
        post_act_idxs = self.col_info.get_all_conseq_col_idxs(pre_action=False)

        # initialize list of rows
        pos_row_idxs = []
        fact_row_idxs = []

        # look through every row
        for i in range(df_cfa.shape[0]):
            # get number of pre-action consequences and post-action consequences
            num_pre_act_conseq = sum(df_cfa.iloc[i,pre_act_idxs])
            num_post_act_conseq = sum(df_cfa.iloc[i,post_act_idxs])

            # check for improvement
            if num_post_act_conseq < num_pre_act_conseq:
                pos_row_idxs.append(i)
                # check for matching factual action
                if num_post_act_conseq == 0:
                    fact_row_idxs.append(i)

        # select only the positive rows
        df_cfa_pos = df_cfa.iloc[pos_row_idxs,:]
        df_cfa_fact = df_cfa.iloc[fact_row_idxs,:]

        return df_cfa_pos, df_cfa_fact

    def convert_yaml_to_pandas(self, yaml_file):
        # open yaml file
        fo = open(yaml_file)
        yaml_dict = yaml.load(fo, Loader=yaml.FullLoader)
        fo.close()

        # get policy data
        policy_data = yaml_dict[self.environment_name]['policy_data']

        # initialize dictionary with column names as keys
        dataset_dict = {}
        for col_name in self.col_info.column_names:
            dataset_dict[col_name] = []

        # compute column indices that include condition risks
        cond_conseq_cols = self.col_info.get_all_cond_conseq_col_names()
        cond_risk_cols = self.col_info.get_all_cond_risk_col_names()
        cond_auto_cols = self.col_info.get_all_cond_auto_col_names()

        # look through list of policy data points
        for pol_point in policy_data:
            # set data for each column
            for col_name in self.col_info.column_names:
                # get column type
                col_type = self.col_info.column_types[col_name]

                # set data for this column
                if self.col_info.check_col_name_for_condition(col_name):
                    # get condition name from current column
                    cond_name = self.col_info.get_condition_name_from_col_name(col_name)
                    # check if condition is present in policy data point
                    if cond_name in pol_point['conditions']:
                        # condition is present in this data point, set to True
                        dataset_dict[col_name].append( col_type(1) )
                    else:
                        # condition is not present in this data point, set to False
                        dataset_dict[col_name].append( col_type(0) )
                elif self.col_info.check_col_name_for_condition_likelihood(col_name):
                    # get condition name from current column
                    cond_name = self.col_info.get_condition_name_from_col_name(col_name)
                    # check if condition is present in policy data point
                    if cond_name in pol_point['conditions']:
                        # condition is present in this data point, set likelihood
                        dataset_dict[col_name].append( col_type(self.risky_conditions[cond_name]['likelihood']) )
                    else:
                        # condition is not present in this data point, set likelihood
                        dataset_dict[col_name].append( col_type(0.0) )
                elif self.col_info.check_col_name_for_condition_consequence(col_name):
                    # get condition name from current column
                    cond_name = self.col_info.get_condition_name_from_col_name(col_name)
                    # check if condition is present in policy data point
                    if cond_name in pol_point['conditions']:
                        # condition is present in this data point, set consequence
                        dataset_dict[col_name].append( col_type(self.risky_conditions[cond_name]['consequence']) )
                    else:
                        # condition is not present in this data point, set consequence
                        dataset_dict[col_name].append( col_type(0.0) )
                elif self.col_info.check_col_name_for_condition_risk(col_name):
                    # get condition name from current column
                    cond_name = self.col_info.get_condition_name_from_col_name(col_name)
                    # check if condition is present in policy data point
                    if cond_name in pol_point['conditions']:
                        # condition is present in this data point, set risk
                        dataset_dict[col_name].append( col_type(self.risky_conditions[cond_name]['risk']) )
                    else:
                        # condition is not present in this data point, set risk to 0
                        dataset_dict[col_name].append( col_type(0.0) )
                elif self.col_info.check_col_name_for_condition_safety(col_name):
                    # get condition name from current column
                    cond_name = self.col_info.get_condition_name_from_col_name(col_name)
                    # check if condition is present in policy data point
                    if cond_name in pol_point['conditions']:
                        # condition is present in this data point, set safety
                        dataset_dict[col_name].append( col_type(self.risky_conditions[cond_name]['safety']) )
                    else:
                        # condition is not present in this data point, set safety to 1
                        dataset_dict[col_name].append( col_type(1.0) )
                elif self.col_info.check_col_name_for_condition_autonomy(col_name):
                    # get condition name from current column
                    cond_name = self.col_info.get_condition_name_from_col_name(col_name)
                    # check if condition is present in policy data point
                    if cond_name in pol_point['conditions']:
                        # condition is present in this data point, get action from policy starter
                        action = self.policy_starters[ tuple([cond_name]) ]['action']
                        # set autonomy level
                        dataset_dict[col_name].append( col_type(self.action_autonomy_space[action]) )
                    else:
                        # condition is not present in this data point, set autonomy level to 1
                        dataset_dict[col_name].append ( col_type(1.0) )
                elif self.col_info.check_col_name_for_consequence(col_name, pre_action=True):
                    # get consequence name from current column
                    conseq_name = self.col_info.get_consequence_name_from_col_name(col_name)
                    # check if consequence is present in policy data point
                    if conseq_name in pol_point['consequences_before_action']:
                        # consequence is present in this data point, set to True
                        dataset_dict[col_name].append( col_type(1) )
                    else:
                        # consequence is not present in this data point, set to False
                        dataset_dict[col_name].append( col_type(0) )
                elif self.col_info.check_col_name_for_consequence(col_name, pre_action=False):
                    # get consequence name from current column
                    conseq_name = self.col_info.get_consequence_name_from_col_name(col_name)
                    # check if consequence is present in policy data point
                    if conseq_name in pol_point['consequences_after_action']:
                        # consequence is present in this data point, set to True
                        dataset_dict[col_name].append( col_type(1) )
                    else:
                        # consequence is not present in this data point, set to False
                        dataset_dict[col_name].append( col_type(0) )
                elif self.col_info.check_col_name_for_state_conseq(col_name):
                    # get consequences from all conditions
                    conseqs = [dataset_dict[col][-1] for col in cond_conseq_cols]
                    # state consequence is maximum of all consequences
                    dataset_dict[col_name].append( col_type(max(conseqs)) )
                elif self.col_info.check_col_name_for_state_risk(col_name):
                    # get risks from all conditions
                    risks = [dataset_dict[col][-1] for col in cond_risk_cols]
                    # state risk is maximum of all risks
                    dataset_dict[col_name].append( col_type(max(risks)) )
                elif self.col_info.check_col_name_for_state_safety(col_name):
                    # get state risk
                    risk = dataset_dict[self.col_info.get_col_name_for_state_risk()][-1]
                    # state safety is 1 - state risk
                    dataset_dict[col_name].append( col_type(1 - risk) )
                elif self.col_info.check_col_name_for_state_autonomy(col_name):
                    # get autonomy levels from all conditions
                    autos = [dataset_dict[col][-1] for col in cond_auto_cols]
                    # state autonomy level is minimum of all autonomy levels
                    dataset_dict[col_name].append( col_type(min(autos)) )
                elif self.col_info.check_col_name_for_action(col_name):
                    # set action name
                    dataset_dict[col_name].append( col_type(pol_point['action']) )
                elif self.col_info.check_col_name_for_action_encoded(col_name):
                    # set encoded action name
                    dataset_dict[col_name].append( col_type(self.action_encoding[pol_point['action']]) )
                else:
                    # should never happen
                    continue
            # end processing column name
        # end processing data points

        # create data frame from dictionary
        df = pd.DataFrame(dataset_dict)

        return df



#############################
### DATA PROCESSING CLASS ###
#############################

class DataProcessing:
    """
    Process red teamed data
    """

    def __init__(self, robot="val_clr", environment="lunar_habitat", initialize_weighted_datasets=False, weighted=[]):
        # set internal paramters
        self.robot_name = robot
        self.environment_name = environment

        # initialize data info
        self.info = DatasetInfo()

        # initialize data formatting
        self.col_info = DatasetColumns(self.robot_name, self.environment_name)

        # initialize data frame
        self.initialize_data_frame(initialize_weighted_datasets, weighted)

    ######################
    ### INITIALIZATION ###
    ######################

    def initialize_data_frame(self, initialize_weighted_datasets=False, weighted=[]):
        # get file name
        _, self.data_file_name = self.info.get_combined_dataset_full_path(self.robot_name, self.environment_name)
        _, self.rrs_data_file_name = self.info.get_rrs_dataset_full_path(self.robot_name, self.environment_name)
        _, self.cfa_data_file_name = self.info.get_cfa_dataset_full_path(self.robot_name, self.environment_name)
        _, (_, self.data_limited_file_name) = self.info.get_combined_dataset_full_path(self.robot_name, self.environment_name, limited_cfa=True)

        # create data frame
        self.df_full = pd.read_csv(self.data_file_name)
        self.df_rrs = pd.read_csv(self.rrs_data_file_name)
        self.df_cfa = pd.read_csv(self.cfa_data_file_name)
        self.df = pd.read_csv(self.data_limited_file_name)

        if initialize_weighted_datasets:
            # check if weights given
            if len(weighted) == 0:
                # set to all weights
                weighted = list(range(2,10))

            # create dictionary for weighted datasets
            self.weighted_dfs = {}
            for i in weighted:
                # get file name
                _, (_, weighted_file_name) = self.info.get_combined_dataset_full_path(self.robot_name, self.environment_name, limited_cfa=True, weight=i)
                self.weighted_dfs[i] = pd.read_csv(weighted_file_name)

    #######################
    ### DATASET HELPERS ###
    #######################

    def prep_data_for_model_training(self, df=None, feature_indices=None):
        # get X and Y data
        X, Y = self.get_data_X_Y(df, feature_indices)
        self.print_X_Y_shapes(X, Y)

        # split data
        X_train, X_test, y_train, y_test = self.train_test_split_data(X, Y)
        self.print_train_test_data_shapes(X_train, X_test, y_train, y_test)
        self.print_train_test_target_value_counts(y_train, y_test)

        return X, Y, X_train, X_test, y_train, y_test

    def get_data_X_Y(self, df=None, feature_indices=None):
        if df is None:
            df = self.df

        # select explanatory variables
        X = None
        if feature_indices is None:
            # use all features
            X = df.drop([self.col_info.get_col_name_for_action(), self.col_info.get_col_name_for_action_encoded()], axis=1)
        else:
            # use given features
            X = df.iloc[:,feature_indices]

        # select target variable
        Y = df[self.col_info.get_col_name_for_action_encoded()]

        return X, Y

    def train_test_split_data(self, X, Y):
        # perform train test split
        X_train, X_test, y_train, y_test = train_test_split(X, Y, random_state=0)

        return X_train, X_test, y_train, y_test

    def correlation_matrix(self, df=None, print_detailed=False):
        if df is None:
            df = self.df

        # drop out categorical variable
        df_quant = df.drop([self.col_info.get_col_name_for_action()], axis=1)

        # compute correlation matrix
        corr_matrix = df_quant.corr()

        print("CORRELATION MATRIX:")
        print(corr_matrix)

        if print_detailed:
            for i in range(corr_matrix.shape[1]):
                print(corr_matrix.iloc[:,i])

        return

    ###################################
    ### DATASET INTERACTION HELPERS ###
    ###################################

    def get_interaction_data(self, df=None, feature_indices=None):
        # prepare data
        X, Y, X_train, X_test, y_train, y_test = self.prep_data_for_model_training(df, feature_indices)

        # initialize polynomial features
        poly = PolynomialFeatures(interaction_only=True)
        # compute polynomial features
        X_interactions = poly.fit_transform(X)

        # format as dataframe
        Xt = pd.DataFrame(X_interactions,columns=poly.get_feature_names_out())
        Xt.columns = Xt.columns.str.replace(' ', '_INT_')
        Xt = Xt.drop(['1'], axis=1)

        # get target column names
        target_col_names = self.col_info.get_col_name_for_action()
        target_col = self.col_info.get_col_name_for_action_encoded()

        data_interactions = pd.concat([Xt.reset_index(drop=True),
                                       df[target_col_names].reset_index(drop=True),
                                       df[target_col].reset_index(drop=True)], axis=1)
        # some weird things happening with additional rows being added, so reset index to fix this

        # print out info about interactions dataframe
        print("Interactions dataset head:")
        print(data_interactions.head())
        print("Interactions dataset columns:")
        print(data_interactions.columns)
        print("Interactions dataset info:")
        print(data_interactions.info())
        print("Interactions dataset shape:")
        print(data_interactions.shape)

        return data_interactions, Xt

    #######################################
    ### DATA SELECTION HELPER FUNCTIONS ###
    #######################################

    def get_feature_indices(self, columns_include=[], columns_exclude=[], columns_exactly=None, data=None):
        if data is None:
            data = self.df

        # get list of all column names
        all_col_names = list(data.columns)

        # initialize list of column names
        cols = []
        if len(columns_include) == 0:
            cols = deepcopy(all_col_names)

        # if columns exactly is non-empty, use those exactly and ignore others
        if columns_exactly is not None:
            col_idxs = []
            for col in columns_exactly:
                col_idxs.append(all_col_names.index(col))
            print("selected columns:", columns_exactly)
            print("selected column indices:", col_idxs)
            return columns_exactly, col_idxs

        # loop through column names and find columns that should be included
        for col in data.columns:
            # loop through possible column include stubs
            for poss_col in columns_include:
                if poss_col in col:
                    cols.append(col)
                    # break out of inner-most for loop
                    break

        # initialize final list of column names
        final_cols = []

        # loop through column names and exclude columns if necessary
        for col in cols:
            keep_col = True
            # loop through possible column exclude stubs
            for ex_col in columns_exclude:
                if ex_col in col:
                    keep_col = False
                    # break out of inner-most for loop
                    break
            # check if we are keeping this column
            if keep_col:
                final_cols.append(col)

        # get indices of column names
        col_idxs = []
        for col in final_cols:
            col_idxs.append(all_col_names.index(col))

        print("selected columns:", final_cols)
        print("selected column indices:", col_idxs)

        return final_cols, col_idxs

    #####################
    ### MODEL HELPERS ###
    #####################

    def create_feature_combos(self, data, num_cols=None):
        # get target column names
        target_col_names = self.col_info.get_col_name_for_action()
        target_col = self.col_info.get_col_name_for_action_encoded()

        # ensure data columns do not include targets
        data_cols = [i for i in data.columns if (i != target_col) and (i != target_col_names)]

        # create all combinations of features
        feature_combos = []

        # if given optional number of columns argument
        if num_cols is not None:
            feature_combos = list(combinations(range(0,len(data_cols)),num_cols))
            return feature_combos

        # otherwise, find all columns
        for r in range(1,len(data.columns)+1):
            for c in combinations(range(0,len(data_cols)),r):
                feature_combos.append(c)
        print("Found " + str(len(feature_combos)) + " feature combinations over " + str(len(data_cols)) + " features")

        return feature_combos

    def train_and_evaluate_model(self, X, Y, X_train, X_test, y_train, y_test):
        # create formula string
        formula, training_data = self.prep_formula_and_training_data(X_train, y_train, X.columns)

        # build model
        logit_model = self.build_and_train_model(formula, training_data)
        if logit_model is None:
            return False, None

        # check how training went
        good_model = self.validate_model_training(logit_model, X.columns)

        # if good model, validate
        if good_model:
            self.evaluate_model(logit_model, X_test, y_test)

        return good_model, logit_model

    def prep_formula_and_training_data(self, X_train, y_train, col_names):
        # get all columns in dataset
        all_columns = None
        if len(col_names) == 1:
            all_columns = col_names[0]
        else:
            all_columns = ' + '.join(col_names)

        # get target column name
        target_col = self.col_info.get_col_name_for_action_encoded()

        # create the formula string
        formula = target_col + " ~ " + all_columns
        print("formula: ", formula)
        print()

        # combine training predictors and targets into one dataframe
        training_data = pd.concat([X_train, y_train], axis=1)
        print("training data shape:",training_data.shape)

        return formula, training_data

    def build_and_train_model(self, formula, training_data):
        try:
            # create multinomial logistic regression modelodel
            logit_model = smf.mnlogit(formula, data=training_data).fit(maxiter=150)
        except LinAlgError as ex:
            print("*** ERROR: singular matrix")
            return None

        return logit_model

    def validate_model_training(self, logit_model, col_names):
        # check convergence or non-nan function value
        if logit_model.mle_retvals['converged'] or not np.isnan(logit_model.mle_retvals['fopt']):
            # check if converged and nan function value
            if np.isnan(logit_model.mle_retvals['fopt']):
                print("\nConverged, but NaN function value; bad model")
                return False
            print("\nPROMISING MODEL with features: ", col_names)
            print()
            print(logit_model.summary())
            print()
            return True

        return False

    def evaluate_model(self, logit_model, X_test, y_test):
        # compute predictions
        y_test_pred_prob = logit_model.predict(X_test)
        y_test_pred = np.argmax(np.array(y_test_pred_prob), axis=1)

        # compute accuracy
        print("Test accuracy:", accuracy_score(y_test, y_test_pred))

        # look at confusion matrix to evaluate model more closely
        cm = confusion_matrix(y_test, y_test_pred)
        print("Confusion matrix:")
        print(cm)
        # ConfusionMatrixDisplay(confusion_matrix=cm, display_labels=None).plot()

        return

    ###################################
    ### LOGISTIC REGRESSION HELPERS ###
    ###################################

    def explore_possible_models(self, df=None, feature_indices=None, explore_weights=None):
        if explore_weights is not None:
            self.__explore_possible_models_with_weights(feature_indices, explore_weights)
        else:
            self.__explore_possible_models(df, feature_indices)
        return

    def __explore_possible_models(self, df=None, feature_indices=None):
        if df is None:
            df = self.df

        if feature_indices is not None:
            df = df.iloc[:,feature_indices]

        # create all combinations of features
        feature_combos = self.create_feature_combos(df)

        # explore combinations
        self.explore_feature_combinations(df, feature_combos)

        return

    def __explore_possible_models_with_weights(self, feature_indices=None, explore_weights=[]):
        # set weights
        if len(explore_weights) == 0:
            # set all possible weights
            explore_weights = list(self.weighted_dfs.keys())

        # loop through all possible weights
        for i in explore_weights:
            # get dataset
            weighted_df = self.weighted_dfs[i]
            print("\n\n\n==============================")
            print("***** EXPLORING WEIGHTED DATASET *****")
            print("RRS : CFA = {} : 1".format(i))
            # explore models over this dataset
            self.__explore_possible_models(weighted_df, feature_indices)
            print("\n\n\n==============================\n\n\n")

        return

    def explore_possible_models_with_interactions(self, df=None, feature_indices=None):
        if df is None:
            df = self.df

        data_interactions, Xt = self.get_interaction_data(df, feature_indices)

        # create all combinations of features
        feature_combos = self.create_feature_combos(Xt)

        # explore combinations
        self.explore_feature_combinations(data_interactions, feature_combos)

        return

    def explore_feature_combinations(self, data, feature_combos):
        # explore combinations
        for combo in feature_combos:
            print("==========")
            print("***** ANALYSIS FOR VARIABLES: *****")
            var_list = list(combo)
            print([data.columns[i] for i in var_list])
            self.run_logistic_regression_analysis(data, var_list)
            print("==========\n\n\n")

        return

    def run_logistic_regression_analysis(self, df=None, feature_indices=None):
        if df is None:
            df = self.df

        # get training and testing data
        X, Y, X_train, X_test, y_train, y_test = self.prep_data_for_model_training(df, feature_indices)

        # create, train, and evaluate logistic regression model
        promising_model, logit_model = self.train_and_evaluate_model(X, Y, X_train, X_test, y_train, y_test)

        return promising_model, logit_model

    ##################
    ### SAVE MODEL ###
    ##################

    def save_model_to_file(self, model, model_name):
        print("Saving model to file...")

        # get file name
        model_file_name = "{0}{1}_{2}_{3}{4}".format(
            self.info.models_dir,
            self.robot_name,
            self.environment_name,
            model_name,
            self.info.model_file_end
        )

        # save model
        pickle.dump(model, open(model_file_name, 'wb'))

        print("Saved model to file! Model location: {}".format(model_file_name))

        return

    ################
    ### PRINTING ###
    ################

    def print_summary_info(self, df=None):
        if df is None:
            df = self.df

        print("DATASET SUMMARY")
        print()
        print("*** COLUMN NAMES:")
        print(df.columns)
        print()
        print("*** INFO:")
        print(df.info())
        print()
        print("*** SHAPE:")
        print(df.shape)
        print()
        print("*** HEAD:")
        print(df.head())
        print()
        return

    def print_target_value_counts(self, df=None):
        if df is None:
            df = self.df

        print("RISK MITIGATING ACTION ENCODINGS")
        for k,v in self.col_info.action_encoding.items():
            print("    {} : {}".format(v,k))
        print()
        print("RISK MITIGATING ACTION VALUE COUNTS")
        print(df[self.col_info.get_col_name_for_action_encoded()].value_counts())
        return

    def print_X_Y_shapes(self, X, Y):
        print("EXPLANATORY AND TARGET VARIABLE SHAPES")
        print()
        print("*** X SHAPE:", X.shape)
        print("*** Y SHAPE:", Y.shape)
        print()
        return

    def print_train_test_data_shapes(self, X_train, X_test, y_train, y_test):
        print("TRAINING AND TESTING DATA SHAPES")
        print()
        print("*** TRAINING DATA SHAPES:")
        print("        X shape:", X_train.shape)
        print("        Y shape:", y_train.shape)
        print()
        print("*** TESTING DATA SHAPES:")
        print("        X shape:", X_test.shape)
        print("        Y shape:", y_test.shape)
        print()
        return

    def print_train_test_target_value_counts(self, y_train, y_test):
        print("RISK MITIGATING ACTION ENCODINGS")
        for k,v in self.col_info.action_encoding.items():
            print("    {} : {}".format(v,k))
        print()
        print("RISK MITIGATING ACTION VALUE COUNTS")
        print("*** TRAINING DATA:")
        print(y_train.value_counts())
        print()
        print("*** TESTING DATA:")
        print(y_test.value_counts())
        print()
        return



#####################
### MAIN FUNCTION ###
#####################

if __name__ == '__main__':
    # set node name
    node_name = "SARDataProcessingNode"
    param_prefix = "/" + node_name + "/"

    # get ROS parameters
    robot_name = rospy.get_param(param_prefix + 'robot', "all")
    env_name = rospy.get_param(param_prefix + 'environment', "all")

    # initialize node
    rospy.init_node(node_name)

    rospy.loginfo("[SAR Data Processing Node] Starting to process all data!")

    # create data info
    info = DatasetInfo()

    # initialize robots and envs to be processed
    robots = []
    envs = []

    # check robot and env inputs
    if robot_name == "all":
        robots = info.supported_robots
    elif robot_name in info.supported_robots:
        robots = [robot_name]
    else:
        rospy.logwarn("[SAR Data Processing Node] Unrecognized robot %s, defaulting to 'all'", robot_name)
        robots = info.supported_robots

    if env_name == "all":
        envs = info.supported_envs
    elif env_name in info.supported_envs:
        envs = [env_name]
    else:
        rospy.logwarn("[SAR Data Processing Node] Unrecognized environment %e, defaulting to 'all'", env_name)

    # process all data
    for robot in robots:
        for env in envs:
            # create data pre-processing object
            data_preprocess = DataPreprocessing(robot, env)

            # create dataset csvs
            rospy.loginfo("[SAR Data Processing Node] Processing data for robot %s in %s environment",
                          robot.upper(), env.upper())
            df_rrs, df_cfa = data_preprocess.convert_yamls_to_dataset_csv()
            rospy.loginfo("[SAR Data Processing Node] Created datasets for robot %s in %s environment",
                          robot.upper(), env.upper())
            data_preprocess.create_weighted_limited_datasets(df1=df_rrs, df2=df_cfa)
            rospy.loginfo("[SAR Data Processing Node] Created weighted datasets for robot %s in %s environment",
                          robot.upper(), env.upper())

    rospy.loginfo("[SAR Data Processing Node] Completed data processing!")

    rospy.loginfo("[SAR Data Processing Node] Node stopped, all done!")
