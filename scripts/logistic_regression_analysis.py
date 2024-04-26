#!/usr/bin/env python3
"""
Build Grasp State Model
Emily Sheetz, NSTGRO VTE 2024
"""

from copy import deepcopy

# dataset class
from data_processing import DataProcessing

##################
### RUN SCRIPT ###
##################

if __name__ == '__main__':
    # create data class
    data = DataProcessing()

    # # print summary info for whole dataset
    data.print_summary_info()
    data.print_target_value_counts()

    # # print summary info for only subset of data
    # data.print_summary_info(data.df_rrs)
    # data.print_target_value_counts(data.df_rrs)

    # get features
    # feature_names, feature_idxs = data.get_feature_indices(columns_exclude=["CONSEQ_POST_ACT_","RISK_MITIGATING_ACTION","RISK_MITIGATING_ACTION_ENCODED"])#, data=data.df_rrs)
    # feature_names, feature_idxs = data.get_feature_indices(columns_include=["RISK"],columns_exclude=["RISK_MITIGATING_ACTION","RISK_MITIGATING_ACTION_ENCODED"],data=data.df_rrs)
    # feature_names, feature_idxs = data.get_feature_indices(columns_include=["RISK"])

    # feature_names, feature_idxs = data.get_feature_indices(columns_exclude=["COND_NAME_", "COND_LIKELI_", "CONSEQ_PRE_ACT_", "CONSEQ_POST_ACT_", "RISK_MITIGATING_ACTION", "RISK_MITIGATING_ACTION_ENCODED"])
    feature_names, feature_idxs = data.get_feature_indices(columns_include=["STATE","RISK_MITIGATING"])

    # logistic regression analysis
    # promising_model, logit_model = data.run_logistic_regression_analysis()#feature_indices=feature_idxs) #data.df_rrs, feature_idxs)

    # correlation matrix
    # data.correlation_matrix() #data.df_rrs

    # explore possible interactions
    data.explore_possible_models(feature_indices=feature_idxs)

