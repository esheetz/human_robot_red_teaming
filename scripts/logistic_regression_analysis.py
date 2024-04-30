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
    # initialize flags for run
    # NOTE that we likely only want ONE FLAG to be true at a time
    # correlation flags
    correlation = False
    print_detailed_correlation = False
    # exploring models flag (for identifying features of interest)
    explore_models = False
    # explore interactions flag (for exploring interactions over features of interest)
    explore_interactions = False
    # build best model
    build_promising_model = True

    # create data class
    data = DataProcessing(robot="val_clr",
                          environment="lunar_habitat",
                          initialize_weighted_datasets=True,
                          weighted=[9])

    # # print summary info for whole dataset
    data.print_summary_info()
    data.print_target_value_counts()

    # correlation matrix
    if correlation:
        data.correlation_matrix(print_detailed=print_detailed_correlation)

    # feature_names, feature_idxs = data.get_feature_indices(columns_exclude=["COND_NAME_", "COND_LIKELI_", "CONSEQ_PRE_ACT_", "CONSEQ_POST_ACT_", "RISK_MITIGATING_ACTION", "RISK_MITIGATING_ACTION_ENCODED"])
    feature_names, feature_idxs = data.get_feature_indices(columns_include=["STATE_CONSEQ","COND_RISK"])

    # explore possible models
    if explore_models:
        data.explore_possible_models(feature_indices=feature_idxs, explore_weights=[7,8,9])

    # explore possible interactions
    if explore_interactions:
        data.explore_possible_models_with_interactions(df=data.weighted_dfs[9], feature_indices=feature_idxs)

    # logistic regression analysis and final training
    if build_promising_model:
        promising_model, logit_model = data.run_logistic_regression_analysis(df=data.weighted_dfs[9], feature_indices=feature_idxs)
        data.save_model_to_file(logit_model, "cond_risk_state_conseq")
