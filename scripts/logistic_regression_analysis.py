#!/usr/bin/env python3
"""
Build Grasp State Model
Emily Sheetz, NSTGRO VTE 2024
"""

# dataset class
from data_processing import DataProcessing

##################
### RUN SCRIPT ###
##################

if __name__ == '__main__':
    # create data class
    data = DataProcessing()

    # print summary info
    data.print_summary_info()
    data.print_target_value_counts()

    # # logistic regression analysis
    # promising_model, logit_model = data.run_logistic_regression_analysis()

    # explore possible interactions
    data.explore_possible_models()

