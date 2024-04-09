"""
Counter Factual Policy Data Point Class
Emily Sheetz, NSTGRO VTE 2024
"""

from policy_data_point import PolicyDataPoint

class CounterFactualPolicyDataPoint(PolicyDataPoint):
    def __init__(self, conditions=[],
                       consequences_before_action=[],
                       action="unnamed_action",
                       consequences_after_action=[]):
        # initialize super class
        super(CounterFactualPolicyDataPoint, self).__init__(conditions,
                                                            consequences_before_action,
                                                            action,
                                                            consequences_after_action)
