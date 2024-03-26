"""
Risk Mitigating Policy Data Point Class
Emily Sheetz, NSTGRO VTE 2024
"""

class RiskMitigatingPolicyDataPoint:
    def __init__(self, conditions=[],
                       action="unnamed_action"):
        # set internal parameters
        self.conditions = tuple([str(i) for i in conditions])
        self.action = str(action)

    #######################
    ### GETTERS/SETTERS ###
    #######################

    def get_policy_data_point_condition_names(self):
        return self.conditions

    def get_policy_data_point_action_name(self):
        return self.action

    ###################################
    ### CHECK DUPLICATED CONDITIONS ###
    ###################################

    def check_data_point_duplicated(self, policy):
        # check policy type
        if type(policy) == dict:
            return self.__check_data_point_in_policy(policy)
        elif type(policy) == RiskMitigatingPolicyDataPoint:
            return self.__check_data_point_conditions(policy)
        else:
            print("WARN: unrecognized policy type: " + type(policy) + " ; assuming no duplicated conditions")
            return False

    #################################
    ### CHECK CONFLICTING ACTIONS ###
    #################################

    def check_and_get_conflicting_data_point(self, policy):
        # check policy type
        if type(policy) == dict:
            return self.__check_and_get_conflicting_data_point_policy(policy)
        elif type(policy) == RiskMitigatingPolicyDataPoint:
            return self.__check_and_get_conflicting_data_point_action(policy)
        else:
            print("WARN: unrecognized policy type: " + type(policy) + " ; assuming no conflicting actions to get")
            return False, None, None

    def check_conflicting_data_point(self, policy):
        # check policy type
        if type(policy) == dict:
            return self.__check_conflicting_data_point_policy(policy)
        elif type(policy) == RiskMitigatingPolicyDataPoint:
            return self.__check_conflicting_data_point_action(policy)
        else:
            print("WARN: unrecognized policy type: " + type(policy) + " ; assuming no conflicting actions")
            return False

    ##########################################################
    ### PRIVATE HELPERS FOR CHECKING DUPLICATED CONDITIONS ###
    ##########################################################

    def __check_data_point_in_policy(self, policy : dict):
        # check if data point is contained in given policy
        return self.conditions in policy.keys()

    def __check_data_point_conditions(self, policy_data_point): # policy_data_point : RiskMitigatingPolicyDataPoint
        # check if conditions are the same
        return self.conditions == policy_data_point.get_policy_data_point_condition_names()

    ########################################################
    ### PRIVATE HELPERS FOR CHECKING CONFLICTING ACTIONS ###
    ########################################################

    def __check_and_get_conflicting_data_point_policy(self, policy : dict):
        # check if data point has same conditions but different action from data points in given policy
        conflict = (self.__check_data_point_in_policy(policy) and
                    (self.__check_conflicting_data_point_action(policy[self.conditions])))

        # check conflict and return actions accordingly
        if conflict:
            # return conflicting actions
            return (conflict, self.action, policy[self.conditions].get_policy_data_point_action_name())
        else:
            # no conflicting actions
            return (conflict, None, None)

    def __check_and_get_conflicting_data_point_action(self, policy_data_point): # policy_data_point : RiskMitigatingPolicyDataPoint
        # check if data point has same conditions but different action from given data point
        conflict = (self.__check_data_point_conditions(policy_data_point) and
                    (self.action != policy_data_point.get_policy_data_point_action_name()))

        # check conflict and return actions accordingly
        if conflict:
            # return conflicting actions
            return (conflict, self.action, policy_data_point.get_policy_data_point_action_name())
        else:
            # no conflicting actions
            return (conflict, None, None)

    def __check_conflicting_data_point_policy(self, policy : dict):
        conflict, _, _ = self.__check_and_get_conflicting_data_point_policy(policy)
        return conflict

    def __check_conflicting_data_point_action(self, policy_data_point): # policy_data_point : RiskMitigatingPolicyDataPoint
        conflict, _, _ = self.__check_and_get_conflicting_data_point_action(policy_data_point)
        return conflict
