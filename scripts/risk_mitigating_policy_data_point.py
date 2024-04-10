"""
Risk Mitigating Policy Data Point Class
Emily Sheetz, NSTGRO VTE 2024
"""

from policy_data_point import PolicyDataPoint

class RiskMitigatingPolicyDataPoint(PolicyDataPoint):
    def __init__(self, conditions=[],
                       consequences_before_action=[],
                       action="unnamed_action",
                       consequences_after_action=[]):
        # initialize super class
        super(RiskMitigatingPolicyDataPoint, self).__init__(conditions,
                                                            consequences_before_action,
                                                            action,
                                                            consequences_after_action)

    ################################################################
    ### CHECK DUPLICATED CONDITIONS / CONSEQUENCES BEFORE ACTION ###
    ################################################################

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

    def check_and_get_conflicting_data_point_action(self, policy):
        # check policy type
        if type(policy) == dict:
            return self.__check_and_get_conflicting_data_point_action_policy(policy)
        elif type(policy) == RiskMitigatingPolicyDataPoint:
            return self.__check_and_get_conflicting_data_point_action(policy)
        else:
            print("WARN: unrecognized policy type: " + type(policy) + " ; assuming no conflicting actions to get")
            return False, None, None

    def check_conflicting_data_point_action(self, policy):
        # check policy type
        if type(policy) == dict:
            return self.__check_conflicting_data_point_action_policy(policy)
        elif type(policy) == RiskMitigatingPolicyDataPoint:
            return self.__check_conflicting_data_point_action(policy)
        else:
            print("WARN: unrecognized policy type: " + type(policy) + " ; assuming no conflicting actions")
            return False

    ###################################################
    ### CHECK CONFLICTING CONSEQUENCES AFTER ACTION ###
    ###################################################

    def check_and_get_conflicting_data_point_consequences(self, policy):
        # check policy type
        if type(policy) == dict:
            return self.__check_and_get_conflicting_data_point_consequences_policy(policy)
        elif type(policy) == RiskMitigatingPolicyDataPoint:
            return self.__check_and_get_conflicting_data_point_consequences(policy)
        else:
            print("WARN: unrecognized policy type: " + type(policy) + " ; assuming no conflicting consequences to get")
            return False, None, None

    def check_conflicting_data_point_consequences(self, policy):
        # check policy type
        if type(policy) == dict:
            return self.__check_conflicting_data_point_consequences_policy(policy)
        elif type(policy) == RiskMitigatingPolicyDataPoint:
            return self.__check_conflicting_data_point_consequences(policy)
        else:
            print("WARN: unrecognized policy type: " + type(policy) + " ; assuming no conflicting consequences")
            return False

    ##########################################################
    ### PRIVATE HELPERS FOR CHECKING DUPLICATED CONDITIONS ###
    ##########################################################

    def __check_data_point_in_policy(self, policy : dict):
        # check if data point is contained in given policy
        return self.get_policy_data_point_dictionary_key() in policy.keys()

    def __check_data_point_conditions(self, policy_data_point): # policy_data_point : RiskMitigatingPolicyDataPoint
        # check if conditions are the same
        return self.get_policy_data_point_dictionary_key() == policy_data_point.get_policy_data_point_dictionary_key()

    ########################################################
    ### PRIVATE HELPERS FOR CHECKING CONFLICTING ACTIONS ###
    ########################################################

    def __check_and_get_conflicting_data_point_action_policy(self, policy : dict):
        # check if data point has same conditions but different action from data points in given policy
        conflict = (self.__check_data_point_in_policy(policy) and
                    (self.__check_conflicting_data_point_action(policy[self.get_policy_data_point_dictionary_key()])))

        # check conflict and return actions accordingly
        if conflict:
            # return conflicting actions
            return (conflict,
                    self.action,
                    policy[self.get_policy_data_point_dictionary_key()].get_policy_data_point_action_name())
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
            return (conflict,
                    self.action,
                    policy_data_point.get_policy_data_point_action_name())
        else:
            # no conflicting actions
            return (conflict, None, None)

    def __check_conflicting_data_point_action_policy(self, policy : dict):
        conflict, _, _ = self.__check_and_get_conflicting_data_point_action_policy(policy)
        return conflict

    def __check_conflicting_data_point_action(self, policy_data_point): # policy_data_point : RiskMitigatingPolicyDataPoint
        conflict, _, _ = self.__check_and_get_conflicting_data_point_action(policy_data_point)
        return conflict

    ##########################################################################
    ### PRIVATE HELPERS FOR CHECKING CONFLICTING CONSEQUENCES AFTER ACTION ###
    ##########################################################################

    def __check_and_get_conflicting_data_point_consequences_policy(self, policy : dict):
        # check if data point has same key but different consequences from data points in given policy
        conflict = (self.__check_data_point_in_policy(policy) and
                    (self.__check_conflicting_data_point_consequences(policy[self.get_policy_data_point_dictionary_key()])))

        # check conflict and return consequences accordingly
        if conflict:
            # return conflicting consequences
            return (conflict,
                    self.consequences_after_action,
                    policy[self.get_policy_data_point_dictionary_key()].get_policy_data_point_consequences_after_action_names())
        else:
            # no conflicting consequences
            return (conflict, None, None)

    def __check_and_get_conflicting_data_point_consequences(self, policy_data_point): # policy_data_point : RiskMitigatingPolicyDataPoint
        # check if data point has same key but different consequences from given data point
        conflict = (self.__check_data_point_conditions(policy_data_point) and
                    (self.consequences_after_action != policy_data_point.get_policy_data_point_consequences_after_action_names()))

        # check conflict and return consequences accordingly
        if conflict:
            # return conflicting consequences
            return (conflict,
                    self.consequences_after_action,
                    policy_data_point.get_policy_data_point_consequences_after_action_names())
        else:
            # no conflicting consequences
            return (conflict, None, None)

    def __check_conflicting_data_point_consequences_policy(self, policy : dict):
        conflict, _, _ = self.__check_and_get_conflicting_data_point_consequences_policy(policy)
        return conflict

    def __check_conflicting_data_point_consequences(self, policy_data_point): # policy_data_point : RiskMitigatingPolicyDataPoint
        conflict, _, _ = self.__check_and_get_conflicting_data_point_consequences(policy_data_point)
        return conflict
