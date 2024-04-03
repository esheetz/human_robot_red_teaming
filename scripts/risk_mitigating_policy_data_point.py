"""
Risk Mitigating Policy Data Point Class
Emily Sheetz, NSTGRO VTE 2024
"""

class RiskMitigatingPolicyDataPoint:
    def __init__(self, conditions=[],
                       consequences_before_action=[],
                       action="unnamed_action",
                       consequences_after_action=[]):
        # set internal parameters
        self.conditions = tuple(sorted([str(i) for i in conditions]))
        self.consequences_before_action = tuple(sorted([str(i) for i in consequences_before_action]))
        self.action = str(action)
        self.consequences_after_action = tuple(sorted([str(i) for i in consequences_after_action]))

    #######################
    ### GETTERS/SETTERS ###
    #######################

    def get_policy_data_point_condition_names(self):
        return self.conditions

    def get_policy_data_point_action_name(self):
        return self.action

    def get_policy_data_point_consequences_before_action_names(self):
        return self.consequences_before_action

    def get_policy_data_point_consequences_after_action_names(self):
        return self.consequences_after_action

    def get_policy_data_point_consequence_names(self):
        return (self.consequences_before_action, self.consequences_after_action)

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

    ############################################
    ### VALIDATE AGAINST STATE/ACTION SPACES ###
    ############################################

    def validate_data_point(self, state_space_names, action_space_names, consequence_space_names):
        return (self.validate_data_point_state_space(state_space_names) and
                self.validate_data_point_action_space(action_space_names) and
                self.validate_data_point_consequence_space(consequence_space_names))

    def validate_data_point_state_space(self, state_space_names):
        for cond in self.conditions:
            if cond not in state_space_names:
                print("ERROR: condition " + cond + " not in state space: ", state_space_names)
                return False
        # if we get here, every condition exists in state space
        return True

    def validate_data_point_action_space(self, action_space_names):
        if self.action not in action_space_names:
            print("ERROR: action " + self.action + " not in action space: ", action_space_names)
            return False
        # if we get here, action exists in action space
        return True

    def validate_data_point_consequence_space(self, consequence_space_names):
        # check before action consequences
        for conseq in self.consequences_before_action:
            if conseq not in consequence_space_names:
                print("ERROR: consequence " + conseq + " not in consequence space: ", consequence_space_names)
                return False
        # check after action consequences
        for conseq in self.consequences_after_action:
            if conseq not in consequence_space_names:
                print("ERROR: consequence " + conseq + " not in consequence space: ", consequence_space_names)
        # if we get here, ever consequence exists in consequence space
        return True

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
