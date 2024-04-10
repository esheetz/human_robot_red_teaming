"""
Policy Data Point Class
Emily Sheetz, NSTGRO VTE 2024
"""

class PolicyDataPoint:
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

    def get_policy_data_point_dictionary_key(self):
        return (self.conditions, self.consequences_before_action)

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
