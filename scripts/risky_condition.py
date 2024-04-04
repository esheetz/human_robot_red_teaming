"""
Risky Condition Class
Emily Sheetz, NSTGRO VTE 2024
"""

from likelihood_consequence_risk import LikelihoodLevels, ConsequenceClasses, RiskScores

class RiskyCondition:
    def __init__(self, name="unnamed_condition",
                       likelihood=LikelihoodLevels.get_max(),
                       consequence=ConsequenceClasses.get_max(),
                       consequence_states=[]):
        # set internal parameters
        self.name = str(name)
        self.consequence_states = tuple(sorted([str(i) for i in consequence_states]))

        # set given likelihood and consequence values
        self.set_likelihood_level(likelihood)
        self.set_consequence_class(consequence)

        # compute corresponding risk and safety scores
        self.compute_risk_score()
        self.compute_safety_score()

    #######################
    ### GETTERS/SETTERS ###
    #######################

    def get_condition_name(self):
        return self.name

    def get_consequence_states(self):
        return self.consequence_states

    def set_likelihood_level(self, likelihood):
        # error check likelihood value
        self.likelihood = LikelihoodLevels.error_check(likelihood)
        return

    def get_likelihood_level(self):
        return self.likelihood

    def get_likelihood_level_name(self):
        return LikelihoodLevels.get_level_name(self.likelihood)

    def set_consequence_class(self, consequence):
        # error check consequence value
        self.consequence = ConsequenceClasses.error_check(consequence)
        return

    def get_consequence_class(self):
        return self.consequence

    def get_consequence_class_name(self):
        return ConsequenceClasses.get_class_name(self.consequence)

    def get_risk_score(self):
        return self.risk_score

    def get_risk_score_name(self):
        return self.risk_score_name

    def get_matrix_risk_score(self):
        return self.matrix_risk_score

    def get_matrix_risk_score_name(self):
        return self.matrix_risk_score_name

    def get_safety_score(self):
        return self.safety_score

    def get_matrix_safety_score(self):
        return self.matrix_safety_score

    ##########################
    ### RISK/SAFETY SCORES ###
    ##########################

    def compute_risk_score(self):
        # compute risk score
        self.risk_score = RiskScores.compute_risk_score(self.likelihood, self.consequence)
        # get corresponding risk score name
        self.risk_score_name = RiskScores.get_score_name(self.likelihood, self.consequence)
        # compute (somewhat more interpretable) risk assessment matrix score and name
        self.matrix_risk_score = RiskScores.compute_matrix_risk_score(self.likelihood, self.consequence)
        self.matrix_risk_score_name = RiskScores.get_matrix_score_name(self.matrix_risk_score)
        return

    def compute_safety_score(self):
        # compute safety score [0,1) based on risk score
        self.safety_score = RiskScores.compute_safety_score(self.likelihood, self.consequence)
        # compute (somewhat more interpretable) safety score
        self.matrix_safety_score = RiskScores.compute_matrix_safety_score(self.likelihood, self.consequence)
        return

    ################################################
    ### VALIDATE AGAINST CONSEQUENCE STATE SPACE ###
    ################################################

    def validate_condition(self, consequence_space_names):
        for state in self.consequence_states:
            if state not in consequence_space_names:
                print("ERROR: consequence " + state + " not in consequence state space: ", consequence_space_names)
                return False
        # if we get here, every consequence exists in consequence space
        return True
