"""
Likelihood Levels Class
Consequence Classes Class
Risk Scores Class

Emily Sheetz, NSTGRO VTE 2024
"""

#########################
### LIKELIHOOD LEVELS ###
#########################

class LikelihoodLevels:
    levels = {
        1 : "improbable",
        2 : "remote",
        3 : "occasional",
        4 : "probable",
        5 : "frequent",
    }
    min_level = 1
    max_level = 5

    @staticmethod
    def get_min():
        return LikelihoodLevels.min_level

    @staticmethod
    def get_max():
        return LikelihoodLevels.max_level

    @staticmethod
    def get_level_name(likelihood) -> str:
        # verify likelihood
        if likelihood not in LikelihoodLevels.levels.keys():
            print("ERROR: unrecognized likelihood level " + str(likelihood))
            return "unknown"

        return LikelihoodLevels.levels[likelihood]

    @staticmethod
    def valid_value(likelihood) -> bool:
        # valid likelihood values are ints and between bounds
        return ((type(likelihood) == int) and
                (LikelihoodLevels.get_min() <= likelihood) and
                (likelihood <= LikelihoodLevels.get_max()))

    @staticmethod
    def error_check(likelihood) -> int:
        # check if valid value received
        if LikelihoodLevels.valid_value(likelihood):
            # return un-modified likelihood
            return likelihood
        else:
            # check reason for invalid input
            if type(likelihood) != int:
                # invalid due to type
                print("ERROR: given likelihood value is of type " + str(type(likelihood)) + 
                      ", but expected type is int; " +
                      "setting likelihood value to max")
            else:
                # invalid due to bounds
                print("ERROR: given likelihood value " + str(likelihood) + " is outside bounds " +
                      "[" + str(LikelihoodLevels.get_min()) + "," + str(LikelihoodLevels.get_max()) + "]; " +
                      "setting likelihood value to max")
            # something was wrong with given value
            return LikelihoodLevels.get_max()



###########################
### CONSEQUENCE CLASSES ###
###########################

class ConsequenceClasses:
    classes = {
        1 : "insignificant",
        2 : "minor",
        3 : "moderate",
        4 : "major",
        5 : "severe",
    }
    min_level = 1
    max_level = 5

    @staticmethod
    def get_min():
        return ConsequenceClasses.min_level

    @staticmethod
    def get_max():
        return ConsequenceClasses.max_level

    @staticmethod
    def get_class_name(consequence) -> str:
        # verify consequence
        if consequence not in ConsequenceClasses.classes.keys():
            print("ERROR: unrecognized consequence class " + str(consequence))
            return "unknown"

        return ConsequenceClasses.classes[consequence]

    @staticmethod
    def valid_value(consequence) -> bool:
        # valid consequence values are ints and between bounds
        return ((type(consequence) == int) and
                (ConsequenceClasses.get_min() <= consequence) and
                (consequence <= ConsequenceClasses.get_max()))

    @staticmethod
    def error_check(consequence) -> int:
        # check if valid value received
        if ConsequenceClasses.valid_value(consequence):
            # return un-modified consequence
            return consequence
        else:
            # check reason for invalid input
            if type(consequence) != int:
                # invalid due to type
                print("ERROR: given consequence value is of type " + str(type(consequence)) + 
                      ", but expected type is int; " +
                      "setting consequence value to max")
            else:
                # invalid due to bounds
                print("ERROR: given consequence value " + str(consequence) + " is outside bounds " +
                      "[" + str(ConsequenceClasses.get_min()) + "," + str(ConsequenceClasses.get_max()) + "]; " +
                      "setting consequence value to max")                
            # something was wrong with the given value
            return ConsequenceClasses.get_max()



###################
### RISK SCORES ###
###################

class RiskScores:
    scores = {
        # risk levels are of form (lower_bound, upper_bound) : "name"
        # with inclusive lower_bound and exclusive upper_bound
        (  1,  3 ) : "negligible",
        (  3,  4 ) : "low",
        (  4, 10 ) : "medium",
        ( 10, 17 ) : "high",
        ( 17, 26 ) : "extreme",
    }
    min_score = LikelihoodLevels.get_min() * ConsequenceClasses.get_min()
    max_score = LikelihoodLevels.get_max() * ConsequenceClasses.get_max()

    @staticmethod
    def get_min():
        return RiskScores.min_score

    @staticmethod
    def get_max():
        return RiskScores.max_score

    @staticmethod
    def get_raw_score_name(risk : int) -> str:
        # verify risk
        if (risk < RiskScores.get_min()) or (RiskScores.get_max() < risk):
            print("ERROR: unrecognized risk score " + str(risk))
            return "unknown"

        # find appropriate bounds
        for bounds in RiskScores.scores.keys():
            lower_bound, upper_bound = bounds
            if (lower_bound <= risk) and (risk < upper_bound):
                return RiskScores.scores[bounds]

        # should return at this point, but just in case
        return "unknown"

    @staticmethod
    def get_score_name(risk : float) -> str:
        # un-normalize risk
        raw_risk = int(risk * RiskScores.get_max())

        return RiskScores.get_raw_score_name(raw_risk)

    @staticmethod
    def compute_raw_risk_score(likelihood : int, consequence : int) -> int:
        # error check
        likelihood = LikelihoodLevels.error_check(likelihood)
        consequence = ConsequenceClasses.error_check(consequence)

        return likelihood * consequence

    @staticmethod
    def compute_risk_score(likelihood : int, consequence : int) -> float:
        raw_risk = RiskScores.compute_raw_risk_score(likelihood, consequence)

        # normalized risk score
        risk = raw_risk / RiskScores.get_max()

        return risk

    @staticmethod
    def compute_safety_score(likelihood : int, consequence : int) -> float:
        # compute risk
        risk = RiskScores.compute_risk_score(likelihood, consequence)

        # compute safety
        safety = 1.0 - risk

        return safety
