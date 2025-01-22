"""
Red Team Command Line Tools Class
Emily Sheetz, NSTGRO VTE 2024
"""

from risk_mitigating_policy_data_point import RiskMitigatingPolicyDataPoint
from red_team_policy import RedTeamPolicy

#############################
### COMMAND LINE PRINTING ###
#############################

class RedTeamCommandLinePrinting:

    @staticmethod
    def print_scenario_conditions(scenario_conditions, red_team):
        print("        RISK CONDITIONS:")
        for condition_name in sorted(scenario_conditions):
            # get condition from state space
            condition = red_team.get_risky_condition_with_name(condition_name)
            print("          - " + condition_name + "    [ " +
                  "L=" + str(condition.get_likelihood_level()) +
                  ", C=" + str(condition.get_consequence_class()) +
                  ", Risk=" + str(condition.get_matrix_risk_score()) +
                  " (" + condition.get_matrix_risk_score_name() + ") ]")
        return

    def print_scenario_consequences(scenario_consequences, conseq_name):
        print("        " + conseq_name.upper() + ":")
        for consequence_name in sorted(scenario_consequences):
            print("          - " + consequence_name)
        return

    def print_action(action):
        print("        RISK MITIGATING ACTION: " + action)
        return

    @staticmethod
    def print_red_teamed_scenario(scenario, scenario_consequences, red_team):
        print("    What should the robot do when the following RISK CONDITIONS (with possible future CONSEQUENCES) exist in the current state?")
        RedTeamCommandLinePrinting.print_scenario_conditions(scenario, red_team)
        RedTeamCommandLinePrinting.print_scenario_consequences(scenario_consequences, "future consequences")
        print()
        return

    @staticmethod
    def print_action_and_consequences(scenario_conditions, scenario_consequences, action, red_team, cf=False):
        if not cf:
            RedTeamCommandLinePrinting.print_risky_scenario_action_and_consequences(scenario_consequences, action)
        else:
            RedTeamCommandLinePrinting.print_counter_factual_action_and_consequences(scenario_conditions, scenario_consequences, action, red_team)
        return

    @staticmethod
    def print_risky_scenario_action_and_consequences(scenario_consequences, action):
        print("    Given the possible future CONSEQUENCES, after the robot takes the RISK MITIGATING ACTION, what future CONSEQUENCES can still occur?")
        RedTeamCommandLinePrinting.print_scenario_consequences(scenario_consequences, "consequences before action")
        RedTeamCommandLinePrinting.print_action(action)
        print()
        return

    @staticmethod
    def print_counter_factual_action_and_consequences(scenario_conditions, scenario_consequences, action, red_team):
        print("    Consider the following RISK CONDITIONS (with possible future CONSEQUENCES):")
        RedTeamCommandLinePrinting.print_scenario_conditions(scenario_conditions, red_team)
        RedTeamCommandLinePrinting.print_scenario_consequences(scenario_consequences, "future consequences")
        print("    After the robot takes the counter-factual RISK MITIGATING ACTION; what future CONSEQUENCES can occur?")
        RedTeamCommandLinePrinting.print_action(action)
        print()
        return

    @staticmethod
    def print_actions(action_space):
        print("    Please select the number of the appropriate action to take in this state:")
        print("        Robot's risk mitigating action space:")
        for i in range(len(action_space)):
            print("          " + str(i) + ". " + action_space[i])
        print("          " + str(len(action_space)) + ". [SKIP THIS SCENARIO]")
        print("    [type 'exit()' to quit]")
        print()
        return

    @staticmethod
    def print_consequences(conseq_space):
        print("    Please select the consequences possible after the action is taken (as a comma separated list of consequence numbers):")
        print("        Consequence state space:")
        for i in range(len(conseq_space)):
            print("          " + str(i) + ". " + conseq_space[i])
        print("          " + str(len(conseq_space)) + ". [NO REMAINING CONSEQUENCES]")
        print("          " + str(len(conseq_space)+1) + ". [SKIP THIS SCENARIO]")
        print("    [type 'exit()' to quit]")
        print()
        return

    @staticmethod
    def print_try_again_message():
        print("    Please try again.")
        print()
        return

    @staticmethod
    def print_separator():
        print()
        print()
        print()
        print("==============================================================================================================")
        print("==============================================================================================================")
        print("==============================================================================================================")
        print()
        print()
        print()
        return

    @staticmethod
    def print_substep_separator():
        print()
        print("- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -")
        print()
        return

    @staticmethod
    def print_skip_message():
        print("    *** Skipping this red teamed scenario, no new data points generated...")
        RedTeamCommandLinePrinting.print_separator()
        return

    @staticmethod
    def print_action_conflict_message(red_team_conditions, point_act, pol_act):
        print("    *** ERROR: this scenario is already in red teamed policy with different action")
        print("            Conditions:", red_team_conditions)
        print("            Just entered action:", point_act)
        print("            Action in stored policy:", pol_act)
        print("    Let's resolve this conflict now.")
        print("    NOTE: your next selection will overwrite the stored policy value.")
        print()
        return

    @staticmethod
    def print_consequence_conflict_message(red_team_conditions, action, point_conseqs, pol_conseqs):
        print("    *** ERROR: this scenario and action are already in red teamed policy with different post-action consequences")
        print("            Conditions:", red_team_conditions)
        print("            Action:", action)
        print("            Just entered consequences after action:", point_conseqs)
        print("            Consequences after action in stored policy:", pol_conseqs)
        print("    Let's resolve this conflict now.")
        print("    NOTE: your next selection will overwrite the stored policy value.")
        print()
        return

    @staticmethod
    def print_got_action_message(action):
        print("    *** Great! Got action:", action)
        RedTeamCommandLinePrinting.print_substep_separator()
        return

    @staticmethod
    def print_got_consequences_message(conseqs):
        print("    *** Great! Got consequences:", conseqs)
        RedTeamCommandLinePrinting.print_substep_separator()
        return

    @staticmethod
    def print_update_policy_message(red_team_conditions, red_team_consequences, action, conseqs):
        print("    *** Great! Updating policy!")
        print("            Conditions:", sorted(list(red_team_conditions)))
        print("            Consequences before action:", sorted(list(red_team_consequences)))
        print("            Action:", action)
        print("            Consequences after action:", sorted(list(conseqs)))
        RedTeamCommandLinePrinting.print_separator()
        return



############################################################
### USER INPUT PROCESSING - SELECT ACTION GIVEN SCENARIO ###
############################################################

class UserInputActionProcessing:

    @staticmethod
    def get_action_from_user_and_resolve_conflicts(red_team, red_team_conditions, red_team_consequences, action_space):
        # get action from user input
        continue_data_gen, abort, action = UserInputActionProcessing.get_action_from_user(red_team, red_team_conditions, red_team_consequences, action_space)
        if abort:
            return continue_data_gen, True, None

        # resolve conflicts
        continue_data_gen, abort, action = UserInputActionProcessing.resolve_action_conflict(red_team, red_team_conditions, red_team_consequences, action_space, action)
        if abort:
            return continue_data_gen, True, None

        return continue_data_gen, False, action

    @staticmethod
    def resolve_action_conflict(red_team, red_team_conditions, red_team_consequences, action_space, action):
        # create temporary policy data point
        temp_pol_point = RiskMitigatingPolicyDataPoint(conditions=red_team_conditions,
                                                       consequences_before_action=red_team_consequences,
                                                       action=action)

        # check for duplicates already in policy
        conflict, point_act, pol_act = temp_pol_point.check_and_get_conflicting_data_point_action(red_team.policy_data)
        if conflict:
            # report conflict
            RedTeamCommandLinePrinting.print_action_conflict_message(red_team_conditions, point_act, pol_act)

            # get action from user input
            continue_data_gen, abort, action = UserInputActionProcessing.get_action_from_user(red_team, red_team_conditions, red_team_consequences, action_space)
            if abort:
                return continue_data_gen, True, None

        return True, False, action

    @staticmethod
    def get_action_from_user(red_team, red_team_conditions, red_team_consequences, action_space):
        # initialize abort flag
        abort = False

        # get scenario input from user and unpack
        user_input = UserInputActionProcessing.scenario_input_process(red_team, red_team_conditions, red_team_consequences, action_space)
        continue_data_gen, action_idx = user_input

        # verify action is not None
        if action_idx is None:
            # no valid action received, check whether skipping scenario or quitting data generation
            if continue_data_gen:
                # skipping scenario
                RedTeamCommandLinePrinting.print_skip_message()
                return True, True, None
            else:
                # quitting
                return False, True, None

        # get action
        action = action_space[action_idx]
        RedTeamCommandLinePrinting.print_got_action_message(action)

        return continue_data_gen, abort, action

    @staticmethod
    def scenario_input_process(red_team, red_team_conditions, red_team_consequences, action_space):
        # initialize loop flag and action index
        got_valid_action = False
        action_idx = None

        # keep asking until valid input received
        while not got_valid_action:
            # print scenario
            RedTeamCommandLinePrinting.print_red_teamed_scenario(red_team_conditions, red_team_consequences, red_team)

            # print possible actions
            RedTeamCommandLinePrinting.print_actions(action_space)

            # ask for user input
            quit, skip, action_idx = UserInputActionProcessing.get_user_input_action(action_space)

            # check result
            if quit:
                return False, None
            if skip:
                return True, None
            if action_idx is None:
                RedTeamCommandLinePrinting.print_try_again_message()
                continue

            # valid action received
            break

        # return action index
        return True, action_idx

    @staticmethod
    def get_user_input_action(action_space):
        # prompt user for input
        val = UserInputActionProcessing.prompt_user_input_action()

        # check for quit
        if val == "exit()":
            return True, None, None

        # validate action index
        skip, action_idx = UserInputActionProcessing.validate_action_index(val, action_space)

        return False, skip, action_idx

    @staticmethod
    def prompt_user_input_action():
        val = input("    Action number: ")
        RedTeamCommandLinePrinting.print_substep_separator()
        return val

    @staticmethod
    def validate_action_index(val, action_space):
        # initialize value
        int_val = None

        # try to convert to int
        try:
            int_val = int(val)
        except:
            print("    *** INVALID INPUT: " + val + " cannot be converted to an integer.")
            print()
            return None, None

        # check indices
        if (int_val < 0) or (int_val > len(action_space)):
            print("    *** INVALID INPUT: received " + val + ", but must be in range [0," + str(len(action_space)) + "]")
            print()
            return None, None
        elif int_val == len(action_space):
            return True, None
        else:
            return False, int_val



################################################################
### USER INPUT PROCESSING - SELECT CONSEQUENCES GIVEN ACTION ###
################################################################

class UserInputConsequenceProcessing:

    @staticmethod
    def get_consequences_from_user_and_resolve_conflicts(red_team, red_team_conditions, red_team_consequences, action, conseq_space):
        # get consequences from user input
        continue_data_gen, abort, conseqs = UserInputConsequenceProcessing.get_consequences_from_user(red_team, red_team_conditions, red_team_consequences, action, conseq_space)
        if abort:
            return continue_data_gen, True, None

        # resolve conflicts
        continue_data_gen, abort, conseqs = UserInputConsequenceProcessing.resolve_consequence_conflict(red_team, red_team_conditions, red_team_consequences, action, conseq_space, conseqs)
        if abort:
            return continue_data_gen, True, None

        return continue_data_gen, False, conseqs

    @staticmethod
    def get_counter_factual_consequences_from_user(red_team, conditions, consequences, cf_action, conseq_space):
        # get consequences from user input
        continue_data_gen, abort, cf_conseqs = UserInputConsequenceProcessing.get_consequences_from_user(red_team, conditions, consequences, cf_action, conseq_space, cf=True)
        if abort:
            return continue_data_gen, True, None

        return continue_data_gen, False, cf_conseqs

    @staticmethod
    def resolve_consequence_conflict(red_team, red_team_conditions, red_team_consequences, action, conseq_space, conseqs):
        # create temporary policy data point
        temp_pol_point = RiskMitigatingPolicyDataPoint(conditions=red_team_conditions,
                                                       consequences_before_action=red_team_consequences,
                                                       action=action,
                                                       consequences_after_action=conseqs)

        # check for duplicates already in policy
        conflict, point_conseqs, pol_conseqs = temp_pol_point.check_and_get_conflicting_data_point_consequences(red_team.policy_data)
        if conflict:
            # report conflict
            RedTeamCommandLinePrinting.print_consequence_conflict_message(red_team_conditions, action, point_conseqs, pol_conseqs)

            # get consequences from user input
            continue_data_gen, abort, conseqs = UserInputConsequenceProcessing.get_consequences_from_user(red_team, red_team_conditions, red_team_consequences, action, conseq_space)
            if abort:
                return continue_data_gen, True, None

        return True, False, conseqs

    @staticmethod
    def get_consequences_from_user(red_team, red_team_conditions, red_team_consequences, action, conseq_space, cf=False):
        # initialize abort flag
        abort = False

        # get consequences after action input from user and unpack
        user_input = UserInputConsequenceProcessing.consequence_input_process(red_team, red_team_conditions, red_team_consequences, action, conseq_space, cf=cf)
        continue_data_gen, conseq_idxs = user_input

        # verify consequences are not None
        if conseq_idxs is None:
            # no valid consequences received, check whether skipping scenario or quitting data generation
            if continue_data_gen:
                # skipping scenario
                RedTeamCommandLinePrinting.print_skip_message()
                return True, True, None
            else:
                # quitting
                return False, True, None

        # get consequences
        conseqs = [conseq_space[i] for i in conseq_idxs]
        RedTeamCommandLinePrinting.print_got_consequences_message(conseqs)

        return continue_data_gen, abort, conseqs

    @staticmethod
    def consequence_input_process(red_team, red_team_conditions, red_team_consequences, action, conseq_space, cf=False):
        # initialize loop flag and consequence indices
        got_valid_consequences = False
        conseq_idxs = None

        # keep asking until valid input received
        while not got_valid_consequences:
            # print scenario
            RedTeamCommandLinePrinting.print_action_and_consequences(red_team_conditions, red_team_consequences, action, red_team, cf=cf)

            # print possible consequences
            RedTeamCommandLinePrinting.print_consequences(conseq_space)

            # ask for user input
            quit, skip, conseq_idxs = UserInputConsequenceProcessing.get_user_input_consequences(conseq_space)

            # check result
            if quit:
                return False, None
            if skip:
                return True, None
            if conseq_idxs is None:
                RedTeamCommandLinePrinting.print_try_again_message()
                continue

            # valid consequences received
            break

        # return consequence indices
        return True, conseq_idxs

    @staticmethod
    def get_user_input_consequences(conseq_space):
        # prompt user for input
        val = UserInputConsequenceProcessing.prompt_user_input_consequences()

        # check for quit
        if val == "exit()":
            return True, None, None

        # validate consequence indices
        skip, conseq_idxs = UserInputConsequenceProcessing.validate_consequence_indices(val, conseq_space)

        return False, skip, conseq_idxs

    @staticmethod
    def prompt_user_input_consequences():
        val = input("    Consequence numbers: ")
        RedTeamCommandLinePrinting.print_substep_separator()
        return val

    @staticmethod
    def validate_consequence_indices(val, conseq_space):
        # initialize value
        int_vals = None

        # try to convert comma separated string to list of ints
        try:
            int_vals = [int(i) for i in val.split(',')]
        except:
            print("    *** INVALID INPUT: " + val + " cannot be converted to a list of integers.")
            print()
            return None, None

        # check indices
        for int_val in int_vals:
            if (int_val < 0) or (int_val > len(conseq_space)+1):
                print("    *** INVALID INPUT: received " + str(int_val) + ", but must be in range [0," + str(len(conseq_space)+1) + "]")
                print()
                return None, None
            elif int_val == len(conseq_space):
                if len(int_vals) == 1:
                    # only selected no consequences, so no consequences
                    return False, []
                else:
                    print("    *** INVALID INPUT: received " + str(int_val) + " (no consequences) as part of list of " + str(len(int_vals)) + " options; if no consequences, only provide one number; otherwise, provide list of consequence numbers")
                    print()
                    return None, None
            elif int_val == (len(conseq_space)+1):
                if len(int_vals) == 1:
                    # only selected skip, so skip
                    return True, None
                else:
                    print("    *** INVALID INPUT: received " + str(int_val) + " (skip) as part of list of " + str(len(int_vals)) + " options; if skipping, only provide skip number; otherwise, provide list of consequence numbers")
                    print()
                    return None, None
            else:
                # valid int received
                continue

        # if we get here, all ints are valid
        return False, int_vals
