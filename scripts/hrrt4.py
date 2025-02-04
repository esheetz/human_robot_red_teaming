"""
Human-Robot Red Teaming Level 4
Update Model Knowledge
"""

import yaml

"""
assumes knowledge base, model, possibilities, and assumptions are properly formatted

creates updated model hypothesis (S+,S-,A+,A-) and facts L
"""
def hrrt4(kb_file_name, kb,
          model_file_name, model,
          possibilities, assumptions):
    # unpack assumptions
    precond_assump, postcond_add_assump, postcond_sub_assump = assumptions
    # get invalid possibilities and assumptions
    # possibilitity is of form (s,a,s',v)
    invalid_poss = [p for p in possibilities if not p[3]]
    # assumption is of form (a,s,v)
    invalid_precond_assump = [a for a in precond_assump if not a[2]]
    invalid_postcond_add_assump = [a for a in postcond_add_assump if not a[2]]
    invalid_postcond_sub_assump = [a for a in postcond_sub_assump if not a[2]]
    invalid_assumps = (invalid_precond_assump, invalid_postcond_add_assump, invalid_postcond_sub_assump)

    # query updates
    model_hypothesis_poss = _query_invalid_possibilities(invalid_poss)
    model_hypothesis_assump = _query_invalid_assumptions(invalid_assumps)

    # ask probing questions
    facts, model_hypothesis_probe = self._query_probing_questions()

    # union model hypothesis
    # each model hypothesis is a tuple of 4:
    #   [0]: add states
    #   [1]: sub states
    #   [2]: add actions
    #   [3]: sub actions
    states_add = model_hypothesis_poss[0] + model_hypothesis_assump[0] + model_hypothesis_probe[0]
    states_sub = model_hypothesis_poss[1] + model_hypothesis_assump[1] + model_hypothesis_probe[1]
    actions_add = model_hypothesis_poss[2] + model_hypothesis_assump[2] + model_hypothesis_probe[2]
    actions_sub = model_hypothesis_poss[3] + model_hypothesis_assump[3] + model_hypothesis_probe[3]
    model_hypothesis = (states_add, states_sub, actions_add, actions_sub)

    # add states to model
    if len(states_add) == 0:
        print("No new states to add to model! Suggests closed-world assumption for model is valid!")
    else:
        model["states"] += states_add
        print("Added states to model!")
    # remove states from model and add to knowledge base
    if len(states_sub) == 0:
        print("No states to remove from model! Suggests model may appropriately cover state space!")
    else:
        model["states"] = [s for s in model["states"] if s not in states_sub]
        kb["avoid_states"] += states_sub
        print("Removed states from model and added them to knowledge base!")
    # add actions to model
    if len(actions_add) == 0:
        print("No new actions to add to model! Suggests closed-world assumption for model is valid!")
    else:
        model["actions"] += actions_add
        print("Added actions to model!")
    # remove actions from model
    if len(actions_sub) == 0:
        print("No actions to remove from model! Suggests model may appropriately cover action space!")
    else:
        model["actions"] = [a for a in model["actions"] if a["name"] not in actions_sub]
        kb["avoid_actions"] += actions_sub
        print("Removed actions from model and added them to knowledge base!")
    # add facts to knowledge base
    if len(facts) == 0:
        print("No new information to add to knowledge base! Suggests modeled knowledge minimizes 'unknown unknowns'!")
    else:
        kb["facts"] += facts
        print("Added new information to knowledge base!")
    # reset confidence score for updated model
    model["confidence_score"] = {
        "successes" : 0,
        "attempts" : 0
    }

    return kb, model, model_hypothesis, facts

def write_hrrt4_yaml(updated_kb, updated_model,
                     model_hypothesis, facts,
                     kb_file, model_file):
    # create file names
    ext_idx = model_file.rfind(".yaml")
    hrrt4_file_name = model_file[:ext_idx]+"_hrrt4"+model_file[ext_idx:]
    file_num = int(model_file[ext_idx-1]) + 1
    updated_model_file_name = model_file[:ext_idx-1]+str(file_num)+model_file[ext_idx:]

    # format model hypothesis as YAML dictionary
    yaml_dict = {}
    yaml_dict["model_hypothesis"] = {
        "states_add" : model_hypothesis[0],
        "states_sub" : model_hypothesis[1],
        "actions_add" : model_hypothesis[2],
        "actions_sub" : model_hypothesis[3],
        "facts" : facts
    }

    # open YAML file in write mode and dump
    fo = open(hrrt4_file_name, 'w')
    yaml.dump(yaml_dict, fo, default_flow_style=False, sort_keys=False)
    fo.close()
    print("Wrote data for HRRT Level 4 to file: " + hrrt4_file_name)

    # format updated model as YAML dictionary
    model_yaml_dict = {}
    model_yaml_dict["model"] = updated_model

    # open new model file in write mode and dump
    fo = open(updated_model_file_name, 'w')
    yaml.dump(model_yaml_dict, fo, default_flow_style=False, sort_keys=False)
    fo.close()
    print("Wrote updated model to file: " + updated_model_file_name)

    # format updated knowledge base as YAML dictionary
    kb_yaml_dict = {}
    kb_yaml_dict["kb"] = updated_kb

    # open kb file in write mode and dump
    fo = open(kb_file, 'w') # 'w' overwrites knowledge base
    yaml.dump(kb_yaml_dict, fo, default_flow_style=False, sort_keys=False)
    fo.close()
    print("Updated knowledge base in file: " + kb_file)

    return

########################
### HELPER FUNCTIONS ###
########################

def _get_model_hypothesis():
    # initialize lists
    states_add = []
    states_sub = []
    actions_add = []
    actions_sub = []

    ### ADD STATES ###
    print("What states should be added to the model?")
    print("    Recall symbolic states are literals or lists of mutually exclusive (mutex) literals.")
    print("    Please enter states to add to model (or Q to quit)")
    quit = False
    while not quit:
        state_input = input("add state (or list of comma separated mutex states): ")
        state_input = state_input.lower()
        if state_input == 'q':
            quit = True
            continue
        elif ',' in state_input:
            # split over commas
            states_add += state_input.split(',')
        else:
            states_add.append(state_input)
        print()
    print()

    ### REMOVE STATES ###
    print("What states should be removed from the model?")
    print("    Recall symbolic states are literals or lists of mutually exclusive (mutex) literals.")
    print("    Please enter states to remove from model (or Q to quit)")
    quit = False
    while not quit:
        state_input = input("remove state (or list of comma separated mutex states): ")
        state_input = state_input.lower()
        if state_input == 'q':
            quit = True
            continue
        elif ',' in state_input:
            # split over commas
            states_sub += state_input.split(',')
        else:
            states_sub.append(state_input)
        print()
    print()

    ### ADD ACTIONS ###
    print("What actions should be added to the model?")
    print("    Recall actions are of the form (name, preconditions, postconditions[add/subtract])")
    print("    Please enter actions to add to model (or Q to quit)")
    quit = False
    while not quit:
        print("add action:")
        a = {}
        action_input = input("    name: ")
        action_input = action_input.lower()
        if action_input == 'q':
            quit = True
            continue
        else:
            a["name"] = action_input.lower()
        action_input = input("    pre-conditions (list of comma separated states): ")
        action_input = action_input.lower()
        if action_input == 'q':
            quit = True
            continue
        elif ',' in action_input:
            # split over commas
            a["precond"] = action_input.split(',')
        else:
            a["precond"] = [action_input]
        action_input = input("    added post-conditions (list of comma separated states): ")
        action_input = action_input.lower()
        if action_input == 'q':
            quit = True
            continue
        elif ',' in action_input:
            # split over commas
            a["postcond_add"] = action_input.split(',')
        else:
            a["postcond_add"] = [action_input]
        action_input = input("    removed post-conditions (list of comma separated states): ")
        action_input = action_input.lower()
        if action_input == 'q':
            quit = True
            continue
        elif ',' in action_input:
            # split over commas
            a["postcond_sub"] = action_input.split(',')
        else:
            a["postcond_sub"] = [action_input]
        # add action to list
        actions_add.append(a)
    print()

    ### REMOVE ACTIONS ###
    print("What actions should be removed from the model?")
    print("    Please enter actions to remove from model (or Q to quit)")
    quit = False
    while not quit:
        action_input = input("remove action: ")
        action_input = action_input.lower()
        if action_input == 'q':
            quit = True
            continue
        else:
            actions_sub.append(action_input)
        print()
    print()

    # return model hypothesis
    return states_add, states_sub, actions_add, actions_sub

def _query_invalid_possibilities(invalid_possibilities):
    # check for empty possibilities
    if len(invalid_possibilities) == 0:
        print("No invalid possibilities found!")
        print("    Suggests model may not contain invalid possibilities!")
        return ([],[],[],[])
    else:
        # print invalid possibilities
        print("The following possibilities were identified as invalid on Level HRRT2:")
        for s,a,sp,v in invalid_possibilities:
            print("    possible state:", s)
            print("        within which it is possible to take action " + a)
            print()
        print("Let's update the model to address these invalid possibilities!")
        print()
        model_hypothesis = _get_model_hypothesis()
        return model_hypothesis

def _query_invalid_assumptions(invalid_assumptions):
    # unpack assumptions
    invalid_precond_assump, invalid_postcond_add_assump, invalid_postcond_sub_assump = invalid_assumptions
    if len(invalid_precond_assump) == 0 and \
        len(invalid_postcond_add_assump) == 0 and \
        len(invalid_postcond_sub_assump) == 0:
        print("No invalid assumptions found!")
        print("    Suggests model may not contain invalid assumptions!")
        return ([],[],[],[])
    else:
        # print invalid assumptions
        print("The following assumptions were identified as invalid on Level HRRT3:")
        for a,s,v in invalid_precond_assump:
            print("    pre-condition state " + s + " can be achieved to perform action " + a)
            print()
        for a,s,v in invalid_postcond_add_assump:
            print("    added post-condition state " + s + " will be achieved as a result of performing action " + a)
            print()
        for a,s,v in invalid_postcond_sub_assump:
            print("    subtracted post-condition state " + s + " will be undone as a result of performing action " + a)
            print()
        print("Let's update the model to address these invalid assumptions!")
        print()
        model_hypothesis = _get_model_hypothesis()
        return model_hypothesis

def _query_probing_questions():
    # initialize list
    facts = []

    # list of probing questions
    questions = [
        "What important information may still be missing from the model?",
        "What should an agent know when completing tasks in this domain?",
        "What catastrophic failures could occur in this domain?",
        "Are there external, independently verified resources for identifying failure cases in this domain?",
        "What are undesirable outcomes (not necessarily catastrophic failures) for this domain?",
        "How much can an agent trust other agents in this domain?",
        "What are unlikely or remote possibilities that may occur in this domain?"
    ]

    for q in questions:
        ### ASK QUESTION ###
        print(q)
        print("    Blue Team, please enter thoughts for this question (or Q to quit)")
        ### BLUE TEAM INPUT ###
        quit = False
        while not quit:
            info = input("[HUMAN-ROBOT BLUE TEAM RESPONSE]: ")
            info = info.lower()
            if info == 'q':
                quit = True
                continue
            else:
                facts.append(info)
            print()
        print()
        print("    Red Team, please enter thoughts for this question (or Q to quit)")
        ### RED TEAM INPUT ###
        quit = False
        while not quit:
            info = input("[HUMAN-ROBOT RED TEAM RESPONSE]: ")
            info = info.lower()
            if info == 'q':
                quit = True
                continue
            else:
                facts.append(info)
            print()
        print()
        print("============================================================")
        print()

    # check for final model update
    valid_input = False
    while not valid_input:
        val = input("Having considered these questions, would the team like to provide additional model updates? (Y/N) ")
        val = val.lower()
        if val not in ['y','n']:
            print("Invalid input, please answer [Y/N] for yes or no.")
        else:
            valid_input = True
        print()
    # optional model update
    model_hypothesis = ([],[],[],[])
    if val == 'y':
        print("Let's update the model to address the team's reflections!")
        print()
        model_hypothesis = _get_model_hypothesis()

    return facts, model_hypothesis
