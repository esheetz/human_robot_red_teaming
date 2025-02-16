"""
Human-Robot Red Teaming Level 3
Challenging Assumption Validity
"""

import yaml

"""
assumes model is properly formatted

creates:
    - list of pre-condition assumptions (s,a)
    - list of (add/sub) post-condition assumptions (a,s')
"""
def hrrt3(model, interactive=False):
    # initialize lists of assumptions
    pre_assump = []
    post_add_assump = []
    post_sub_assump = []

    # generate assumptions
    for a in model["actions"]:
        for s in a["precond"]:
            pre_assump.append((a["name"],s))
        for s in a["postcond_add"]:
            post_add_assump.append((a["name"],s))
        for s in a["postcond_sub"]:
            post_sub_assump.append((a["name"],s))

    # query validity of assumptions
    pre_assump_validity = _query_assumption_validity(
                              assumptions_list=pre_assump,
                              state_prompt_text="pre-condition",
                              action_prompt_text="can be achieved to perform",
                              interactive=interactive
                          )
    post_add_assump_validity = _query_assumption_validity(
                                   assumptions_list=post_add_assump,
                                   state_prompt_text="added post-condition",
                                   action_prompt_text="will be achieved as a result of performing",
                                   interactive=interactive
                               )
    post_sub_assump_validity = _query_assumption_validity(
                                   assumptions_list=post_sub_assump,
                                   state_prompt_text="subtracted post-condition",
                                   action_prompt_text="will be undone as a result of performing",
                                   interactive=interactive
                               )

    return pre_assump_validity, post_add_assump_validity, post_sub_assump_validity

def write_hrrt3_yaml(assump_lists, model_file):
    # create file name
    ext_idx = model_file.rfind(".yaml")
    hrrt3_file_name = model_file[:ext_idx]+"_hrrt3"+model_file[ext_idx:]

    # unpack assumptions
    precond_assump, postcond_add_assump, postcond_sub_assump = assump_lists

    # format assumptions as YAML file
    yaml_dict = {}
    yaml_dict["assumptions"] = {}

    yaml_dict["assumptions"]["precond"] = _create_assumption_dicts(precond_assump)
    yaml_dict["assumptions"]["postcond_add"] = _create_assumption_dicts(postcond_add_assump)
    yaml_dict["assumptions"]["postcond_sub"] = _create_assumption_dicts(postcond_sub_assump)

    # open YAML file in write mode and dump
    fo = open(hrrt3_file_name, 'w')
    yaml.dump(yaml_dict, fo, default_flow_style=False, sort_keys=False)
    fo.close()
    print("Wrote data for HRRT Level 3 to file: " + hrrt3_file_name)

    return

def process_chatbot_hrrt3_text(assump_lists, file_name=None, raw_text=None):
    # unpack assumptions
    precond_assump, postcond_add_assump, postcond_sub_assump = assump_lists

    # initialize lines of text
    lines = None

    # check given options
    if file_name is None and raw_text is None:
        print("ERROR: given no text to process")
        return
    elif file_name is not None and raw_text is None:
        # open file and read text
        fo = open(file_name, 'r')
        lines = fo.readlines()
        fo.close()
    elif raw_text is not None:
        # split text around newlines
        lines = raw_text.split('\n')
    else:
        print("ERROR: invalid options, either file_name or raw_text must be given")
        return

    # parse text to identify assumptions and validity
    for line in lines:
        # check if states/actions found
        if '_' not in line:
            # no state/action found, continue
            continue
        # parse state and action
        tokens = line.split(' ')
        state_action_tokens = [t for t in tokens if '_' in t]
        # remove artificats
        state_action_tokens = [s.replace('**','') for s in state_action_tokens]
        # TODO STRONG ASSUMPTION: (s, a) ordering
        state, action = state_action_tokens

        valid = False
        # find validity
        ans_idx_Y = line.find('Y')
        ans_idx_N = line.find('N')
        if ans_idx_Y == -1 and ans_idx_N == -1:
            # assume invalid
            valid = False
        # check if valid
        if ans_idx_Y != -1:
            valid = True

        # set validity
        precond_assump_idx = [i for i,(a,s,v) in enumerate(precond_assump) if ((s == state) and (a == action))]
        postcond_add_assump_idx = [i for i,(a,s,v) in enumerate(postcond_add_assump) if ((s == state) and (a == action))]
        postcond_sub_assump_idx = [i for i,(a,s,v) in enumerate(postcond_sub_assump) if ((s == state) and (a == action))]
        # check if all are empty
        if len(precond_assump_idx) == 0 and \
            len(postcond_add_assump_idx) == 0 and \
            len(postcond_sub_assump_idx) == 0:
            print("ERROR: could not find in any assumption lists, parsed state {} and action {}".format(state, action))

        # process all matching (a,s) pre-condition pairs
        for idx in precond_assump_idx:
            # convert to list
            assump_list = list(precond_assump[idx])
            # set validity
            assump_list[2] = valid
            # replace in assumptions
            precond_assump[idx] = tuple(assump_list)
        # process all matching (a,s) post-condition add pairs
        for idx in postcond_add_assump_idx:
            # convert to list
            assump_list = list(postcond_add_assump[idx])
            # set validity
            assump_list[2] = valid
            # replace in assumptions
            postcond_add_assump[idx] = tuple(assump_list)
        # process all matching (a,s) post-condition subtract pairs
        for idx in postcond_sub_assump_idx:
            # convert to list
            assump_list = list(postcond_sub_assump[idx])
            # set validity
            assump_list[2] = valid
            # replace in assumptions
            postcond_sub_assump[idx] = tuple(assump_list)

    return precond_assump, postcond_add_assump, postcond_sub_assump



########################
### HELPER FUNCTIONS ###
########################

def _query_assumption_validity(assumptions_list,
                               state_prompt_text, action_prompt_text,
                               interactive=False):
    # initialize validity list
    validity_list = []

    # loop through assumptions
    for a,s in assumptions_list:
        # initialize assumption validity
        assump_validity = None
        # check interactive
        if not interactive:
            # just print out questions for ChatGPT
            print("Implicit assumption that " + state_prompt_text +
                    " state " + s + " " + action_prompt_text + " action " + a)
            print("    Is this assumption always valid? (Y/N) __________")
            print()
        else:
            # query user input for assumption until valid
            valid_input = False
            while not valid_input:
                print("Implicit assumption that " + state_prompt_text +
                        " state " + s + " " + action_prompt_text + " action " + a)
                val = input("    Is this assumption always valid? (Y/N) ")
                val = val.lower()
                if val not in ['y','n']:
                    print("Invalid input, please answer [Y/N] for yes or no.")
                else:
                    valid_input = True
                print()
            # received user input, store assumption validity
            assump_validity = (val == 'y')
        validity_list.append((a,s,assump_validity))

    return validity_list

def _create_assumption_dicts(assump_list):
    # initialize list of dictionaries
    assump_dicts = []

    # loop through assumptions
    for assump in assump_list:
        a,s,v = assump
        # create dict
        a_dict = {
            "action" : a,
            "assumed_state" : s,
            "validity" : v
        }
        # append to list
        assump_dicts.append(a_dict)

    return assump_dicts
