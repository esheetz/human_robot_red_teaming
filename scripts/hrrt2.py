"""
Human-Robot Red Teaming Level 2
Enumeration of Possibilities
"""

from itertools import chain, combinations
from copy import deepcopy
import yaml

"""
assumes model is properly formatted

creates list of (s,a,s') tuples of possibilities
"""
def hrrt2(model, interactive=False):
    # create all possible state combinations
    state_idxs = [i for i in range(len(model["states"]))]
    powerset_state_idxs = list(chain.from_iterable(
                            combinations(state_idxs,r)
                            for r in range(1,len(state_idxs)+1)))

    # generate states
    powerset_states = []
    for combo in powerset_state_idxs:
        states = [[]]
        for i in combo:
            if type(model["states"][i]) == str:
                # single state, append to states
                states = [s+[model["states"][i]] for s in states]
            else: # type(model["states"][i]) == list(str)
                # create new state for each mutex state
                states = [s+[ms] for s in states for ms in model["states"][i]]
        # add state to powerset
        powerset_states += states

    # create all possible (s,a,s') combos
    possibilities = []
    for s in powerset_states:
        # consider all possible actions
        for a in model["actions"]:
            # check if preconditions satisfied
            if set(a["precond"]).issubset(set(s)):
                # action is valid in state
                post_sa = deepcopy(s)
                # add/subtract post-conditions
                post_sa += a["postcond_add"]
                post_sa = [i for i in post_sa if i not in a["postcond_sub"]]
                # add to list of possibilities
                possibilities.append([s,a["name"],post_sa])

    # query validity of possibilities
    possibility_validity = _query_possibility_validity(possibilities, interactive)

    return possibility_validity

def hrrt2_abridged_transcript(possibilities):
    # create set of states
    possibility_set = set()

    # create set of only unique states
    for s,_,_,_ in possibilities:
        possibility_set.add(tuple(s))

    # print transcript for unique possibilities
    for s in possibility_set:
        # just print out questions for ChatGPT
        print("Found possible state:", list(s))
        print("    Is this possibility always valid (safe, feasible, etc.)? (Y/N) __________")
        print()

    print("Please format your answers as a YAML file, with a list of states and their validity.")

    return

def write_hrrt2_yaml(possibilities, model_file):
    # create file name
    ext_idx = model_file.rfind(".yaml")
    hrrt2_file_name = model_file[:ext_idx]+"_hrrt2"+model_file[ext_idx:]

    # format possibilities as YAML dictionary
    yaml_dict = {}
    yaml_dict["possibilities"] = []

    for p in possibilities:
        s,a,sp,v = p
        # create dict
        p_dict = {
            "state" : s,
            "action" : a,
            "next_state" : sp,
            "validity" : v
        }
        # append to list
        yaml_dict["possibilities"].append(p_dict)

    # open YAML file in write mode and dump
    fo = open(hrrt2_file_name, 'w')
    yaml.dump(yaml_dict, fo, default_flow_style=False, sort_keys=False)
    fo.close()
    print("Wrote data for HRRT Level 2 to file: " + hrrt2_file_name)

    return

def process_chatbot_hrrt2_text(possibilities, file_name=None, raw_text=None):
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

    # parse text to identify states and validity
    for line in lines:
        # find state
        state_idx_start = line.find("[")
        state_idx_end = line.find("]")
        if state_idx_start == -1 or state_idx_end == -1:
            # no state found, continue
            continue
        # parse state, ignore square brackets
        state_txt = line[state_idx_start+1:state_idx_end]
        # remove any quotes
        state_txt = state_txt.replace('"','')
        state_txt = state_txt.replace("'","")
        # get state as list of literals
        state = state_txt.split(', ')

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
        state_poss_idx = [i for i,(s,a,sp,v) in enumerate(possibilities) if s == state]
        if len(state_poss_idx) == 0:
            print("ERROR: could not find in possibilities list, parsed state", state)
        # process all matching states
        for idx in state_poss_idx:
            # convert to list
            poss_list = list(possibilities[idx])
            # set validity
            poss_list[3] = valid
            # replace in possibilities
            possibilities[idx] = tuple(poss_list)

    return possibilities

def process_chatbot_hrrt2_yaml(file_name=None, raw_yaml=None):
    # initialize yaml dict
    yaml_dict = None

    # check given options
    if file_name is None and raw_yaml is None:
        print("ERROR: given no text to process")
        return
    elif file_name is not None and raw_yaml is None:
        # open file and read text
        fo = open(file_name, 'r')
        yaml_dict = yaml.load(fo, Loader=yaml.FullLoader)
        fo.close()
    elif raw_yaml is not None:
        # read raw text as yaml
        yaml_dict = yaml.load(raw_text, Loader=yaml.FullLoader)
    else:
        print("ERROR: invalid options, either file_name or raw_text must be given")
        return

    # initialize list of possibilities
    possibilities = []

    # loop through yaml
    for s_dict in yaml_dict["states"]:
        print("NOT IMPLEMENTED")

    return



########################
### HELPER FUNCTIONS ###
########################

def _query_possibility_validity(possibility_list, interactive=False):
    # initialize validity list
    validity_list = []

    # loop through possibilities
    for s,a,sp in possibility_list:
        # initialize possibility validity
        poss_validity = None
        # check interactive
        if not interactive:
            # just print out questions for ChatGPT
            print("Found possible state:", s)
            print("    within which it is possible to take action " + a)
            print("    Is this possibility always valid (safe, feasible, etc.)? (Y/N) __________")
            print()
        else:
            # query user input for possibility until valid
            valid_input = False
            while not valid_input:
                print("Found possible state:", s)
                print("    within which it is possible to take action " + a)
                val = input("    Is this possibility always valid (safe, feasible, etc.)? (Y/N) ")
                val = val.lower()
                if val not in ['y','n']:
                    print("Invalid input, please answer [Y/N] for yes or no.")
                else:
                    valid_input = True
                print()
            # received user input; store possibility validity
            poss_validity = (val == 'y')
        validity_list.append((s,a,sp,poss_validity))

    return validity_list
