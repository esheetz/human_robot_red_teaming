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
def hrrt2(model):
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
    possibility_validity = _query_possibility_validity(possibilities)

    return possibility_validity

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

########################
### HELPER FUNCTIONS ###
########################

def _query_possibility_validity(possibility_list):
    # initialize validity list
    validity_list = []

    # loop through possibilities
    for s,a,sp in possibilitiy_list:
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
        # received user input; store possibility validity
        poss_validity = (val == 'y')
        validity_list.append((s,a,sp,poss_validity))

    return validity_list
