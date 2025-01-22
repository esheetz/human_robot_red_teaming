"""
Human-Robot Red Teaming Level 2
Enumeration of Possibilities
"""

from itertools import chain, combinations
from copy import deepcopy

"""
assumes model is properly formatted

creates list of (s,a,s') tuples of possibilities
"""
def hrrt2(model):
    # create all possible state combinations
    state_idxs = [i for i in range(len(model["states"]))]
    powerset_state_idxs = list(chain.from_iterable(
                            combinations(state_idxs,r)
                            for r in range(1,len(s)+1)))

    # generate states
    powerset_states = []
    for combo in powerset_state_idxs:
        states = [[]]
        for i in combo:
            if type(model["states"][i]) == str:
                # single state, append to states
                states = [s.append(i) for s in states]
            else: # type(model["states"][i]) == list(str)
                # copy states
                states = states*len(model["states"][i])
                # create new state for each mutex state
                states = [s.append(ms) for s in states for ms in model["states"][i]]
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
                post_sa = [i for i in post_sa if not in a["postcond_sub"]]
                # add to list of possibilities
                possibilities.append([s,a,post_sa])

    return possibilities
