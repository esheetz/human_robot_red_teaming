"""
Consequence State Class
Emily Sheetz, NSTGRO VTE 2024
"""

class ConsequenceState:
    def __init__(self, name="unnamed_consequence"):
        # set internal parameters
        self.name = str(name)

    #######################
    ### GETTERS/SETTERS ###
    #######################

    def get_consequence_name(self):
        return self.name
