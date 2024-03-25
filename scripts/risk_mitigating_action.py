"""
Risk Mitigating Action Class
Emily Sheetz, NSTGRO VTE 2024
"""

class RiskMitigatingAction:
	def __init__(self, name="unnamed_action"):
		# set internal parameters
		self.name = str(name)

	#######################
	### GETTERS/SETTERS ###
	#######################

	def get_action_name(self):
		return self.name
