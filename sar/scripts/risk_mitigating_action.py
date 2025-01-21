"""
Risk Mitigating Action Class
Emily Sheetz, NSTGRO VTE 2024
"""

class RiskMitigatingAction:
	def __init__(self, name="unnamed_action", autonomy_level=0.0):
		# set internal parameters
		self.name = str(name)
		self.autonomy_level = autonomy_level

	#######################
	### GETTERS/SETTERS ###
	#######################

	def get_action_name(self):
		return self.name

	def get_action_autonomy_level(self):
		return self.autonomy_level
