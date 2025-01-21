"""
Domain-Specific Knowledge Class
Emily Sheetz, NSTGRO VTE 2024
"""

import rospy

from safety_aware_reasoning.srv import RiskyScenarioDataGeneration, RiskyScenarioDataGenerationRequest, RiskyScenarioDataGenerationResponse
from safety_aware_reasoning.srv import CounterFactualDataGeneration, CounterFactualDataGenerationRequest, CounterFactualDataGenerationResponse

############################################
### DOMAIN-SPECIFIC KNOWLEDGE BASE CLASS ###
############################################

class DomainSpecificKnowledge:
    def __init__(self, node_name=None, service_name_stub=None):
        # set node name
        if node_name is None:
            self.node_name = "Generic Domain-Specific Knowledge"
        else:
            self.node_name = node_name

        # set service names
        self.risky_scenario_service_name  = "knowledge_based_risky_scenario_data_gen"
        self.counter_factual_service_name = "knowledge_based_counter_factual_data_gen"
        if service_name_stub is not None:
            self.risky_scenario_service_name  = service_name_stub + "_" + self.risky_scenario_service_name
            self.counter_factual_service_name = service_name_stub + "_" + self.counter_factual_service_name
    
        # advertise services
        self.advertise_services()

    ##########################
    ### ADVERTISE SERVICES ###
    ##########################

    def advertise_services(self):
        self.risky_scenario_service = rospy.Service(self.risky_scenario_service_name,
                                                    RiskyScenarioDataGeneration,
                                                    self.risky_scenario_callback)
        self.counter_factual_service = rospy.Service(self.counter_factual_service_name,
                                                     CounterFactualDataGeneration,
                                                     self.counter_factual_callback)

        rospy.loginfo("[%s] Providing services for knowledge-based data generation!", self.node_name)

        return

    ############################################
    ### ABSTRACT METHODS : SERVICE CALLBACKS ###
    ############################################

    def risky_scenario_callback(self, req : RiskyScenarioDataGenerationRequest) -> RiskyScenarioDataGenerationResponse:
        raise NotImplementedError

    def counter_factual_callback(self, req : CounterFactualDataGenerationRequest) -> CounterFactualDataGenerationResponse:
        raise NotImplementedError
