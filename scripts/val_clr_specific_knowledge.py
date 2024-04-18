#!/usr/bin/env python3
"""
Domain-Specific Knowledge Class
Emily Sheetz, NSTGRO VTE 2024
"""

import rospy

import sys
from copy import deepcopy

# import base class
from domain_specific_knowledge import DomainSpecificKnowledge

# import state space, consequence space, action space, and policy data readers
from risky_condition_reader import RiskyConditionReader
from consequence_state_reader import ConsequenceStateReader
from risk_mitigating_action_reader import RiskMitigatingActionReader
from policy_data_point import PolicyDataPoint
from risk_mitigating_policy_data_reader import RiskMitigatingPolicyDataReader

from safety_aware_reasoning.srv import RiskyScenarioDataGeneration, RiskyScenarioDataGenerationRequest, RiskyScenarioDataGenerationResponse
from safety_aware_reasoning.srv import CounterFactualDataGeneration, CounterFactualDataGenerationRequest, CounterFactualDataGenerationResponse

#########################################################
### VAL / CLR DOMAIN-SPECIFIC KNOWLEDGE DERIVED CLASS ###
#########################################################

class ValCLRSpecificKnowledge(DomainSpecificKnowledge):
    def __init__(self, robot="val", environment="lunar_habitat"):
        # initialize super class
        super(ValCLRSpecificKnowledge, self).__init__(node_name="Val / CLR Specific Knowledge",
                                                      service_name_stub="val_clr")

        # set internal paramters
        self.robot_name = robot
        self.environment_name = environment

        # initialize readers
        self.state_space_reader = RiskyConditionReader(robot=self.robot_name,
                                                       environment=self.environment_name)
        self.consequence_state_space_reader = ConsequenceStateReader(robot=self.robot_name,
                                                                     environment=self.environment_name)
        self.action_space_reader = RiskMitigatingActionReader(robot=self.robot_name,
                                                              environment=self.environment_name)
        self.policy_starter_reader = RiskMitigatingPolicyDataReader(robot=self.robot_name,
                                                                    environment=self.environment_name,
                                                                    human_gen_data=True)

        # initialize spaces
        self.initialized = True
        self.__initialize_consequence_state_space()
        self.__initialize_state_space()
        self.__initialize_action_space()
        self.__initialize_policy_starter()



    #######################################
    ### INITIALIZATION HELPER FUNCTIONS ###
    #######################################

    def __initialize_state_space(self):
        # process state space
        self.state_space_reader.process_risky_conditions()
        if not self.state_space_reader.check_valid_conditions():
            rospy.logerr("[%s] Could not initialize state space", self.node_name)
            self.initialized = False
        else:
            rospy.loginfo("[%s] Successfully initialized state space!", self.node_name)
        return

    def __initialize_consequence_state_space(self):
        # process consequence state space
        self.consequence_state_space_reader.process_consequence_states()
        if not self.consequence_state_space_reader.check_valid_states():
            rospy.logerr("[%s] Could not initialize consequence state space", self.node_name)
            self.initialized = False
        else:
            rospy.loginfo("[%s] Successfully initialized consequence state space!", self.node_name)
        return

    def __initialize_action_space(self):
        # process action space
        self.action_space_reader.process_risk_mitigating_actions()
        if not self.action_space_reader.check_valid_actions():
            rospy.logerr("[%s] Could not initialize action space", self.node_name)
            self.initialized = False
        else:
            rospy.loginfo("[%s] Successfully initialized action space!", self.node_name)
        return

    def __initialize_policy_starter(self):
        # process policy data
        self.policy_starter_reader.process_risk_mitigating_policy_data()
        if not self.policy_starter_reader.check_valid_policy():
            rospy.logerr("[%s] Could not initialize human-generated policy starter data points", self.node_name)
            self.initialized = False
        else:
            rospy.loginfo("[%s] Successfully initialized human-generated policy data points!", self.node_name)
        return



    #########################
    ### SERVICE CALLBACKS ###
    #########################

    def risky_scenario_callback(self, req : RiskyScenarioDataGenerationRequest) -> RiskyScenarioDataGenerationResponse:
        # initialize response
        res = RiskyScenarioDataGenerationResponse()

        # attempt to get actions and consequences from given scenario
        output = self.get_knowledge_based_risky_scenario_output(req.condition_names,
                                                                req.pre_action_consequence_names)
        # unpack
        succ, action, conseq = output

        # set response
        if succ:
            res.success = True
            res.action_name = action
            res.post_action_consequence_names = conseq
        else:
            res.success = False
            res.action_name = ""
            res.post_action_consequence_names = []

        # return result
        return res

    def counter_factual_callback(self, req : CounterFactualDataGenerationRequest) -> CounterFactualDataGenerationResponse:
        # initialize response
        res = CounterFactualDataGenerationResponse()

        # attempt to get consequences from given counter-factual scenario
        output = self.get_knowledge_based_counter_factual_output(req.condition_names,
                                                                 req.pre_action_consequence_names,
                                                                 req.factual_action_name,
                                                                 req.factual_post_action_consequence_names,
                                                                 req.counter_factual_action_name)
        # unpack
        succ, msg, conseq = output

        # set response
        if succ:
            rospy.loginfo("[%s] Generated counter-factual data point: %s", self.node_name, msg)
            res.success = True
            res.post_action_consequence_names = conseq
        else:
            res.success = False
            res.post_action_consequence_names = []

        # return result
        return res



    #######################
    ### SERVICE HELPERS ###
    #######################

    def get_knowledge_based_risky_scenario_output(self, condition_names, pre_action_conseq_names):
        # get action space for given risky scenario
        action_names = self.get_risky_scenario_action_space(condition_names, pre_action_conseq_names)

        # get action(s) with lowest autonomy level
        action = self.get_lowest_autonomy_level_action(action_names)

        # check success
        if action is None:
            return False, None, None

        # get consequences for chosen action
        post_action_conseq_names = self.get_consequences_after_action_from_policy(action)

        # check success
        if post_action_conseq_names is None:
            return False, None, None

        # successfully generated knowledge-based output! return
        return True, action, post_action_conseq_names

    def get_knowledge_based_counter_factual_output(self, condition_names, pre_action_conseq_names, f_action_name, f_post_action_conseq_names, cf_action_name):
        # get factual and counter-factual actions
        f_action = self.action_space_reader.get_risk_mitigating_action_with_name(f_action_name)
        cf_action = self.action_space_reader.get_risk_mitigating_action_with_name(cf_action_name)

        # get autonomy levels
        f_action_auto_level = f_action.get_action_autonomy_level()
        cf_action_auto_level = cf_action.get_action_autonomy_level()

        # initialize counter-factual post-action consequences and message
        cf_post_action_conseq_names = []
        msg = ""

        # compare autonomy levels
        if cf_action_auto_level == f_action_auto_level:
            # counter-factual action requires equal autonomy; just as safe as factual action
            msg = "counter-factual action ({}) requires equal autonomy as factual action ({}), and is just as safe!".format(cf_action_name, f_action_name)

            # assume counter-factual action has same consequences as factual action
            cf_post_action_conseq_names = deepcopy(f_post_action_conseq_names)

        elif cf_action_auto_level < f_action_auto_level:
            # counter-factual action requires less autonomy; may be even safer than factual action
            msg = "counter-factual action ({}) requires less autonomy than factual action ({}), and may even be safer!".format(cf_action_name, f_action_name)

            # get effect of counter-factual action (consequences resolved by action)
            cf_action_effects = self.get_consequences_resolved_by_action_from_policy(cf_action_name)

            # check success
            if cf_action_effects is None:
                return False, None, None

            # subtract counter-factual resolved consequences from factual resolved consequences, since we are even safer than factual action
            cf_post_action_conseq_names = [conseq for conseq in f_post_action_conseq_names if conseq not in cf_action_effects]
            # equivalent to set difference:
            #     cf_post_action_conseq_names = f_post_action_conseq_names - cf_action_effects

        else: # cf_action_auto_level > f_action_auto_level
            # counter-factual action requires more autonomy; will not adequately address all safety conditions
            msg = "counter-factual action ({}) requires more autonomy than factual action ({}), which will not be as safe.".format(cf_action_name, f_action_name)

            # get effect of counter-factual action (consequences resolved by action)
            cf_action_effects = self.get_consequences_resolved_by_action_from_policy(cf_action_name)

            # check success
            if cf_action_effects is None:
                return False, None, None

            # subtract counter-factual resolved consequences from initial condition consequences, since we will not necessarily address any factually resolved consequences
            cf_post_action_conseq_names = [conseq for conseq in pre_action_conseq_names if conseq not in cf_action_effects]
            # equivalent to set difference:
            #     cf_post_action_conseq_names = pre_action_conseq_names - cf_action_effects

        return True, msg, cf_post_action_conseq_names



    ##############################
    ### RISKY SCENARIO HELPERS ###
    ##############################

    def get_risky_scenario_action_space(self, condition_names, pre_action_conseq_names):
        # initialize action space
        action_space_set = set()

        # loop through condition names
        for cond_name in condition_names:
            # get condition from state space
            cond = self.state_space_reader.get_risky_condition_with_name(cond_name)

            # get condition consequences
            consequences = cond.get_consequence_states()

            # create temporary policy data point
            temp_pol_point = PolicyDataPoint(conditions=[cond_name],
                                             consequences_before_action=consequences)

            # get key to find condition in policy
            temp_key = temp_pol_point.get_policy_data_point_dictionary_key()

            # get action for single single condition
            pol_point = self.policy_starter_reader.risk_mitigating_policy[temp_key]

            # get action
            act = pol_point.get_policy_data_point_action_name()

            # add to set
            action_space_set.add(act)

        return list(action_space_set)

    def get_lowest_autonomy_level_action(self, action_names):
        # initialize lowest autonomy levels
        lowest_action = []
        lowest_autonomy_level = float('inf')

        # loop through actions
        for act_name in action_names:
            # get action from action space
            act = self.action_space_reader.get_risk_mitigating_action_with_name(act_name)

            # get action autonomy level
            autonomy_level = act.get_action_autonomy_level()

            # check autonomy level
            if autonomy_level < lowest_autonomy_level:
                # new lowest found, overwrite previous values
                lowest_action = [act_name]
                lowest_autonomy_level = autonomy_level
            elif autonomy_level == lowest_autonomy_level:
                # add action to running list
                lowest_action.append(act_name)
            else:
                # higher autonomy level, disregard
                continue

        # verify only one action found
        if len(lowest_action) != 1:
            rospy.logwarn("[%s] Found multiple actions to address risky scenario; cannot determine unique knowledge-based action", self.node_name)
            return None

        # return single action
        return lowest_action[0]

    def get_consequences_after_action_from_policy(self, action_name):
        # initialize consequences
        consequences = []

        # loop through policy data
        for temp_key in self.policy_starter_reader.risk_mitigating_policy.keys():
            # get policy data point
            temp_pol_point = self.policy_starter_reader.risk_mitigating_policy[temp_key]

            # check if action matches
            if action_name == temp_pol_point.get_policy_data_point_action_name():
                # add to list of consequences
                consequences.append(temp_pol_point.get_policy_data_point_consequences_after_action_names())

        # verify only one set of consequences found
        if len(consequences) != 1:
            rospy.logwarn("[%s] Found multiple sets of possible consequences after taking action %s; cannot determine unique knowledge-based set of consequences", self.node_name, action_name)
            return None

        # return single set of consequences
        return consequences[0]

    ###############################
    ### COUNTER-FACTUAL HELPERS ###
    ###############################

    def get_consequences_resolved_by_action_from_policy(self, action_name):
        # initialize consequences
        consequences = []

        # loop through policy data
        for temp_key in self.policy_starter_reader.risk_mitigating_policy.keys():
            # get policy data point
            temp_pol_point = self.policy_starter_reader.risk_mitigating_policy[temp_key]

            # check if action matches
            if action_name == temp_pol_point.get_policy_data_point_action_name():
                # get before and after consequences
                bef_conseq, aft_conseq = temp_pol_point.get_policy_data_point_consequence_names()

                # get consequences resolved by action
                conseqs = [conseq for conseq in bef_conseq if conseq not in aft_conseq]
                # equivalent to set difference:
                #     conseqs = bef_conseq - aft_conseq

                # add to list of consequences
                consequences.append(conseqs)

        # verify only one set of consequences found
        if len(consequences) != 1:
            rospy.logwarn("[%s] Found multiple sets of possible consequences resolved by action %s; cannot determine unique knowledge-based set of consequences", self.node_name, action_name)
            return None

        # return single set of resolved consequences
        return consequences[0]



#####################
### MAIN FUNCTION ###
#####################

if __name__ == '__main__':
    # set node name
    node_name = "ValCLRSpecificKnowledgeServerNode"
    param_prefix = "/" + node_name + "/"

    # get ROS parameters
    robot_name = rospy.get_param(param_prefix + 'robot', "val")
    env_name = rospy.get_param(param_prefix + 'environment', "lunar_habitat")

    # initialize node
    rospy.init_node(node_name)

    # create server node
    server_node = ValCLRSpecificKnowledge(robot=robot_name, environment=env_name)

    if not server_node.initialized:
        rospy.logerr("[%s] Could not initialize domain-specific knowledge server node", server_node.node_name)
        # exit with error
        sys.exit(1)
    else:
        rospy.loginfo("[%s] Successfully initialized domain-specific knowledge server node for robot %s in %s environment!",
                      server_node.node_name,
                      server_node.robot_name.upper(), server_node.environment_name.upper())

    # run node, wait for requests
    while not rospy.is_shutdown():
        rospy.spin()

    rospy.loginfo("[%s] Node stopped, all done!", server_node.node_name)
    # exit with success
    sys.exit(0)
