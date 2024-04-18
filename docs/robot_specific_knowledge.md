# Robot and Domain-Specific Knowledge

Depending on the embodiment and capabilities of a robot and depending on the particular domain, robots are expected to behave differently and take different risk mitigating actions under different circumstances.  This means that during the human-robot red teaming exercises, the human operators are expected to consider domain-specific and robot-specific knowledge to determine how the robot should act in different domains.

Robot-specific and/or domain-specific knowledge can be used to automatically generate data during [red teamed data generation](docs/red_team_data_generation.md).  This package includes a generic `DomainSpecificKnowledge` base class that can be inherited by derived classes.  Inherited classes must implement two abstract methods: `risky_scenario_callback` and `counter_factual_callback`.  These methods are service callbacks that aid in automatic red-team data generation.



## Knowledge Specific to Valkyrie and CLR

Currently, we support safety-aware reasoning for the Valkyrie and ChonkUR L. Rail-E robots.  Knowledge for these robots was learned during initial human-robot red teaming exercises and captured in the `ValCLRSpecificKnowledge` class.

### Automated Risky Scenario Data Generation

For Val and CLR, we assume that any risky situation can be mitigated by down-grading the robot's autonomy.  When presented with a random risky scenario with multiple risky conditions present, the condition that requires the lowest autonomy to resolve dominates and takes precedent.  In this way, we can automatically generate appropriate risk mitigating actions and post-action consequences for a red teamed risky scenario.

### Automated Counter-Factual Data Generation

For Val and CLR, we assume that any counter-factual situation can be mitigated by actions that require less autonomy than the factual situation.  If a counter-factual action requires equal or more autonomy than the factual action, it may not address all of the same consequence states as the factual action.  By looking at the human-generated policy starter data provided in the `config/` directory, we can approximate the effect of the counter-factual risk mitigating action to determine what (if any) of the present consequences are addressed by this counter-factual action.

### Launching the Knowledge Servers

To launch the Val and CLR specific servers, run:

```
roslaunch safety_aware_reasoning val_clr_specific_knowledge.launch
```

These services are automatically launched when [human-robot red teaming data generation exercises](docs/red_team_data_generation.md) are launched.
