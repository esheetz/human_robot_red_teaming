# Safety Aware Reasoning Project

## TODOs

Counter-factual red teaming
- [x] create general policy data point for less repeated coded
- [x] create counter-factual policy data point and reader classes; avoids collision changes needed for risk mitigating policy
- [x] rename red teamed data files to indicate both RS generation and CFA generation
- [x] NOTE that CFA will likely create situations with same (condition,pre-conseq) and different (action,post-conseq), since point of CFA is to understand effects; probably want list instead of dictionary
- [ ] add support to red team node to do both RS and CFA modes
- [ ] add counter-factual policy to red team policy
- [ ] randomly select risky scenario
- [ ] randomly select risk mitigating action
- [ ] ask user for input on state space
- [ ] error checking on user input
- [ ] check for multiples? may not be necessary
- [ ] write to YAML file

Model creation
- [ ] input risky conditions for real (not just tests for development purposes)
- [ ] input action spaces for real (not just tests for development purposes)
- [ ] input initial policy starters for real (not just tests for development purposes)
- [ ] generate red teamed data (~500 data points per robot per environment)
- [ ] logistic regression analysis (combined, CLR only, Val only, household only, lunar only, CLR/household, CLR/lunar, Val/household, Val/lunar)
- [ ] model(s) training and validation
- [ ] save models for later use

Safety score and risk score computations
- [ ] estimate safety/risk of current plan/task
- [ ] relevant for reporting

Online data point recording
- [ ] record data points online during task execution, store in separate dataset
- [ ] relearn from dataset and/or suggestions to relearn from dataset when new data is collected

Counterfactual reasoning for identifying upstream decision points
- [ ] report actions to user
- [ ] ask user for input on what action should have been taken
- [ ] generate additional data points based on user input
- [ ] add to online dataset
