# Safety Aware Reasoning Project

## TODOs

Accept user input about risky conditions and likelihood/consequence scores
- [x] method for inputting/formatting info about conditions, likelihood/consequence scores
- [x] error checking on inputting condition likelihood/consequence information for robot
- [x] computing risk=probability x impact and safety=1-risk
- [x] rename everything from safety conditions to risky conditions
- [x] method for inputting/formatting info about robot's action space of risk mitigating actions and/or task actions
- [x] error checking on inputting action space information for robot
- [x] formatting action space for later

Red teaming for data extension
- [ ] format data for initial condition and mitigating action combos
- [ ] randomly generate combinations of conditions
- [ ] ask user for input on which mitigating action to take, error checking on input/suggestion
- [ ] add to running training dataset

Model creation
- [ ] input risky conditions for real (not just tests for development purposes)
- [ ] input action spaces for real (not just tests for development purposes)
- [ ] logistic regression analysis (combined, CLR only, Val only, household only, lunar only, CLR/household, CLR/lunar, Val/household, Val/lunar)
- [ ] model training and validation
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
