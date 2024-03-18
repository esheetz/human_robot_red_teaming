# Safety Aware Reasoning Project

## TODOs

Accept user input about safety conditions and likelihood/consequence scores
- [ ] method for inputting/formatting info about conditions, likelihood/consequence scores
- [x] error checking on inputting condition likelihood/consequence information for robot
- [x] computing risk=probability x impact and safety=1-risk
- [ ] method for inputting/formatting info about robot's action space of risk mitigating actions and/or task actions
- [ ] error checking on inputting this information for robot
- [ ] formatting action space for later

Red teaming for data extension
- [ ] randomly generate combinations of conditions
- [ ] ask user for input on which mitigating action to take, error checking on input/suggestion
- [ ] add to running training dataset

Model creation
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
