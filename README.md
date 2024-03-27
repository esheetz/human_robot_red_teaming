# Safety Aware Reasoning Project

## TODOs

Red teaming for data extension
- [x] format data for initial condition and mitigating action combos
- [x] read in human-generated data
- [x] refactor into risk mitigating policy data point class for checking duplicates?
- [x] add risky condition and risk action helper for checking if something exists in list
- [x] initialize red team node with state/action spaces and initial policy
- [x] randomly generate combinations of conditions
- [x] ask user for input on which mitigating action to take, error checking on input/suggestion
- [x] check for multiples in generated conditions and in user input for mitigating action
- [x] write to YAML file
- [x] add to running training dataset

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
