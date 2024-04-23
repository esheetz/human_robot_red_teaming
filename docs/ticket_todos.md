# Safety Aware Reasoning Project

## TODOs

Model creation
- [x] input consequence states for real (not just tests for development purposes)
- [x] validate input of consequence states against data readers to verify formatting
- [x] input risky conditions for real (not just tests for development purposes)
- [x] validate input of risky conditions against data readers to verify formatting
- [x] input action spaces for real (not just tests for development purposes)
- [x] validate input of action spaces against data readers to verify formatting
- [x] input initial policy starters for real (not just tests for development purposes)
- [x] validate input of initial policy starters against data readers to verify formatting
- [x] validate red team initialization to verify formatting
- [x] final testing of data generation; minor debugging
- [x] prevent CF red teaming from generating repeated data points
- [x] right now Val and CLR are the same, so create combined "val_clr" robot to simplify data generation
- [x] update docs with combined val_clr robot
- [x] final clean of generated data to start fresh
- [x] generate random risky scenario data (63 datapoints per environment, 126 total datapoints for combined Val/CLR robot in combined environments)
- [x] generate counter-factual data (252 datapoints for lunar, 315 datapoints for household, 567 total datapoints for combined Val/CLR robot in combined environments)
- [ ] augment data with dummy variables and mean/max risk/safety scores; save formatted data as CSVs to save time on further data processing
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
