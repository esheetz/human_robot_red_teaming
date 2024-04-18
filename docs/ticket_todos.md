# Safety Aware Reasoning Project

## TODOs

Documentation
- [x] ticket TODO page
- [x] main README
- [x] reader page
- [x] specific knowledge
- [ ] red team data generation

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
- [ ] final clean of all red-teamed data to start fresh
- [ ] add instructions to README about how to generate data and run nodes
- [ ] generate red teamed data (~500 data points per robot per environment) (depending on how long this takes, may put off CLR for later) (but right now CLR data is exactly the same, nothing is specific to Val)
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
