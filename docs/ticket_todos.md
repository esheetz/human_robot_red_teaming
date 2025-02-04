# Safety Aware Reasoning Project

## TODOs

HRRT Level 2: Enumeration
- [x] check formatting on model and knowledge base
- [x] read in model (states and actions) and knowledge base
- [x] enumerate possibilities from model
- [x] dialogue tree prompts about possibilities
- [x] write possibilities (and validity) to YAML file

HRRT Level 3: Assumptions
- [x] assumptions in actions:
	- [x] pre-condition assumptions ({s},a)
	- [x] post-condition assumptions (a, {s})
	- [x] alternate contingency plans?
- [x] dialogue tree prompts about assumptions
- [x] save assumptions (and validity) to YAML file

HRRT Level 4: Updating model knowledge
- [x] probing questions about possibilities and assumptions
- [x] accept user input for states and actions (prompt each sub-component of actions)
- [x] suggest when no new information has been gained
- [x] update model with added states
- [x] update model with removed states (but add to knowledge base)
- [x] update model with added actions
- [x] update model with removed actions (but add to knowledge base)
- [x] update knowledge base formatting
- [x] general probing questions
- [x] add input for human red agents
- [x] update knowledge base with rules 
- [x] update model formatting with confidence score
- [x] save off hypothesis to YAML file
- [x] save updated model (and initial confidence score) to YAML file
- [x] save updated knowledge base to YAML file
- [ ] remove SAR code and SAR todos

## SAR TODOs

Model deployment
- [ ] launch node to load model, format data, get predictions, and send off predictions appropriately
- [ ] reporting of safety/risk scores and evaluation, predictions of risk mitigating actions
- [ ] no online training for now, but could add for later

Stretch goals
- [ ] Safety score and risk score computations
	- [ ] estimate safety/risk of current plan/task
	- [ ] relevant for reporting
- [ ] Online data point recording
	- [ ] record data points online during task execution, store in separate dataset?
	- [ ] relearn from dataset and/or suggestions to relearn from dataset when new data is collected
- [ ] Online counterfactual reasoning for identifying upstream decision points
	- [ ] report actions to user
	- [ ] ask user for input on what action should have been taken
	- [ ] generate additional data points based on user input
	- [ ] add to online dataset
