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
- [x] remove SAR code and SAR todos

ChatGPT Blue Team: accept input from ChatGPT and update models
- [x] modify printing and red teaming levels to accept ChatGPT inputs
- [ ] additional functions for processing ChatGPT input from transcripts
- [ ] seed several model problems
- [ ] several iterations of HRRTing levels for problems until ChatGPT doesn't provide sufficient info

Model planning problems:
- [ ] incorporate pyperplan
- [ ] convert model files to STRIPS files
- [ ] research failure modes
- [ ] process for randomizing task goals and failures
- [ ] pipeline for attempting planning, updating model confidence, and moving to next model
- [ ] record data about experiments to file
- [ ] model planning experiments
