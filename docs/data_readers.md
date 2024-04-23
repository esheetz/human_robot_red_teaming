# Data Readers

This package captures human-robot red teamed knowledge for safely executing tasks in safety-critical domains.  This knowledge includes:
- *risky conditions* that may arise during task execution
- *consequence states* that describe the possible consequences of risky conditions
- *risk mitigating actions* that can be taken by the robot to reduce risks
- *risk mitigating policy data* that describes the tuples of:
    - risk conditions
    - anticipated consequence states before any action is taken
    - risk mitigating action
    - anticipated consequence states after the risk mitigating action is taken

Each of these types of knowledge are captured in YAML files under the `config/` directory.  Each different robot has its own folder under the `config/` directory.  Each different environment is a key in the robot's YAML file to capture how that robot acts under different environments.

To test the formatting of the `config/` YAML files, run the following data readers:

```
# risky conditions
roslaunch safety_aware_reasoning risky_condition_reader.launch

# consequence states
roslaunch safety_aware_reasoning consequence_state_reader.launch

# risk mitigating actions
roslaunch safety_aware_reasoning risk_mitigating_action_reader.launch

# risk mitigating policy data
roslaunch safety_aware_reasoning risk_mitigating_policy_data_reader.launch
```

Note that each reader has two optional launch arguments:
- `robot` to specify the robot subfolder under the `config/` directory; current supported robots are `val` (Valkyrie), `clr` (ChonkUR L. Rail-E), or `val_clr` (which treats both Valkyrie and CLR as the same robot).
- `env` to specify the environment; current supported environments are `household` and `lunar_habitat`
