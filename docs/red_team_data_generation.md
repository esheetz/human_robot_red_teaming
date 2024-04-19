# Human-Robot Red Teamed Data Generation

In order to explore the space of risky situations that may arise in a given domain, the human and robot will engage in a human-robot red teaming exercise where they will generate policy data points to train the risk mitigating action utility model.

There are two types of data points that will be generated: random risky scenarios and random counter-factual scenarios.  To launch either data generation protocol, run:

```
roslaunch safety_aware_reasoning red_team_risky_scenario_data_extension.launch
roslaunch safety_aware_reasoning red_team_counter_factual_data_extension.launch
```

Note that each has the following optional launch arguments:
- `robot` to specify the robot subfolder under the `config/` directory; current supported robots are `val` (Valkyrie) or `clr` (ChonkUR L. Rail-E)
- `env` to specify the environment; current supported environments are `household` and `lunar_habitat`
- `num_points` to specify the number of data points to generate
- `auto_gen_data` to flag whether the data should be automatically generated based on robot- and domain-specific knowledge-based rules

By default, `auto_gen_data` is set to `true`, in which case the robot will apply [robot- and domain-specific knowledge-based rules](docs/robot_specific_knowledge.md) to automatically generate data points.  If `auto_gen_data` is set to `false`, then the human can interactively provide data points for randomly generated risky scenarios and counter-factual scenarios via the command line.

The policy data generated through human-robot red teaming is stored in the `data/` directory.  Since these files are written by the red team data extension nodes, they will be formatted properly.  If you want to view the contents of these files, you can use the following data readers:

```
# human-robot red teamed risky scenario policy data
roslaunch safety_aware_reasoning risk_mitigating_policy_data_reader.launch human_generated_data:=false

# human-robot red teamed counter-factual policy data
roslaunch safety_aware_reasoning counter_factual_policy_data_reader.launch
```
