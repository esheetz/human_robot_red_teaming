# Human-Robot Red Teamed Data

- [Data Generation](#data-generation)
- [Data Pre-Processing](#data-pre-processing)



## Data Generation

In order to explore the space of risky situations that may arise in a given domain, the human and robot will engage in a human-robot red teaming exercise where they will generate policy data points to train the risk mitigating action utility model.

> :warning: **NOTE that due to randomness of human-robot red teaming exercise**, data generation may be somewhat inefficient in that the same random scenarios are generated multiple times.  This is because for non-automated data generation, it can be helpful for the human team to re-consider multiple scenarios.  For automated data generation, we keep the randomness to avoid code rewrite.  In practice, the automated data generation will consider the full state space it may just take some time to randomly find each point.

There are two types of data points that will be generated: random risky scenarios and random counter-factual scenarios.  To launch either data generation protocol, run:
```
roslaunch safety_aware_reasoning red_team_risky_scenario_data_extension.launch
roslaunch safety_aware_reasoning red_team_counter_factual_data_extension.launch
```

Note that each has the following optional launch arguments:
- `robot` to specify the robot subfolder under the `config/` directory; current supported robots are `val` (Valkyrie), `clr` (ChonkUR L. Rail-E), or `val_clr` (which treats both Valkyrie and CLR as the same robot).
- `env` to specify the environment; current supported environments are `household` and `lunar_habitat`
- `num_points` to specify the number of data points to generate
- `auto_gen_data` to flag whether the data should be automatically generated based on robot- and domain-specific knowledge-based rules

By default, `auto_gen_data` is set to `true`, in which case the robot will apply [robot- and domain-specific knowledge-based rules](robot_specific_knowledge.md) to automatically generate data points.  If `auto_gen_data` is set to `false`, then the human can interactively provide data points for randomly generated risky scenarios and counter-factual scenarios via the command line.

The policy data generated through human-robot red teaming is stored in the `data/` directory.  Since these files are written by the red team data extension nodes, they will be formatted properly.  If you want to view the contents of these files, you can use the following data readers:
```
# human-robot red teamed risky scenario policy data
roslaunch safety_aware_reasoning risk_mitigating_policy_data_reader.launch human_generated_data:=false

# human-robot red teamed counter-factual policy data
roslaunch safety_aware_reasoning counter_factual_policy_data_reader.launch
```



## Data Pre-Processing

Once data is generated, it can be pre-processed to create a proper dataset and saved as CSV files for later analysis and model training.  To process the data, run:
```
roslaunch safety_aware_reasoning data_processing.py
```

Note that by default, this node assumes that data has been generated for all robots in all environments, and pre-processes all data at once.  If data needs to be processed for a specific robot in a specific environment, the following optional launch arguments can be used:
- `robot` to specify the robot subfolder under the `config/` directory; current supported robots are `val` (Valkyrie), `clr` (ChonkUR L. Rail-E), or `val_clr` (which treats both Valkyrie and CLR as the same robot).
- `env` to specify the environment; current supported environments are `household` and `lunar_habitat`
