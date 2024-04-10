#---------------------------------------------------------------------------
#     Package Setup
#---------------------------------------------------------------------------
cmake_minimum_required(VERSION 3.1)
project(safety_aware_reasoning)

set(INSTALL_LIB_DIR ${CMAKE_INSTALL_PREFIX}/lib)

#------------------------------------------------------------------------
#     Build Setup
#------------------------------------------------------------------------
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  #dynamic_reconfigure
  #geometry_msgs
  #std_msgs
  #message_generation
)

#------------------------------------------------------------------------
#     dynamic_reconfigure Param Config Setup
#------------------------------------------------------------------------
#generate_dynamic_reconfigure_options(
#  cfg/SafetyExceptionReporterParams.cfg
#)
#message("generated param config files!")

#------------------------------------------------------------------------
#     ROS Messages Setup
#------------------------------------------------------------------------
# generate messages
#add_message_files(
#  FILES
#
#)

# dependencies for generated messages
#generate_messages(
#  DEPENDENCIES
#  geometry_msgs
#  std_msgs
#)

#message("generated messages!")

#------------------------------------------------------------------------
#     catkin Setup
#------------------------------------------------------------------------
catkin_package(
  CATKIN_DEPENDS roscpp rospy #geometry_msgs std_msgs message_runtime
)
include_directories(${catkin_INCLUDE_DIRS})

#------------------------------------------------------------------------
#     Include Directories
#------------------------------------------------------------------------
include_directories ("${PROJECT_SOURCE_DIR}/..")
include_directories ("${PROJECT_SOURCE_DIR}")

#-----------------------------------------------------------------------
#     Python Executable Installation
#-----------------------------------------------------------------------
install(PROGRAMS
  # YAML CHECKING HELPER CLASSES
  scripts/yaml_formatting_checks.py
  # STATE SPACE CLASSES/SCRIPTS
  scripts/consequence_state.py
  scripts/consequence_state_reader.py
  scripts/likelihood_consequence_risk.py
  scripts/risky_condition.py
  scripts/risky_condition_reader.py
  # ACTION SPACE CLASSES/SCRIPTS
  scripts/risk_mitigating_action.py
  scripts/risk_mitigating_action_reader.py
  # POLICY SUPERCLASS
  scripts/policy_data_point.py
  # POLICY CLASSES/SCRIPTS
  scripts/risk_mitigating_policy_data_point.py
  scripts/risk_mitigating_policy_data_reader.py
  # COUNTER FACTUAL POLICY CLASSES/SCRIPTS
  scripts/counter_factual_policy_data_point.py
  scripts/counter_factual_policy_data_reader.py
  # RED TEAMING HELPERS/CLASSES/SCRIPTS
  scripts/red_team_policy.py
  scripts/red_team_command_line_tools.py
  scripts/red_team_data_extension.py
  DESTINATION lib/${PROJECT_NAME} #${CATKIN_PACKAGE_SHARE_DESTINATION}
)
message("installed python scripts!")
