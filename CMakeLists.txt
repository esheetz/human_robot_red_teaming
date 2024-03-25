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
# TODO
install(PROGRAMS
  scripts/likelihood_consequence_risk.py
  scripts/risky_condition.py
  scripts/risky_condition_reader.py
  scripts/risk_mitigating_action.py
  scripts/risk_mitigating_action_reader.py
  scripts/risk_mitigating_policy_starter_reader.py
  scripts/yaml_formatting_checks.py
  scripts/test.py # TODO remove?
  DESTINATION lib/${PROJECT_NAME} #${CATKIN_PACKAGE_SHARE_DESTINATION}
)
message("installed python scripts!")
