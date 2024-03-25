#!/usr/bin/env python3
"""
Test Node
Emily Sheetz, NSTGRO VTE 2024
"""

import rospy
from risky_condition_reader import RiskyConditionReader

if __name__ == '__main__':
    # initialize node
    rospy.init_node("TestNode")

    # create reporter node
    # reporter_node = SafetyExceptionReporterNode()

    reader = RiskyConditionReader()

    print("Created node!")

    # run node, wait for reports
    while not rospy.is_shutdown():
        rospy.spin()

    rospy.loginfo("[Test Node] Node stopped, all done!") # TODO
