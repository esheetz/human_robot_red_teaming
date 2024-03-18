#!/usr/bin/env python3
"""
Test Node
Emily Sheetz, NSTGRO VTE 2024
"""

import rospy
from process_safety_conditions import SafetyConditionReader

if __name__ == '__main__':
    # initialize node
    rospy.init_node("TestNode")

    # create reporter node
    # reporter_node = SafetyExceptionReporterNode()

    safety_reader = SafetyConditionReader()

    print("Created node!")

    # run node, wait for reports
    while not rospy.is_shutdown():
        rospy.spin()

    rospy.loginfo("[Safety Exception Reporter Node] Node stopped, all done!") # TODO
