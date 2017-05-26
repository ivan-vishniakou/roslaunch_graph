#!/usr/bin/python
import sys
import rospy
import smach
import smach_ros


class ScenarioNameTest(smach.StateMachine):
    """docstring for Scenario Name test."""
    def __init__(self, args):
        self.args = args
        smach.StateMachine.__init__(self, outcomes=['DONE'])

        with self:
            # Add states here as normal
            smach.StateMachine.add()


def main():
    rospy.init_node('scenario_name_test')

    SM = ScenarioTest()

    # Start SMACH viewer
    smach_viewer = smach_ros.IntrospectionServer('scenario_name_test', SM,
                                                 'SCENARIO_NAME_TEST')
    smach_viewer.start()

    result = SM.execute()
    # stop SMACH viewer
    while (result is None):
        rospy.spin()
    rospy.loginfo('Scenario complete.')
    smach_viewer.stop()


if __name__ == '__main__':
    main()
