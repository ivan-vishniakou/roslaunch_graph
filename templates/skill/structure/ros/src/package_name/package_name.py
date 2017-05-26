#!/usr/bin/env python
import rospy
import smach

# action lib
from smach_ros import ActionServerWrapper

from mdr_actions.msg import SkillNameAction
from mdr_actions.msg import SkillNameFeedback
from mdr_actions.msg import SkillNameResult


class SetActionLibResult(smach.State):
    """docstring for SetActionLibResult."""
    def __init__(self, result):
        smach.State.__init__(self, outcomes=['succeeded'],
                             input_keys=['skill_name_goal'],
                             output_keys=['skill_name_feedback',
                                          'skill_name_result'])
        self.result = result

    def execute(self, userdata):
        result = SkillNameResult()
        result.success = self.result
        userdata.skill_name_result = result
        return 'succeeded'

# QUESTION: Is it an issue if the Action and the Class are named the same?


class SkillName(smach.StateMachine):
    """docstring for SkillName."""

    def __init__(self, arg, timeout=10):
        smach.StateMachine.__init__(self,
                                    outcomes=['OVERALL_SUCCESS',
                                              'OVERALL_FAILED', 'PREEMPTED'],
                                    input_keys=['skill_name_goal'],
                                    output_keys=['skill_name_feedback',
                                                 'skill_name_result'])

        self.arg = arg

        with self:
            # Add states as a normal state machine
            smach.StateMachine.add()

            # Add the actionlib states
            smach.StateMachine.add('SET_ACTION_LIB_SUCCESS',
                                   SetActionLibResult(True),
                                   transitions={
                                               'succeeded': 'OVERALL_SUCCESS'})
            smach.StateMachine.add('SET_ACTION_LIB_FAILURE',
                                   SetActionLibResult(False),
                                   transitions={'succeeded': 'OVERALL_FAILED'})

            # TODO Review how to add the PREEMTED state as well.


def main():
    rospy.init_node('skill_name_server')

    # Construct state machine
    sm = SkillName()

    # Smach viewer
    sis = smach_ros.IntrospectionServer('skill_name_smach_viewer', sm,
                                        '/SKILL_NAME_SMACH_VIEWER')
    sis.start()

    # Construct action server wrapper
    asw = ActionServerWrapper(
        server_name='skill_name_server',
        action_spec=SkillNameAction,
        wrapped_container=sm,
        succeeded_outcomes=['OVERALL_SUCCESS'],
        aborted_outcomes=['OVERALL_FAILED'],
        preempted_outcomes=['PREEMPTED'],
        goal_key='S_goal',
        feedback_key='skill_name_feedback',
        result_key='skill_name_result')
    # Run the server in a background thread
    asw.run_server()
    rospy.spin()


if __name__ == '__main__':
    main()
