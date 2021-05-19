#!/usr/bin/env python

import rospy
import smach
from geometry_msgs.msg import Pose
from moving import Moving
from set_goal import Goal


# define state rotating (searching for marker)
class Rotation(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['rotate','found'])
        self.counter = 0
        self.moving = Moving()

    def execute(self, userdata):
        rospy.loginfo('Executing state ROTATION')
        if self.counter < 13:
            self.moving.rotate(-0.5)
            rospy.sleep(0.1)
            self.counter += 1
            return 'rotate'
        else:
            return 'found'


# define state navigating (approaching marker)
class Navigation(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['reached'])
        self.goal = Goal("summit_xl_map")

    def execute(self, userdata):
        rospy.loginfo('Executing state NAVIGATION')
        pose = Pose()
        pose.position.x = 2
        pose.position.y = -1
        pose.orientation.w = 0.302
        a = self.goal.movebase_client(pose)
        return 'reached'
        

# main
def main():
    rospy.init_node('smach_example_state_machine')
    moving = Moving()

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['finished'])

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('ROTATION', Rotation(), 
                               transitions={'rotate':'ROTATION', 
                                            'found':'NAVIGATION'})
        smach.StateMachine.add('NAVIGATION', Navigation(), 
                               transitions={'reached':'finished'})

    # Execute SMACH plan
    outcome = sm.execute()
    moving.rotate(0.0)


if __name__ == '__main__':
    main()
