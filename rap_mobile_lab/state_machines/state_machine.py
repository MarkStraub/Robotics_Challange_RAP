#!/usr/bin/env python

import rospy
import smach
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import Pose
from visualization_msgs.msg import Marker
from moving import Moving
from set_goal import Goal


# define state rotating (searching for marker)
class Rotation(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['rotate','found'], output_keys=['rot_pose_out'])
        self.counter = 0
        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)
        self.moving = Moving()
        self.pose = None

    def execute(self, userdata):
        rospy.loginfo('Executing state ROTATION')
        rospy.Subscriber('/visualization_marker', Marker, self.callback_marker, queue_size=1)

        # Rotate until the a marker has been found
        if self.pose is None:
            self.moving.rotate(-0.5)
            rospy.sleep(0.1)
            return 'rotate'
        else:
            # Pass pose to Navigation
            userdata.rot_pose_out = self.pose
            # Set pose to None for the next time
            self.pose = None
            return 'found'

    def callback_marker(self, data):
        # Check if it is the correct marker
        if data.id == 13:
            # Position depends to map
            transform = self.tf_buffer.lookup_transform('summit_xl_odom', data.header.frame_id, rospy.Time(0))
         
            pose_transformed = tf2_geometry_msgs.do_transform_pose(data, transform)

            self.pose = pose_transformed.pose
            

# define state navigating (approaching marker)
class Navigation(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['reached'], input_keys=['nav_pose_in'])
        self.goal = Goal("summit_xl_map")

    def execute(self, userdata):
        rospy.loginfo('Executing state NAVIGATION')
        self.goal.movebase_client(userdata.nav_pose_in)
        return 'reached'
        

# main
def main():
    rospy.init_node('smach_example_state_machine')
    moving = Moving()

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['finished'])
    sm.userdata.pose = None

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('ROTATION', Rotation(), 
                               transitions={'rotate':'ROTATION', 
                                            'found':'NAVIGATION'},
                                remapping={'rot_pose_out':'pose'})
        smach.StateMachine.add('NAVIGATION', Navigation(), 
                               transitions={'reached':'finished'},
                               remapping={'nav_pose_in':'pose'})

    # Execute SMACH plan
    outcome = sm.execute()

    # Stop rotating
    moving.rotate(0.0)


if __name__ == '__main__':
    main()
