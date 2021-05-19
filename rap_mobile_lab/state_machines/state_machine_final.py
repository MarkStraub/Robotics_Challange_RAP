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
        self.marker_pose_x = 0
        self.marker_pose_y = 0
        self.pose = None

        rospy.Subscriber('/visualization_marker', Marker, self.callback_marker, queue_size=1)

    def execute(self, userdata):
        rospy.loginfo('Executing state ROTATION')

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
            transform = self.tf_buffer.lookup_transform('summit_xl_odom', data.header.frame_id, data.header.stamp)
         
            pose_transformed = tf2_geometry_msgs.do_transform_pose(data, transform)

            # Check if the found marker is not the same as before
            if self.cmpr(self.marker_pose_x, pose_transformed.pose.position.x) or self.cmpr(self.marker_pose_y, pose_transformed.pose.position.y):
                # Update marker pose
                self.marker_pose_x = round(pose_transformed.pose.position.x, 2)
                self.marker_pose_y = round(pose_transformed.pose.position.y, 2)

                self.pose = pose_transformed.pose
            
    
    def cmpr(self, old, new):
        new = round(new, 2)
        if new < (old -1.0) or new > (old + 1.0):
            return True
        else:
            return False


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
                               transitions={'reached':'ROTATION'},
                               remapping={'nav_pose_in':'pose'})

    # Execute SMACH plan
    outcome = sm.execute()

    # Stop rotating
    moving.rotate(0.0)


if __name__ == '__main__':
    main()
