#!/usr/bin/env python
import sys, getopt
import rospy
import tf2_ros
import tf2_geometry_msgs
import math
from visualization_msgs.msg import Marker
from ar_track_alvar_msgs.msg import AlvarMarkers
from moveit_python.geometry import rotate_pose_msg_by_euler_angles
from moving import Moving
from set_goal import Goal
from geometry_msgs.msg import PoseStamped
#from tf_conversions.transformations import quaternion_from_euler


class Follow_markers():
    '''Class for follow markers.'''

    def __init__(self, distance = 0.5):
        rospy.init_node('mobile_lab', anonymous=True)
        rospy.loginfo("Initialized RAP MOBILE LAB")
        self.moving = Moving()
        self.goal = Goal("summit_xl_map", distance=distance)
        self.tf_buffer = tf2_ros.Buffer();
        self.listener = tf2_ros.TransformListener(self.tf_buffer)

        self.marker_pose_x = 0
        self.marker_pose_y = 0
        
        self.marker_sub = rospy.Subscriber('/visualization_marker', Marker, self.callback_marker, queue_size=1)
        self.goal_pub = rospy.Publisher("/nav_goal", PoseStamped)

        rospy.loginfo("---------------------------------------------")
        rospy.loginfo("START THE PROGRAMM")

        # Rotate the whole time
        while not rospy.is_shutdown():
            self.moving.rotate(-0.5)

        # Stop rotating at the end
        self.moving.rotate(0)


    def callback_marker(self, data):
        # Check if it is the correct marker
        if data.id == 13:
            # Position depends to map
            transform = self.tf_buffer.lookup_transform('summit_xl_odom', data.header.frame_id, rospy.Time(0))
            
            # rospy.loginfo("Transform: \n%s", transform)
            pose_transformed = tf2_geometry_msgs.do_transform_pose(data, transform)

            # rospy.loginfo("Pose Transformed: \n%s", pose_transformed)

            # Check if the found marker is not the same as before
            if self.cmpr(self.marker_pose_x, pose_transformed.pose.position.x) or self.cmpr(self.marker_pose_y, pose_transformed.pose
            .position.y):
                rospy.loginfo("NEW MARKER FOUND")
                # Update marker pose
                self.marker_pose_x = round(pose_transformed.pose.position.x, 2)
                self.marker_pose_y = round(pose_transformed.pose.position.y, 2)

                # let's cheat, using the tf of the marker
                goal_pose = PoseStamped()
                goal_pose.header.stamp = data.header.stamp
                goal_pose.header.frame_id = "ar_marker_13"
                goal_pose.pose.position.z += 1 # should use distance arg
                # quat = quaternion_from_euler(math.pi / 2, 0, )
                goal_pose.pose.orientation.x = -0.7071068
                goal_pose.pose.orientation.w = 0.7071068

                self.goal_pub.publish(goal_pose)
                rospy.sleep(1)

                # transform to map frame 
                transform = self.tf_buffer.lookup_transform('summit_xl_map', goal_pose.header.frame_id, goal_pose.header.stamp)
                goal_on_map = tf2_geometry_msgs.do_transform_pose(goal_pose, transform)

                # # hack, fix quaternion if ar_track didn't get it right
                # goal_on_map.pose.orientation.x = 0
                # goal_on_map.pose.orientation.y = 0

                self.goal_pub.publish(goal_on_map)

                self.goal.movebase_client(goal_on_map)
            

    def cmpr(self, old, new):
        new = round(new, 2)
        if new < (old -1.0) or new > (old + 1.0):
            return True
        else:
            return False

# Process arguments if they have been passed
def main(argv):
    distance = 0.5
    try:
        opts, args = getopt.getopt(argv,"hd:",["distance="])
    except getopt.GetoptError:
        print("follow_markers.py -d <distance>")
        sys.exit(2)
    for opt, arg in opts:
        if opt == '-h':
            print("follow_markers.py -d <distance>")
            sys.exit()
        elif opt in ("-d", "--distance"):
            distance = float(arg)
    
    print("Distance is " + str(distance))
    try:
        follow_markers = Follow_markers(distance=distance)
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.logerr("Error")

# If the python node is executed as main process (sourced directly)
if __name__ == '__main__':
    main(sys.argv[1:])