#!/usr/bin/env python

import rospy
import actionlib
from actionlib import SimpleActionClient, SimpleActionServer
from tf.listener import TransformerROS
from tf2_ros.transform_listener import TransformListener
from custom_msgs.msg import PickAction

# For arm control
import moveit_commander
from moveit_commander.conversions import pose_to_list
import geometry_msgs.msg
import std_srvs.srv
import moveit_msgs
import shape_msgs

# For aruco
from aruco_msgs.msg import Marker
from aruco_msgs.msg import MarkerArray

# For gripper control
from gripper_control import GripperControl
from gazebo_ros_link_attacher.srv import Attach, AttachRequest, AttachResponse

# For head control
from look_to_point import LookToPoint
import copy
from std_srvs.srv import Empty

# To tuck the arm
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal
from sensor_msgs.msg import JointState

import tf2_ros
import tf2_geometry_msgs
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import math as m

from visualization_msgs.msg import Marker

from actionlib_msgs.msg import GoalStatus

class PlaceActionClass(object):
    def __init__(self):
        self._action_name = "controls/place_server"
        self._as = actionlib.SimpleActionServer(self._action_name, PickAction, execute_cb=self.execute_cb, auto_start = False)
        self._aruco_sub = rospy.Subscriber("aruco_marker_publisher/markers", MarkerArray, self.aruco_cb)
        self._as.start()
        self._aruco_pose = geometry_msgs.msg.PoseStamped()
        self._aruco_id = 0
        self._use_aruco = False
        self._aruco_found = False

        self._object_width = 0.05
        self._object_height = 0.10

        self._vis_pub = rospy.Publisher("visualization_marker", Marker, queue_size=0)


        self._scene = moveit_commander.PlanningSceneInterface()
        arm_side = "right"
        group_name = "arm_" + arm_side + "_torso"
        self._group = moveit_commander.MoveGroupCommander(group_name)
        self._robot = moveit_commander.RobotCommander()

        self._clear_octomap_srv = rospy.ServiceProxy(
            '/clear_octomap', Empty)
        self._clear_octomap_srv.wait_for_service()

        # Transform gripper_tool_link to frame between fingers
        self._tfBuffer = tf2_ros.Buffer()
        self._listener = tf2_ros.TransformListener(self._tfBuffer)


    def publish_vis_marker(self, pose):
        # Method to display a marker in RViz.
        # You can call this method with a goal pose to visualise where the arm is trying to move for debugging purposes.
        marker = Marker()
        marker.header.frame_id = "base_footprint"
        marker.header.stamp = rospy.Time()
        marker.ns = "marker_vis"
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.pose = pose
        marker.scale.x = 0.1
        marker.scale.y = 0.01
        marker.scale.z = 0.01
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        self._vis_pub.publish(marker)

    def aruco_cb(self, msg):
        # Callback to update pose of detected aruco marker on the object
        if self._use_aruco:
            for marker in msg.markers:
                if marker.id == self._aruco_id:
                    self._aruco_found = True
                    self._aruco_pose = marker.pose.pose


    def execute_cb(self, goal):
        r = rospy.Rate(1)
        print('PLACE GOAL: ', goal)

        # Move gripper to marker position
        arm_goal = copy.deepcopy(self._aruco_pose.pose)
        gripper_side = goal.gripper_side.rstrip('grip')
        self._group = moveit_commander.MoveGroupCommander('arm_{}_torso'.format(gripper_side))
        
        # Place basket using play motion
        client = actionlib.SimpleActionClient("play_motion", PlayMotionAction)
        client.wait_for_server()
        rospy.loginfo("...connected.")

        rospy.wait_for_message("joint_states", JointState)
        rospy.sleep(3.0)

        rospy.loginfo("Place basket...")
        place_goal = PlayMotionGoal()
        place_goal.motion_name = 'place_basket'
        place_goal.skip_planning = False

        client.send_goal(place_goal)
        client.wait_for_result(rospy.Duration(10.0))
        rospy.loginfo("Basket placed.")

    # 4. Open the gripper
        #### YOUR CODE ####
        # open gripper
        gripper_right = GripperControl(gripper_side)
        gripper_right.run('open')
        rospy.sleep(4)
        rospy.loginfo("opened gripper")

    # 5. Detach the 'collision object' in Moveit
        box_name = "aruco_cube_" + str(self._aruco_id)

        attached_objects = self._scene.get_attached_objects([box_name])
        is_attached = len(attached_objects.keys()) > 0
        rospy.loginfo(attached_objects.keys())

        if is_attached:
            eef_link = self._group.get_end_effector_link()
            self._scene.remove_attached_object(eef_link, name=box_name)
            rospy.loginfo("collision object detached succesfully")


    # 6. Unlink arm an object so that they no longer move together
        detach_srv = rospy.ServiceProxy('link_attacher_node/detach', Attach)
        detach_srv.wait_for_service()
        rospy.loginfo("Detaching Product and gripper")
        req = AttachRequest()
        req.model_name_1 = "basket"
        req.link_name_1 = "link"
        req.model_name_2 = "tiago_dual"
        req.link_name_2 = "arm_{}_7_link".format(gripper_side)

        detached = detach_srv.call(req)
        rospy.loginfo("Object detached: %s", detached)

    # 7. Move the arm back to a neutral position
        #### YOUR CODE ####
        rospy.loginfo("Tuck arm...")
        place_goal = PlayMotionGoal()
        place_goal.motion_name = 'home_{}'.format(gripper_side)
        place_goal.skip_planning = False

        client.send_goal(place_goal)
        client.wait_for_result(rospy.Duration(10.0))
        rospy.loginfo("Arm tucked.")

        self._scene.remove_world_object(box_name)

        goal_state = client.get_state()

        # if goal_state == GoalStatus.SUCCEEDED:
        #     self._as.set_succeeded()
        #     place_goal = PlayMotionGoal()
        #     place_goal.motion_name = 'wave'
        #     place_goal.skip_planning = False

        #     client.send_goal(place_goal)
        client.wait_for_result(rospy.Duration(10.0))

        # else:
        #     rospy.logfatal('Failed to tuck arm..')
        #     self._as.set_aborted()



if __name__ == "__main__":
    rospy.init_node("place_server")
    rospy.loginfo("place server started")

    PlaceActionClass()

    while not rospy.is_shutdown():
        rospy.sleep(1)
