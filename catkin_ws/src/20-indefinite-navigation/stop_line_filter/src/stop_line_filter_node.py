#!/usr/bin/env python
import rospy
import tf
import numpy as np
from scipy.spatial import cKDTree
from duckietown_msgs.msg import SegmentList, Segment, BoolStamped, StopLineReading, LanePose, FSMState
from std_msgs.msg import Float32
from geometry_msgs.msg import Point
import time
import math
from visualization_msgs.msg import Marker, MarkerArray

class StopLineFilterNode(object):
    def __init__(self):
        self.node_name = rospy.get_name()
        rospy.loginfo("[%s] Initializing " % self.node_name)

        # Get vehicle name from namespace
        self.veh_name = rospy.get_namespace().strip("/")
        rospy.loginfo("[%s] Vehicle name: %s" % (self.node_name, self.veh_name))

        # state vars
        self.lane_pose = LanePose()
        self.state = "JOYSTICK_CONTROL"
        self.sleep = False
        self.active = True

        # params
        self.stop_distance = self.setupParam("~stop_distance", 0.25)  # forward threshold distance to the stop line
        self.stop_distance_y = self.setupParam("~stop_distance_y", 0.10)  # sideways threshold distance to the stop line
        self.min_segs = self.setupParam("~min_segs", 3)  # minimum number of red segments threshold
        self.off_time = self.setupParam("~off_time", 2)  # time (in sec) to sleep
        self.point_distance = self.setupParam("~point_distance", 0.10)  # distance between points

        # Setup publishers
        self.pub_stop_line_reading = rospy.Publisher("~stop_line_reading", StopLineReading, queue_size=1)
        self.pub_stop_line_point = rospy.Publisher("~stop_line_point", MarkerArray, queue_size=1)
        self.pub_at_stop_line = rospy.Publisher("~at_stop_line", BoolStamped, queue_size=1)

        # Setup subscribers
        self.sub_segs = rospy.Subscriber("~segment_list", SegmentList, self.processSegments)
        self.sub_lane = rospy.Subscriber("~lane_pose", LanePose, self.processLanePose)
        self.sub_mode = rospy.Subscriber("fsm_node/mode", FSMState, self.processStateChange)
        self.sub_switch = rospy.Subscriber("~switch", BoolStamped, self.cbSwitch)

        self.params_update = rospy.Timer(rospy.Duration.from_sec(1.0), self.updateParams)

        rospy.loginfo("[%s] Initialzed." % self.node_name)

    def setupParam(self,param_name,default_value):
        value = rospy.get_param(param_name,default_value)
        rospy.set_param(param_name,value)  # Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " % (self.node_name, param_name, value))
        return value

    def updateParams(self, event):
        self.stop_distance = rospy.get_param("~stop_distance")
        self.stop_distance_y = rospy.get_param("~stop_distance_y")
        self.min_segs = rospy.get_param("~min_segs")
        self.off_time = rospy.get_param("~off_time")
        self.point_distance = rospy.get_param("~point_distance")

    def processStateChange(self, msg):
        if self.state == "INTERSECTION_CONTROL" and (msg.state == "LANE_FOLLOWING" or msg.state == "PARALLEL_AUTONOMY"):
            self.afterIntersectionWork()
        self.state = msg.state

    def afterIntersectionWork(self):
        rospy.loginfo("stop line sleep start")
        self.sleep = True
        rospy.sleep(self.off_time)
        self.sleep = False
        rospy.loginfo("stop line sleep end")

    def cbSwitch(self, switch_msg):
        self.active = switch_msg.data
        if self.active and self.state == "INTERSECTION_CONTROL":
            self.afterIntersectionWork()

    def processLanePose(self, lane_pose_msg):
        self.lane_pose = lane_pose_msg

    def processSegments(self, segment_list_msg):
        if not self.active or self.sleep:
            return

        # Get all red points (one red segment is two points)
        points = None
        for segment in segment_list_msg.segments:
            if segment.color != segment.RED:
                continue
            if segment.points[0].x < 0 or segment.points[1].x < 0:  # the point is behind us
                continue

            p1_lane = self.to_lane_frame(segment.points[0])
            p2_lane = self.to_lane_frame(segment.points[1])

            if points is None:
                points = np.vstack([p1_lane, p2_lane])
            else:
                points = np.vstack([points, p1_lane, p2_lane])

        stop_line_reading_msg = StopLineReading()
        stop_line_reading_msg.header.stamp = segment_list_msg.header.stamp
        stop_line_reading_msg.stop_line_detected = False
        stop_line_reading_msg.at_stop_line = False

        # Exit if not enough red segments was found
        if points is None or points.shape[0] <= self.min_segs * 2:
            self.pub_stop_line_reading.publish(stop_line_reading_msg)
            return

        # Prepare visualization
        marker_array = MarkerArray()
        # Find groups of points, i.e. multiple stop line candidates
        tree = cKDTree(points)
        groups = self.make_equiv_classes(tree.query_pairs(r=self.point_distance))
        for group in groups:
            points_in_group = points[list(group)]
            if points_in_group.shape[0] <= self.min_segs * 2:  # ensure enough points in group
                continue
            mean = points_in_group.mean(axis=0)
            at_stop_line = mean[0] < self.stop_distance and math.fabs(mean[1]) < self.stop_distance_y
            if at_stop_line:
                stop_line_reading_msg.stop_line_detected = True
                stop_line_point = Point()
                stop_line_point.x = mean[0]
                stop_line_point.y = mean[1]
                stop_line_reading_msg.stop_line_point = stop_line_point
                stop_line_reading_msg.at_stop_line = True

            # Add visualization marker
            lamdba_, v = np.linalg.eig(np.cov(points_in_group.T))
            angle = np.arccos(v[0, 0]) + np.pi / 2
            marker_array.markers.append(self.stop_line_to_marker(mean, angle, at_stop_line))

        # Set IDs and publish markers
        marker_id = 0
        for m in marker_array.markers:
            m.id = marker_id
            marker_id += 1
        self.pub_stop_line_point.publish(marker_array)

        if stop_line_reading_msg.at_stop_line:
            msg = BoolStamped()
            msg.header.stamp = stop_line_reading_msg.header.stamp
            msg.data = True
            self.pub_at_stop_line.publish(msg)

        self.pub_stop_line_reading.publish(stop_line_reading_msg)

    def to_lane_frame(self, point):
        p_homo = np.array([point.x, point.y, 1])
        phi = self.lane_pose.phi
        d = self.lane_pose.d
        t = np.array([[math.cos(phi), -math.sin(phi), 0],
                      [math.sin(phi), math.cos(phi), d],
                      [0, 0, 1]])
        p_new_homo = t.dot(p_homo)
        p_new = p_new_homo[0:2]
        return p_new

    def stop_line_to_marker(self, point, angle, at_stop_line):
        marker = Marker()
        marker.header.frame_id = self.veh_name
        marker.header.stamp = rospy.get_rostime()
        marker.ns = self.veh_name + '/stop_line'
        marker.type = Marker.MESH_RESOURCE
        marker.mesh_resource = 'package://stop_line_filter/meshes/fence.stl'
        marker.action = Marker.ADD
        marker.lifetime = rospy.Duration.from_sec(1.0)

        yaw_quat = tf.transformations.quaternion_about_axis(-angle, [0, 0, 1])
        marker.pose.orientation.x = yaw_quat[0]
        marker.pose.orientation.y = yaw_quat[1]
        marker.pose.orientation.z = yaw_quat[2]
        marker.pose.orientation.w = yaw_quat[3]

        marker.pose.position.x = point[0]
        marker.pose.position.y = point[1]
        marker.pose.position.z = 0.01

        marker.scale.x = marker.scale.y = marker.scale.z = 0.003

        if at_stop_line:
            marker.color.r = 1.0
            marker.color.g = 0.0
        else:
            marker.color.r = 0.0
            marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1

        return marker

    @staticmethod
    def make_equiv_classes(pairs):
        groups = {}
        for (x, y) in pairs:
            xset = groups.get(x, {x})
            yset = groups.get(y, {y})
            jset = xset | yset
            for z in jset:
                groups[z] = jset
        return set(map(tuple, groups.values()))

    def on_shutdown(self):
        rospy.loginfo("[%s] Shutdown." % self.node_name)


if __name__ == '__main__':
    rospy.init_node('stop_line_filter', anonymous=False)
    lane_filter_node = StopLineFilterNode()
    rospy.on_shutdown(lane_filter_node.on_shutdown)
    rospy.spin()
