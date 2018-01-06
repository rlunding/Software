#!/usr/bin/env python
import rospy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool
from std_msgs.msg import Float32
from geometry_msgs.msg import PoseArray

### note you need to change the name of the robot to yours here
# from obst_avoid.detector import Detector
from duckietown_utils import get_base_name, rgb_from_ros, rectify, load_camera_intrinsics


class ObstAvoidNode(object):
    def __init__(self):
        self.node_name = "Obstacle Avoidance Node"
        robot_name = rospy.get_param("~robot_name", "")
        self.count = 1

        # self.detector = Detector(robot_name=robot_name)  # not sure what that does

        ########################
        ###### Publishers ######
        # Emergency brake to be triggered iff == 1
        self.pub_topic = 'obstacle_emergency_stop_flag'.format(robot_name)
        self.brake_pub = rospy.Publisher(self.pub_topic, Bool, queue_size=1)

        self.pub_topic = 'obstacle_avoidance_active_flag'.format(robot_name)
        self.brake_pub = rospy.Publisher(self.pub_topic, Bool, queue_size=1)

        # Desired d. Only read when Obstacle is detected
        self.pub_topic = '/{}/obst_avoid/d_desired'.format(robot_name)
        self.publisher = rospy.Publisher(self.pub_topic, Float32, queue_size=1)
        # ToDo: Consider Float32MultiArray if theta will be used.
        # Desired theta for dev-controllers to tune the controls
        self.pub_topic = '/{}/obst_avoid/theta_desired'.format(robot_name)
        self.publisher = rospy.Publisher(self.pub_topic, Float32, queue_size=1)

        ########################
        ###### Subscribers #####
        self.sub_topic = '/{}/obst_detect/posearray'.format(robot_name)
        self.subscriber = rospy.Subscriber(self.sub_topic, PoseArray, self.obstacleCallback)

        # ToDo: d_current, theta_current
        self.sub_topic = '/{}/localization_node/'.format(robot_name)
        self.subscriber = rospy.Subscriber(self.sub_topic, CompressedImage, self.dCallback)
        self.sub_topic = '/{}/localization_node/'.format(robot_name)
        self.subscriber = rospy.Subscriber(self.sub_topic, CompressedImage, self.thetaCallback)

        # ToDo: intersection
        self.sub_topic = '/{}/'.format(robot_name)
        self.subscriber = rospy.Subscriber(self.sub_topic, CompressedImage, self.intersectionCallback)

        # ToDo: line_detection
        self.sub_topic = '/{}/Anti_Instagram'.format(robot_name)
        self.subscriber = rospy.Subscriber(self.sub_topic, CompressedImage, self.lineCallback)

    def obstacleCallback(self, obstacle_poses):
        amount_obstacles = len(obstacle_poses)
        amount_obstacles_on_track = 0
        rospy.loginfo('Callbakkkk')
        for x in range(0, amount_obstacles, 1):
            if obstacle_poses[0].pose.z > 0:
                amount_obstacles_on_track+=1
        if amount_obstacles_on_track == 0:
            return
        if amount_obstacles_on_track == 1:
            avoider()
        else:
            self.brake_pub.publish(1);

        return

    def dCallback(self):
        return

    def thetaCallback(self):
        return

    def thetaCallback(self):
        return

    def intersectionCallback(self):
        return

    def lineCallback(self):
        return

    def onShutdown(self):
        rospy.loginfo('Shutting down Obstacle Detection, back to unsafe mode')


if __name__ == '__main__':
    rospy.init_node('obst_avoidance_node', anonymous=False)
    obst_avoidance_node = ObstAvoidNode()
    rospy.on_shutdown(obst_avoidance_node.onShutdown)
    rospy.spin()