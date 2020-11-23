#!/usr/bin/env python3
import numpy as np
import rospy
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import Twist2DStamped, WheelEncoderStamped, WheelsCmdStamped
from geometry_msgs.msg import TransformStamped, Quaternion, Vector3, Transform
from tf.transformations import quaternion_from_matrix
import tf

from geometry import SE2_from_xytheta, rotation_translation_from_SE3, SE3_from_SE2

from duckiebot_kinematics import DuckiebotKinematics


class EncoderLocalizationNode(DTROS):

    def __init__(self, node_name):

        # Initialize the DTROS parent class
        super(EncoderLocalizationNode, self).__init__(node_name=node_name,node_type=NodeType.GENERIC)
        self.veh_name = rospy.get_namespace().strip("/")
        self.radius = rospy.get_param(f'/{self.veh_name}/kinematics_node/radius', 100)
        self.baseline = 0.0968
        x_init = rospy.get_param(f'/{self.veh_name}/{node_name}/x_init', 100)
        self.pose = SE2_from_xytheta([x_init, 0, np.pi])
        self.db_kinematics = DuckiebotKinematics(radius=self.radius, baseline=self.baseline)
        self.vel_left = 0.0
        self.vel_right = 0.0
        self.encoder_ticks_left_total = 0
        self.encoder_ticks_left_delta_t = 0
        self.encoder_ticks_right_total = 0
        self.encoder_ticks_right_delta_t = 0
        self.encoder_timestamp = rospy.Time.now()
        self.max_number_ticks = 135
        init_pose_SE3 = SE3_from_SE2(self.pose)
        rot, trans = rotation_translation_from_SE3(init_pose_SE3)
        quaternion_tf = quaternion_from_matrix(init_pose_SE3)
        quaternion = Quaternion(quaternion_tf[0], quaternion_tf[1], quaternion_tf[2], quaternion_tf[3])
        translation = Vector3(trans[0], trans[1], trans[2])
        trafo = Transform(translation, quaternion)
        self.tfs_msg = TransformStamped()
        self.tfs_msg.header.frame_id = 'map'
        self.tfs_msg.header.stamp = self.encoder_timestamp
        self.tfs_msg.child_frame_id = 'encoder_baselink'
        self.tfs_msg.transform = trafo
        self.br = tf.TransformBroadcaster()

        self.sub_executed_commands = rospy.Subscriber(f'/{self.veh_name}/wheels_driver_node/wheels_cmd_executed',
                                                      WheelsCmdStamped, self.cb_executed_commands)
        self.sub_encoder_ticks_left = rospy.Subscriber(f'/{self.veh_name}/left_wheel_encoder_node/tick',
                                                       WheelEncoderStamped, self.callback_left)
        self.sub_encoder_ticks_right = rospy.Subscriber(f'/{self.veh_name}/right_wheel_encoder_node/tick',
                                                        WheelEncoderStamped, self.callback_right)

        self.pub_tf_enc_loc = rospy.Publisher(f'/{self.veh_name}/{node_name}/transform_stamped',
                                             TransformStamped, queue_size=10)

    def callback_right(self, data):
        self.encoder_timestamp = data.header.stamp
        if self.vel_right == 0.0:
            self.encoder_ticks_right_delta_t += 0
            self.encoder_ticks_right_total = data.data + 1
        else:
            delta = data.data - self.encoder_ticks_right_total
            if delta > 3 or delta < -3:
                delta = 0
            self.encoder_ticks_right_delta_t += max(min(3, delta), -3)
            self.encoder_ticks_right_total = data.data

    def callback_left(self, data):
        self.encoder_timestamp = data.header.stamp
        if self.vel_left == 0.0:
            self.encoder_ticks_left_delta_t += 0
            self.encoder_ticks_left_total = data.data + 1
        else:
            delta = data.data - self.encoder_ticks_left_total
            if delta > 3 or delta < -3:
                delta = 0
            self.encoder_ticks_left_delta_t += max(min(3, delta), -3)
            self.encoder_ticks_left_total = data.data

    def cb_executed_commands(self, data):
        self.vel_right = data.vel_right
        self.vel_left = data.vel_left

    def onShutdown(self):
        super(EncoderLocalizationNode, self).onShutdown()

    def run(self, event=None):
        self.pose = self.db_kinematics.step(self.encoder_ticks_left_delta_t, self.encoder_ticks_right_delta_t, self.pose)
        self.encoder_ticks_right_delta_t = 0
        self.encoder_ticks_left_delta_t = 0
        pose_SE3 = SE3_from_SE2(self.pose)
        rot, trans = rotation_translation_from_SE3(pose_SE3)
        quaternion_tf = quaternion_from_matrix(pose_SE3)
        quaternion = Quaternion(quaternion_tf[0], quaternion_tf[1], quaternion_tf[2], quaternion_tf[3])
        translation = Vector3(trans[0], trans[1], trans[2])
        trafo = Transform(translation, quaternion)
        self.tfs_msg.transform = trafo
        if self.vel_left == 0.0 and self.vel_right == 0.0:
            self.tfs_msg.header.stamp = rospy.Time.now()
        else:
            self.tfs_msg.header.stamp = self.encoder_timestamp
        self.br.sendTransformMessage(self.tfs_msg)
        self.pub_tf_enc_loc.publish(self.tfs_msg)




if __name__ == '__main__':
    # Initialize the node
    encoder_node = EncoderLocalizationNode(node_name='encoder_localization_node')
    timer = rospy.Timer(rospy.Duration(1.0 / 30.0), encoder_node.run)
    rospy.on_shutdown(encoder_node.onShutdown)
    # Keep it spinning to keep the node alive
    rospy.spin()
