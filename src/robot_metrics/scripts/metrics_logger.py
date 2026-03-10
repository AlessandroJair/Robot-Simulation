#!/usr/bin/env python3

import rospy
import csv
import os
import math
import numpy as np
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion
import tf2_ros

# Try to import gazebo messages for ground truth
try:
    from gazebo_msgs.msg import ModelStates
    HAS_GAZEBO_MSGS = True
except ImportError:
    HAS_GAZEBO_MSGS = False

# Try to import rtabmap messages (optional)
try:
    from rtabmap_msgs.msg import Info as RtabmapInfo
    from rtabmap_msgs.msg import OdomInfo
    HAS_RTABMAP_MSGS = True
except ImportError:
    try:
        from rtabmap_ros.msg import Info as RtabmapInfo
        from rtabmap_ros.msg import OdomInfo
        HAS_RTABMAP_MSGS = True
    except ImportError:
        HAS_RTABMAP_MSGS = False
        rospy.logwarn("rtabmap messages not found. SLAM metrics will be limited to TF-based.")


class MetricsLogger:
    def __init__(self):
        rospy.init_node('metrics_logger', anonymous=False)

        # Parameters
        self.robot_name = rospy.get_param('~robot_name', 'robot')
        cmd_vel_topic = rospy.get_param('~cmd_vel_topic', '/cmd_vel')
        self.output_dir = rospy.get_param('~output_dir', os.path.expanduser('~/metrics_output'))
        self.model_name = rospy.get_param('~model_name', self.robot_name)

        # Create output directory
        if not os.path.exists(self.output_dir):
            os.makedirs(self.output_dir)

        # Data storage
        self.data_rows = []

        # Latest values (updated by callbacks)
        # -- Locomotion metrics --
        self.pitch_rad = 0.0
        self.roll_rad = 0.0
        self.yaw_rad = 0.0
        self.accel_x = 0.0
        self.accel_y = 0.0
        self.accel_z = 0.0
        self.angular_vel_x = 0.0
        self.angular_vel_y = 0.0
        self.angular_vel_z = 0.0
        # Angular acceleration (numerical derivative of angular velocity)
        self.angular_accel_x = 0.0
        self.angular_accel_z = 0.0
        self._prev_angular_vel_x = None
        self._prev_angular_vel_z = None
        self._prev_imu_time = None
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_z = 0.0
        self.odom_vx = 0.0
        self.odom_vyaw = 0.0
        self.cmd_vx = 0.0
        self.cmd_vyaw = 0.0

        # -- Ground truth from Gazebo --
        self.gt_x = 0.0
        self.gt_y = 0.0
        self.gt_z = 0.0
        self.gt_roll = 0.0
        self.gt_pitch = 0.0
        self.gt_yaw = 0.0

        # -- SLAM metrics --
        self.loop_closure_count = 0
        self.proximity_detection_count = 0
        self.slam_inliers = 0
        self.slam_matches = 0
        self.slam_icpRMS = 0.0
        self.slam_icpCorrespondences = 0
        self.slam_processing_time = 0.0
        # TF map->odom correction tracking
        self.tf_correction_x = 0.0
        self.tf_correction_y = 0.0
        self.tf_correction_yaw = 0.0
        self.prev_tf_x = None
        self.prev_tf_y = None
        self.prev_tf_yaw = None

        # TF listener for map->odom corrections
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Subscribers - Locomotion
        rospy.Subscriber('/imu/data', Imu, self.imu_callback)
        rospy.Subscriber('/rtabmap/odom', Odometry, self.odom_callback)
        rospy.Subscriber(cmd_vel_topic, Twist, self.cmd_vel_callback)

        # Subscribers - Ground truth
        if HAS_GAZEBO_MSGS:
            rospy.Subscriber('/gazebo/model_states', ModelStates, self.model_states_callback)
            rospy.loginfo("  Ground truth topic: /gazebo/model_states (model: %s)", self.model_name)
        else:
            rospy.logwarn("gazebo_msgs not found. Ground truth (RPE/ATE) will not be available.")

        # Subscribers - SLAM
        if HAS_RTABMAP_MSGS:
            rospy.Subscriber('/rtabmap/info', RtabmapInfo, self.rtabmap_info_callback)
            rospy.Subscriber('/rtabmap/odom_info', OdomInfo, self.odom_info_callback)
            rospy.loginfo("  SLAM topics: /rtabmap/info, /rtabmap/odom_info")

        # Register shutdown hook to save CSV
        rospy.on_shutdown(self.save_csv)

        # Logging rate
        self.rate = rospy.Rate(50)  # 50 Hz

        rospy.loginfo("MetricsLogger started for robot: %s", self.robot_name)
        rospy.loginfo("  IMU topic: /imu/data")
        rospy.loginfo("  Odom topic: /rtabmap/odom")
        rospy.loginfo("  CmdVel topic: %s", cmd_vel_topic)
        rospy.loginfo("  TF tracking: map -> odom")
        rospy.loginfo("  Output dir: %s", self.output_dir)

    # ---- Locomotion callbacks ----

    def imu_callback(self, msg):
        q = msg.orientation
        roll, pitch, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.pitch_rad = pitch
        self.roll_rad = roll
        self.yaw_rad = yaw

        self.accel_x = msg.linear_acceleration.x
        self.accel_y = msg.linear_acceleration.y
        self.accel_z = msg.linear_acceleration.z

        # Compute angular acceleration (numerical derivative)
        current_time = msg.header.stamp.to_sec()
        wx = msg.angular_velocity.x
        wz = msg.angular_velocity.z
        if self._prev_imu_time is not None:
            dt = current_time - self._prev_imu_time
            if dt > 0:
                self.angular_accel_x = (wx - self._prev_angular_vel_x) / dt
                self.angular_accel_z = (wz - self._prev_angular_vel_z) / dt
        self._prev_angular_vel_x = wx
        self._prev_angular_vel_z = wz
        self._prev_imu_time = current_time

        self.angular_vel_x = msg.angular_velocity.x
        self.angular_vel_y = msg.angular_velocity.y
        self.angular_vel_z = msg.angular_velocity.z

    def odom_callback(self, msg):
        self.odom_x = msg.pose.pose.position.x
        self.odom_y = msg.pose.pose.position.y
        self.odom_z = msg.pose.pose.position.z
        self.odom_vx = msg.twist.twist.linear.x
        self.odom_vyaw = msg.twist.twist.angular.z

    def cmd_vel_callback(self, msg):
        self.cmd_vx = msg.linear.x
        self.cmd_vyaw = msg.angular.z

    def model_states_callback(self, msg):
        try:
            idx = msg.name.index(self.model_name)
        except ValueError:
            return
        pose = msg.pose[idx]
        self.gt_x = pose.position.x
        self.gt_y = pose.position.y
        self.gt_z = pose.position.z
        q = pose.orientation
        roll, pitch, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.gt_roll = roll
        self.gt_pitch = pitch
        self.gt_yaw = yaw

    # ---- SLAM callbacks ----

    def rtabmap_info_callback(self, msg):
        if msg.loopClosureId > 0:
            self.loop_closure_count += 1
        if msg.proximityDetectionId > 0:
            self.proximity_detection_count += 1

    def odom_info_callback(self, msg):
        self.slam_inliers = msg.inliers
        self.slam_matches = msg.matches
        self.slam_icpRMS = msg.icpInliersRatio
        self.slam_icpCorrespondences = msg.icpCorrespondences
        self.slam_processing_time = msg.timeEstimation

    # ---- TF correction tracking ----

    def update_tf_correction(self):
        try:
            trans = self.tf_buffer.lookup_transform('map', 'odom', rospy.Time(0),
                                                     rospy.Duration(0.05))
            tx = trans.transform.translation.x
            ty = trans.transform.translation.y
            q = trans.transform.rotation
            _, _, tyaw = euler_from_quaternion([q.x, q.y, q.z, q.w])

            if self.prev_tf_x is not None:
                self.tf_correction_x = tx - self.prev_tf_x
                self.tf_correction_y = ty - self.prev_tf_y
                self.tf_correction_yaw = tyaw - self.prev_tf_yaw

            self.prev_tf_x = tx
            self.prev_tf_y = ty
            self.prev_tf_yaw = tyaw
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            pass

    # ---- Main loop ----

    def run(self):
        while not rospy.is_shutdown():
            self.update_tf_correction()

            timestamp = rospy.get_time()
            tf_correction_mag = math.sqrt(
                self.tf_correction_x**2 + self.tf_correction_y**2)

            row = [
                timestamp,
                # Locomotion
                self.pitch_rad,
                self.roll_rad,
                self.yaw_rad,
                self.accel_x,
                self.accel_y,
                self.accel_z,
                self.angular_vel_x,
                self.angular_vel_y,
                self.angular_vel_z,
                self.angular_accel_x,
                self.angular_accel_z,
                self.odom_x,
                self.odom_y,
                self.odom_z,
                self.odom_vx,
                self.odom_vyaw,
                self.cmd_vx,
                self.cmd_vyaw,
                # Ground truth
                self.gt_x,
                self.gt_y,
                self.gt_z,
                self.gt_roll,
                self.gt_pitch,
                self.gt_yaw,
                # SLAM
                self.loop_closure_count,
                self.proximity_detection_count,
                self.slam_inliers,
                self.slam_matches,
                self.slam_icpRMS,
                self.slam_icpCorrespondences,
                self.slam_processing_time,
                tf_correction_mag,
                self.tf_correction_yaw,
            ]
            self.data_rows.append(row)
            self.rate.sleep()

    def save_csv(self):
        if not self.data_rows:
            rospy.logwarn("No data collected, skipping CSV save.")
            return

        filepath = os.path.join(self.output_dir, '{}.csv'.format(self.robot_name))
        header = [
            'timestamp',
            # Locomotion
            'pitch_rad', 'roll_rad', 'yaw_rad',
            'accel_x', 'accel_y', 'accel_z',
            'angular_vel_x', 'angular_vel_y', 'angular_vel_z',
            'angular_accel_x', 'angular_accel_z',
            'odom_x', 'odom_y', 'odom_z',
            'odom_vx', 'odom_vyaw',
            'cmd_vx', 'cmd_vyaw',
            # Ground truth
            'gt_x', 'gt_y', 'gt_z',
            'gt_roll', 'gt_pitch', 'gt_yaw',
            # SLAM
            'loop_closure_count', 'proximity_detection_count',
            'slam_inliers', 'slam_matches',
            'slam_icp_inliers_ratio', 'slam_icp_correspondences',
            'slam_processing_time_s',
            'tf_correction_magnitude', 'tf_correction_yaw',
        ]

        with open(filepath, 'w') as f:
            writer = csv.writer(f)
            writer.writerow(header)
            writer.writerows(self.data_rows)

        rospy.loginfo("Saved %d rows to %s", len(self.data_rows), filepath)


if __name__ == '__main__':
    try:
        logger = MetricsLogger()
        logger.run()
    except rospy.ROSInterruptException:
        pass
