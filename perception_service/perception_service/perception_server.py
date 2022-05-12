#!/usr/bin/env python3
# Copyright (c) 2021 Juan Miguel Jimeno
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http:#www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
import threading
import time
import rclpy
from rclpy.qos import qos_profile_sensor_data
from rclpy.node import Node
from rclpy.action import ActionServer
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import TransformStamped, Pose, PoseArray
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import TransformException

from perception_service.numpify.image import image_to_numpy
from perception_service.algorithms.algorithms import pixel_to_pose
from perception_service.geom.geometry import transform_point

from perception_interfaces.srv import ObjectPoses
from perception_service.models.yolov5 import ObjectFinder


class PerceptionServer(Node):
    def __init__(self):
        super().__init__('perception_server')
        self._lock = threading.Lock()
        self.declare_parameters(
            namespace='',
            parameters=[
                ('image_topic', 'camera/color/image_raw'),
                ('depth_topic', 'camera/aligned_depth_to_color/image_raw'),
                ('depth_info_topic', 'camera/aligned_depth_to_color/camera_info'),
                ('base_frame_id', 'camera_link'),
            ]
        )

        self._object_finder = ObjectFinder(
            weights='/home/juan/yolov5/runs/train/astro_boy_det2/weights/last.pt',
            data='/home/juan/yolov5/data/astro_boy.yaml',
            image_size=[640,640],
            conf_thres=0.2,
            iou_thres=0.4
        )

        self._image_topic = self.get_parameter('image_topic').value
        self._depth_topic = self.get_parameter('depth_topic').value
        self._depth_info_topic = self.get_parameter('depth_info_topic').value
  
        self._base_frame = self.get_parameter('base_frame_id').value

        self._depth_image = None
        self._depth_info = None
        self._image = None
        self._camera_tf_rec = False

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._camera_trans_rot = None

        self._depth_info_sub = self.create_subscription(
            CameraInfo,
            self._depth_info_topic,
            self._depth_info_cb,
            qos_profile_sensor_data
        )
        self._depth_info_sub

        self._depth_sub = self.create_subscription(
            Image,
            self._depth_topic,
            self._depth_cb,
            qos_profile_sensor_data
        )
        self._depth_sub

        self._image_sub = self.create_subscription(
            Image,
            self._image_topic,
            self._image_cb,
            qos_profile_sensor_data
        )
        self._image_sub

        self._wait_for_data()
        self._wait_for_transform()

        self._perception_srv = self.create_service(
            ObjectPoses, 
            'object_poses', 
            self._perception_service_callback
        )
        self._request_id = 0
        self.get_logger().info('Perception Server Ready')

    def _fill_pose_msg(self, pos, orient):
        p = Pose()

        p.position.x = pos[0]
        p.position.y = pos[1]
        p.position.z = pos[2]

        p.orientation.x = orient[0]
        p.orientation.y = orient[1]
        p.orientation.z = orient[2]
        p.orientation.w = orient[3]

        return p

    def _wait_for_data(self):
        self.get_logger().info(f'Waiting for image topic {self._image_topic}...')
        while self._image is None:
            rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().info(f'Image topic ok!')

        self.get_logger().info(f'Waiting for depth image topic {self._depth_topic}...')
        while self._depth_image is None:
            rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().info(f'Depth image ok!')

        self.get_logger().info(f'Waiting for depth info topic {self._depth_info_topic}...')
        while self._depth_info is None:
            rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().info(f'Depth info topic ok!')

    def _wait_for_transform(self):
        while not self._camera_tf_rec:
            self.get_logger().info(f'Waiting for camera transform from \'{self._base_frame}\' to \'{self._depth_frame}\'')
            try:
                now = rclpy.time.Time()
                camera_transform = self._tf_buffer.lookup_transform(
                    self._base_frame,
                    self._depth_frame,
                    now
                )

                trans = (
                    camera_transform.transform.translation.x,
                    camera_transform.transform.translation.y,
                    camera_transform.transform.translation.z
                )
                quat = [
                    camera_transform.transform.rotation.x,
                    camera_transform.transform.rotation.y,
                    camera_transform.transform.rotation.z,
                    camera_transform.transform.rotation.w
                ]

                self._camera_trans_rot = (trans, quat)
                self._camera_tf_rec = True
                return

            except TransformException as ex:
                pass
     
            rclpy.spin_once(self, timeout_sec=1.0)

    def _depth_cb(self, msg):
        self._depth_image = image_to_numpy(msg)

    def _image_cb(self, msg):
        self._image = image_to_numpy(msg) 

    def _depth_info_cb(self, msg):
        self._depth_info = msg
        self._depth_constant = 1.0 / msg.k[4]
        self._depth_frame = msg.header.frame_id

    def _get_object_pose(self, target_objects, image, depth_image):
        classes, centroids, confidence = self._object_finder.find_objects(image)

        classes_f = []
        centroids_f = []
        confidence_f = []
        pose_array = PoseArray()
        pose_array.header.frame_id = self._base_frame
        pose_array.header.stamp = self.get_clock().now().to_msg()

        for i, obj_class in enumerate(classes):
            if obj_class in target_objects:
                classes_f.append(obj_class)
                centroids_f.append(centroids[i])
                confidence_f.append(confidence[i])

        for i, centroid in enumerate(centroids_f):
            object_pos, quat = pixel_to_pose(
                depth_image,
                centroid,
                (10, 10),
                self._depth_constant,
                True
            )

            if object_pos is None:
                continue

            transformed_object_pos = transform_point(self._camera_trans_rot, object_pos)

            pose_msg = self._fill_pose_msg(
                transformed_object_pos,
                quat
            )

            pose_array.poses.append(pose_msg)

            self.get_logger().info(f'REQUEST ID: {self._request_id} | {classes_f[i]} object found!')

        return classes_f, pose_array

    def _perception_service_callback(self, request, response):
        target_objects = request.objects
        self.get_logger().info(f'REQUEST ID: {self._request_id} | Request Received to find {target_objects}.')

        classes, object_poses = self._get_object_pose(
            target_objects, 
            self._image, 
            self._depth_image
        )

        response.objects = classes
        response.poses = object_poses

        self._request_id += 1
        return response


def main(args=None):
    rclpy.init(args=args)
    ofs = PerceptionServer()
    rclpy.spin(ofs)
    calibrator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()



