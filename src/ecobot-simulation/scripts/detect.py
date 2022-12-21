#!/usr/bin/env python3.8
from typing import List
import math
import yaml
import cv2
import numpy as np
from absl import app

import rospy
from sensor_msgs.msg import Image
from rospy_tutorials.msg import Floats
from cv_bridge import CvBridge

import tensorflow as tf


rospy.init_node('search_eco_robot_detect_node', log_level=rospy.DEBUG)
with open(rospy.get_param('~config_path'), "r") as f:
    config = yaml.safe_load(f)

detection_model_img_size = config['model_params']['detection_model_img_size']
detection_thr = config['model_params']['detection_thr']
cam_frame_origin_x = config['camera_params']['cam_frame_origin_x']
cam_frame_origin_y = config['camera_params']['cam_frame_origin_y']
camera_depth = config['camera_params']['cam_depth']
focal_length = config['camera_params']['cam_focal_length']
ox = config['camera_params']['ox']
oy = config['camera_params']['oy']
table_velocity = config['table_params']['table_velocity']


class RosTensorFlow:
    def __init__(
        self,
        model,
        in_image_topic: str,
        out_detection_topic: str
    ):
        self._cv_bridge = CvBridge()
        self.model = model

        rospy.Subscriber(
            in_image_topic,
            Image,
            self.callback,
            queue_size=1,
            buff_size=2**25
        )
        self.pub_coordinates = rospy.Publisher(
            out_detection_topic,
            Floats,
            queue_size=1
        )

    def preprocess_image(self, image: np.ndarray) -> tf.Tensor:
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB).astype(np.float32)
        image = tf.expand_dims(image, axis=0)
        input_tensor = tf.convert_to_tensor(image, dtype=tf.float32)
        input_tensor = tf.image.resize(
            input_tensor,
            (detection_model_img_size, detection_model_img_size)
        )
        return input_tensor

    def detect(self, input_tensor: tf.Tensor) -> np.ndarray:
        self.model.allocate_tensors()
        input_details = self.model.get_input_details()
        output_details = self.model.get_output_details()

        self.model.set_tensor(input_details[0]['index'], input_tensor.numpy())
        self.model.invoke()

        scores = self.model.get_tensor(output_details[0]['index'])
        boxes = self.model.get_tensor(output_details[1]['index'])
        return boxes[scores > detection_thr]

    def from_bbox_to_coordinates(
        self,
        target_bboxes: np.ndarray,
        image_height: int,
        image_width: int
    ) -> List[float]:
        y1, x1, y2, x2 = target_bboxes
        x1 = int(x1 * image_width)
        y1 = int(y1 * image_height)
        x2 = int(x2 * image_width)
        y2 = int(y2 * image_height)
        bbox_center_x = image_height - (y1 + (y2 - y1) / 2)
        bbox_center_y = image_width - (x1 + (x2 - x1) / 2)

        x = cam_frame_origin_x + (bbox_center_x - ox) * camera_depth / focal_length
        y = cam_frame_origin_y + (bbox_center_y - oy) * camera_depth / focal_length
        return [x, y]

    def get_point_trajectory(
        self,
        coordinates: tuple,
        shot_time: float
    ) -> List[float]:
        reference = [0, cam_frame_origin_y]
        point = np.array(coordinates) - reference
        x, y = point[1], -point[0]

        theta_rad = math.atan(y / x)  # current angle in rads
        distance_to_target = theta_rad if theta_rad > 0 else math.pi + theta_rad

        R = np.array([
            [math.sin(distance_to_target), math.cos(distance_to_target)],
            [math.cos(distance_to_target), -math.sin(distance_to_target)]
        ])

        rotated_point = np.array([y, x]).dot(R)
        target_y = rotated_point[0] + cam_frame_origin_y

        duration_to_target = distance_to_target / abs(table_velocity)  # in seconds
        target_time = shot_time + duration_to_target
        return [target_y, target_time]

    def callback(self, image_msg: Image) -> None:
        shot_time = rospy.Time.now().to_sec()

        image_np = self._cv_bridge.imgmsg_to_cv2(image_msg, "bgr8")
        image_height, image_width, _ = image_np.shape
        input_tensor = self.preprocess_image(image_np)
        detections = self.detect(input_tensor)

        if not detections.tolist():
            return

        target_bbox = detections[0]
        coordinates = self.from_bbox_to_coordinates(
            target_bbox,
            image_height,
            image_width
        )
        target_y, target_time = self.get_point_trajectory(coordinates, shot_time)
        self.pub_coordinates.publish([target_y, target_time])

    def main(self):
        rospy.spin()


def main_action(_argv):
    model = tf.lite.Interpreter(rospy.get_param('~model_path'))

    actor = RosTensorFlow(
        model=model,
        in_image_topic=rospy.get_param('~in_image_topic'),
        out_detection_topic=rospy.get_param('~out_detection_topic'),
    )
    actor.main()


if __name__ == '__main__':
    app.run(main_action)
