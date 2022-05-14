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
from yolov5_ros.inference import Yolov5
import yaml


class ObjectFinder:
    def __init__(self, yaml_config):
        with open(yaml_config, "r") as stream:
            try:
                config = yaml.safe_load(stream)
            except yaml.YAMLError as exc:
                print(exc)

        weights = config['yolov5']['weights']
        data = config['yolov5']['data']
        imgsz = config['yolov5']['image_size']
        conf_thres = config['yolov5']['confidence_threshold']
        iou_thres = config['yolov5']['iou_threshold']
        device = config['yolov5']['gpu_id']

        self._model = Yolov5(
            weights=weights,
            data=data,
            imgsz=imgsz,
            conf_thres=conf_thres,
            iou_thres=iou_thres, 
            device=device
        )

    def find_objects(self, img):
        classes, bboxes, confidence = self._model.predict(img)

        centroids =[]
        for bbox in bboxes:
            min_x, min_y = bbox[0]
            max_x, max_y = bbox[1]
            center_x = (min_x + max_x) / 2.
            center_y = (min_y + max_y) / 2.
            centroids.append((center_x, center_y))

        return classes, centroids, confidence