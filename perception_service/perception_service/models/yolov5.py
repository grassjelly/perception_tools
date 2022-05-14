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