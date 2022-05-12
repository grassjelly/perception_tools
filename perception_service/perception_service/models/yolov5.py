from yolov5_ros.inference import Yolov5


class ObjectFinder:
    def __init__(self, **kwargs):
        self._model = Yolov5(
            weights=kwargs['weights'],
            data=kwargs['data'],
            imgsz=kwargs['image_size'],
            conf_thres=kwargs['conf_thres'],
            iou_thres=kwargs['iou_thres']
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