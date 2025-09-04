import cv2
import yaml
import numpy as np
from typing import Tuple, Union
from tflite_runtime.interpreter import Interpreter
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class YOLOTFLite:
    def __init__(self, model: str, conf: float = 0.25, iou: float = 0.45, metadata: Union[str, None] = None):
        self.conf = conf
        self.iou = iou

        # Load class names
        if metadata is None:
            self.classes = {i: str(i) for i in range(1000)}
        else:
            with open(metadata) as f:
                self.classes = yaml.safe_load(f)["names"]

        np.random.seed(42)
        self.color_palette = np.random.uniform(128, 255, size=(len(self.classes), 3))

        # Load TFLite model
        self.model = Interpreter(model_path=model)
        self.model.allocate_tensors()

        input_details = self.model.get_input_details()[0]
        self.in_height, self.in_width = input_details["shape"][1:3]
        self.in_index = input_details["index"]
        self.in_scale, self.in_zero_point = input_details["quantization"]
        self.int8 = input_details["dtype"] == np.int8

        output_details = self.model.get_output_details()[0]
        self.out_index = output_details["index"]
        self.out_scale, self.out_zero_point = output_details["quantization"]

    def letterbox(self, img: np.ndarray, new_shape=(640, 640)):
        """Resize and pad to target shape while keeping aspect ratio."""
        shape = img.shape[:2]  # current shape
        r = min(new_shape[0] / shape[0], new_shape[1] / shape[1])
        new_unpad = int(round(shape[1] * r)), int(round(shape[0] * r))
        dw, dh = (new_shape[1] - new_unpad[0]) / 2, (new_shape[0] - new_unpad[1]) / 2

        if shape[::-1] != new_unpad:
            img = cv2.resize(img, new_unpad, interpolation=cv2.INTER_LINEAR)

        top, bottom = int(round(dh - 0.1)), int(round(dh + 0.1))
        left, right = int(round(dw - 0.1)), int(round(dw + 0.1))
        img = cv2.copyMakeBorder(img, top, bottom, left, right, cv2.BORDER_CONSTANT, value=(114, 114, 114))
        return img, (top / img.shape[0], left / img.shape[1])

    def preprocess(self, img: np.ndarray):
        img, pad = self.letterbox(img, (self.in_width, self.in_height))
        img = img[..., ::-1][None]  # BGR→RGB, add batch
        img = np.ascontiguousarray(img).astype(np.float32) / 255.0
        return img, pad

    def draw_detections(self, img, box, score, class_id):
        x1, y1, w, h = box
        color = self.color_palette[class_id]
        cv2.rectangle(img, (int(x1), int(y1)), (int(x1 + w), int(y1 + h)), color, 2)

        label = f"{self.classes.get(class_id, class_id)}: {score:.2f}"
        (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
        cv2.rectangle(img, (int(x1), int(y1 - th - 4)), (int(x1 + tw), int(y1)), color, -1)
        cv2.putText(img, label, (int(x1), int(y1 - 2)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)

    def detect(self, img: np.ndarray):
        # Preprocess
        x, pad = self.preprocess(img)
        if self.int8:
            x = (x / self.in_scale + self.in_zero_point).astype(np.int8)

        # Inference
        self.model.set_tensor(self.in_index, x)
        self.model.invoke()
        outputs = self.model.get_tensor(self.out_index)

        if self.int8:
            outputs = (outputs.astype(np.float32) - self.out_zero_point) * self.out_scale

        # Postprocess
        outputs[:, 0] -= pad[1]
        outputs[:, 1] -= pad[0]
        outputs[:, :4] *= max(img.shape)
        outputs = outputs.transpose(0, 2, 1)
        outputs[..., 0] -= outputs[..., 2] / 2
        outputs[..., 1] -= outputs[..., 3] / 2

        detections, counts = [], {}
        for out in outputs:
            boxes = out[:, :4]
            scores = out[:, 4:]

            cand_boxes, cand_scores, cand_ids = [], [], []
            for i, box in enumerate(boxes):
                class_id = int(np.argmax(scores[i]))
                score = scores[i][class_id]
                if score >= self.conf:
                    cand_boxes.append(box)
                    cand_scores.append(float(score))
                    cand_ids.append(class_id)

            if not cand_boxes:
                continue

            keep = cv2.dnn.NMSBoxes([b.tolist() for b in cand_boxes], cand_scores, self.conf, self.iou)
            if keep is None or len(keep) == 0:
                continue

            for k in np.array(keep).flatten():
                x, y, w, h = cand_boxes[k]
                class_id = cand_ids[k]
                score = cand_scores[k]
                self.draw_detections(img, (x, y, w, h), score, class_id)
                label = self.classes.get(class_id, str(class_id))
                counts[label] = counts.get(label, 0) + 1
                detections.append({"bbox": (int(x), int(y), int(w), int(h)),
                                   "class_id": class_id,
                                   "score": score,
                                   "label": label})
        return counts, img, detections
    
CAM_TOPICS = ["X3_1/camera/image_raw", "X3_2/camera/image_raw", "X3_3/camera/image_raw"]
PUB_TOPICS = ["X3_1/obj_recognition", "X3_2/obj_recognition", "X3_3/obj_recognition"]

class YOLONode(Node):
    def __init__(self):
        super().__init__('yolo_tflite_node')

        # Load detector
        self.detector = YOLOTFLite(
            "/home/manohara/ros_ws/src/military_drones/resources/yolo11n_float32.tflite",
            metadata="/home/manohara/ros_ws/src/military_drones/resources/coco.yaml"
        )

        self.bridge = CvBridge()

        # Create subscribers and publishers
        self.subs = []
        self.pubs = []
        for cam_topic, pub_topic in zip(CAM_TOPICS, PUB_TOPICS):
            self.subs.append(self.create_subscription(
                Image, cam_topic, lambda msg, t=pub_topic: self.callback(msg, t), 10
            ))
            self.pubs.append(self.create_publisher(Image, pub_topic, 10))

        self.get_logger().info("YOLO TFLite node initialized and running.")

    def callback(self, msg, pub_topic):
        # Convert ROS Image to OpenCV
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Run detection
        counts, annotated, detections = self.detector.detect(cv_image)

        # Convert back to ROS Image and publish
        out_msg = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
        for pub in self.pubs:
            if pub.topic_name == pub_topic:
                pub.publish(out_msg)
                break

        self.get_logger().info(f"Published annotated image on {pub_topic} with counts: {counts}")


def main(args=None):
    rclpy.init(args=args)
    node = YOLONode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()