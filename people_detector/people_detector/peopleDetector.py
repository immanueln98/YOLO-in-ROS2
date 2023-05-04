import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import time
import numpy as np

class ObjectDetector(Node):
    def __init__(self):
        super().__init__('peopleDetector')
        self.subscription = self.create_subscription(Image, '/image/bgr', self.detect_objects, 10)
        self.publisher = self.create_publisher(Image, 'output_image', 10)
        self.net = cv2.dnn.readNet("/home/imanuel/yolov3.weights", "/home/imanuel/darknet/cfg/yolov3.cfg")
        with open("/home/imanuel/darknet/data/coco.names", "r") as f:
            self.classes = [line.strip() for line in f.readlines()]
        layer_names = self.net.getLayerNames()
        self.output_layers = [layer_names[i - 1] for i in self.net.getUnconnectedOutLayers()]
        self.bridge = CvBridge()

    def detect_objects(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        height, width, channels = img.shape
        blob = cv2.dnn.blobFromImage(img, 1/255.0, (416, 416), (0, 0, 0), True, crop=False)
        self.net.setInput(blob)
        start = time.time()
        outs = self.net.forward(self.output_layers)
        end = time.time()
        print("[INFO] YOLO took {:.6f} seconds".format(end - start))
        class_ids = []
        confidences = []
        boxes = []
        class_id = None 
        for out in outs:
            for detection in out:
                scores = detection[5:]
                class_id = np.argmax(scores)
                confidence = scores[class_id]
                if confidence > 0.5:
                    center_x = int(detection[0] * width)
                    center_y = int(detection[1] * height)
                    w = int(detection[2] * width)
                    h = int(detection[3] * height)
                    x = int(center_x - w / 2)
                    y = int(center_y - h / 2)
                    boxes.append([x, y, w, h])
                    confidences.append(float(confidence))
                    class_ids.append(class_id)
        indexes = cv2.dnn.NMSBoxes(boxes, confidences, 0.5, 0.4)
        font = cv2.FONT_HERSHEY_PLAIN
        for i in range(len(boxes)):
            if i in indexes:
                x, y, w, h = boxes[i]
                label = str(self.classes[class_ids[i]])
                confidence = confidences[i]
                color = (0, 255, 0)
                cv2.rectangle(img, (x, y), (x + w, y + h), color, 2)
                cv2.putText(img, label + " " + str(round(confidence, 2)), (x, y + 30), font, 3, (0, 255, 0), 3)
        output_msg = self.bridge.cv2_to_imgmsg(img, encoding="bgr8")
        self.publisher.publish(output_msg)

def main(args=None):
    rclpy.init(args=args)
    object_detector = ObjectDetector()
    rclpy.spin(object_detector)
    object_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
