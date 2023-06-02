#!/usr/bin/env python3

from ultralytics import YOLO
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from new_yolo.msg import InferenceResult
from new_yolo.msg import Yolov8Inference

bridge = CvBridge()

class Camera_subscriber(Node):

    def __init__(self, model):
        super().__init__('camera_subscriber')
        self.model = model
        self.yolov8_inference = Yolov8Inference()
        self.subscription = self.create_subscription(
            Image,
            '/image/bgr',
            self.camera_callback,
            10)
        self.subscription 

        self.yolov8_pub = self.create_publisher(Yolov8Inference, "/Yolov8_Inference", 1)
        self.img_pub = self.create_publisher(Image, "/inference_result", 1)
        self.inference_result = InferenceResult()
        
    def camera_callback(self, data):

        img = bridge.imgmsg_to_cv2(data, "bgr8")
        results = self.model(img, conf = 0.5)

        self.yolov8_inference.header.frame_id = "inference"
        self.yolov8_inference.header.stamp = self.get_clock().now().to_msg()

        self.inference_result = InferenceResult()

        # Reserve memory for yolov8_inference
        inference_list = []
        #self.yolov8_inference.yolov8_inference.reserve(len(results) * 5)

        for r in results:
            boxes = r.boxes
            for box in boxes:
                
                b = box.xyxy[0].to('cpu').detach().numpy().copy()  # get box coordinates in (top, left, bottom, right) format
                c = box.cls
                self.inference_result.class_name = self.model.names[int(c)]
                self.inference_result.top = int(b[0])
                self.inference_result.left = int(b[1])
                self.inference_result.bottom = int(b[2])
                self.inference_result.right = int(b[3])
                inference_list.append(self.inference_result)
                #self.yolov8_inference.yolov8_inference.append(self.inference_result)

            #camera_subscriber.get_logger().info(f"{self.yolov8_inference}")
        self.yolov8_inference.yolov8_inference = inference_list
        annotated_frame = results[0].plot()

        img_msg = bridge.cv2_to_imgmsg(annotated_frame, encoding = 'rgb8')  

        #self.img_pub.publish(img_msg)
        #self.yolov8_pub.publish(self.yolov8_inference)
        self.yolov8_inference.yolov8_inference.clear()

if __name__ == '__main__':
    rclpy.init(args=None)

    #Load yolo model
    model = YOLO('~/home/imanuel/ros2_ws/src/new_yolo/scripts/yolov8n.pt')
    camera_subscriber = Camera_subscriber(model)

    rclpy.spin(camera_subscriber)
    rclpy.shutdown()
