# YOLO-in-ROS2
Working Code to use yolo in the ros2 environment.

**Ensure you have Darknet installed and the yolo cfg and weights file downloaded** 
(This Repo utilises Yolo-v3)

Contains two Nodes:
### People_detector
Code used to invoke YOLO within the ros2 environment to detect object, mainly people in this use case however, it works on all objects.

### Webcam
Used to run the people detector algorithim using a regular webcam

## Steps
1. Clone the repo within a ros2/src/ workspace and change the path directories for the weights and label files within peopleDetector.py
2. Build the workspace with **colcon build**
3. Source your workspace i.e export local_setup.bash
4. Run **poeple_dector** with ros2 run
5. Run **webcam_py** in a separate terminal
6. In another terminal, ros2 run rqt_image_view and select the output image topic to view live results.

**Tested on:**\
Open-CV 4.7.0\
Ubuntu 20.04\
ROS2 Foxy


References

@article{yolov3,
  title={YOLOv3: An Incremental Improvement},
  author={Redmon, Joseph and Farhadi, Ali},
  journal = {arXiv},
  year={2018}
}
