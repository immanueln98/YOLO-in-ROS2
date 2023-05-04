# YOLO-in-ROS2
Working Code to use yolo in the ros2 environment.

Contains two Nodes:
### People_detector
Code used to invoke YOLO within the ros2 environment to detect object, mainly people in this use case

### Webcam
Used to run the people detector algorithim using a regular webcam

## Steps
1. Clone the repo within a ros2/src/ workspace.
2. Build the workspace with **colcon build**
3. Run **poeple_dector** with ros2 run
4. Run **webcam_py** in a separate terminal
5. In another terminal, ros2 run rqt_image_view and select the output image topic to view live results.
