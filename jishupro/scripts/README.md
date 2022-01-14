### 認識 ###

#### USBカメラ ####
```ls /dev/video```でvideo1があることを確認する。

```
rosrun uvc_camera uvc_camera_node _device:=/dev/video1
```

- PCのカメラの場合```rosrun usb_cam usb_cam_node```

#### Edge TPU ####
```
source /opt/ros/melodic/setup.bash
source ~/coral_ws/devel/setup.bash
roslaunch coral_usb edgetpu_human_pose_estimator.launch INPUT_IMAGE:=/image_raw
```

#### Human Pose Estimator確認 ####
```
rosrun image_view image_view image:=/edgetpu_human_pose_estimator/output/image
```
