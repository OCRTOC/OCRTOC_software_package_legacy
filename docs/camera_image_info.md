| topic | frame_id | height*width | encoding | step | camera_info topic | description |
| :--- | :--- | :---: | :---: | :---: | :--- | :--- |
| /kinect/color/image_raw | rgb_camera_link | 720*1280 | bgra8 | 5120 | /kinect/color/camera_info | The raw image from the color camera. |
| /kinect/color/image_rect_color | rgb_camera_link | 720*1280 | bgra8 | 5120 | /kinect/color/camera_info | The color image, rectified. |
| /kinect/color_to_depth/image_raw | depth_camera_link | 576*640 | bgra8 | 2560 | /kinect/color_to_depth/camera_info | The color image, transformed into the depth camera co-ordinate space. |
| /kinect/depth/image_raw | depth_camera_link | 576*640 | 32FC1 | 2560 | /kinect/depth/camera_info | The raw image from the depth camera. |
| /kinect/depth/image_rect | depth_camera_link | 576*640 | 32FC1 | 2560 | /kinect/depth/camera_info | The depth image, rectified. |
| /kinect/depth_to_color/image_raw | rgb_camera_link | 720*1280 | 32FC1 | 5120 | /kinect/depth_to_rgb/camera_info | The depth image, transformed into the color camera co-ordinate space. |
| /kinect/ir/image_raw | depth_camera_link | 576*640 | mono16 | 1280 | /kinect/ir/camera_info | The raw infrared image from the depth camera sensor. |
| /kinect/ir/image_rect | depth_camera_link | 576*640 | mono16 | 1280 | /kinect/ir/camera_info | The infrared image, rectified. |
| /realsense/color/image_raw | realsense_color_optical_frame | 480*640 | rgb8 | 1920 | /realsense/color/camera_info | The raw image from the color camera. |
| /realsense/depth/image_raw | realsense_depth_optical_frame | 480*640 | 16UC1 | 1280 | /realsense/depth/camera_info | The raw image from the depth camera. |
| /realsense/aligned_depth_to_color/image_raw | realsense_color_optical_frame | 480*640 | 16UC1 | 1280 | /realsense/aligned_depth_to_color/camera_info | The depth image, transformed into the color camera co-ordinate space. |
| /realsense/aligned_depth_to_infra1/image_raw | realsense_infra1_optical_frame | 480*640 | 16UC1 | 1280 | /realsense/aligned_depth_to_infra1/camera_info | The depth image, transformed into the ir1 camera co-ordinate space. |
| /realsense/infra1/image_raw | realsense_infra1_optical_frame | 480*640 | mono8 | 640 | /realsense/infra1/camera_info | The raw infrared image from the ir camera sensor1. |
| /realsense/infra2/image_raw | realsense_infra2_optical_frame | 480*640 | mono8 | 640 | /realsense/infra2/camera_info | The raw infrared image from the ir camera sensor2. |
