# lidar_cam_calibration
器材:

1. 一般usb cam(單眼)
2. Velodyne VLP-16

將要使用的 submodule update
```
git submodule update --init --recursive
```
流程:

1.  usb cam 校正後參數(轉換矩陣等)
2. arcuo mapping
3. arcuo mapping with lidar

單眼相機校正流程

[http://wiki.ros.org/camera_calibration/Tutorials/MonocularCalibration](http://wiki.ros.org/camera_calibration/Tutorials/MonocularCalibration)

校正用棋盤格

[check-108.pdf](https://s3-us-west-2.amazonaws.com/secure.notion-static.com/f4b1e795-1852-4e34-8372-49a9ca61bae4/check-108.pdf)

Arcuo 產生器 [https://chev.me/arucogen/](https://chev.me/arucogen/)

Dictionary要選擇original

範例使用的是ID 26跟582

![https://s3-us-west-2.amazonaws.com/secure.notion-static.com/062fde84-1652-40ed-9241-d444d2fcf9bf/aruco-582.svg](https://s3-us-west-2.amazonaws.com/secure.notion-static.com/062fde84-1652-40ed-9241-d444d2fcf9bf/aruco-582.svg)

校正結果如下：

![https://s3-us-west-2.amazonaws.com/secure.notion-static.com/892cc17d-e8e3-4680-8f8c-b7fdafcd4a3d/Screenshot_2021-08-11_213319.png](https://s3-us-west-2.amazonaws.com/secure.notion-static.com/892cc17d-e8e3-4680-8f8c-b7fdafcd4a3d/Screenshot_2021-08-11_213319.png)

使用arcuo mapping 確認是否能mapping到arcuo

```jsx
<node pkg="aruco_mapping" type="aruco_mapping" name="aruco_mapping" output="screen">
    <remap from="/image_raw" to="/usb_cam/image_raw"/>

    <param name="calibration_file" type="string" value="$(find aruco_mapping)/data/usb_cam.ini" /> 
    <param name="num_of_markers" type="int" value="2" />
    <param name="marker_size" type="double" value="0.016"/>
    <param name="space_type" type="string" value="plane" />
    <param name="roi_allowed" type="bool" value="false" />

  </node>
```

將校正後的結果,如範例的usb_cam.ini在裡面更改資料

```jsx
# Prosilica camera intrinsics

[image]

width
640

height
480

[camera]

camera matrix
727.079910 0.000000 317.463020
0.000000 734.695266 240.018079
0.000000 0.000000 1.000000

distortion
0.078355 -0.382089 0.001584 0.000016 0.000000

rectification
1.000000 0.000000 0.000000
0.000000 1.000000 0.000000
0.000000 0.000000 1.000000

projection
727.703247 0.000000 317.400100 0.000000
0.000000 737.639832 240.387843 0.000000
0.000000 0.000000 1.000000 0.000000
```

roslaunch velodyne_pointcloud VLP16_points.launch

roslaunch usb_cam usb_cam-test.launch

roslaunch lidar_camera_calibration find_transform.launch
