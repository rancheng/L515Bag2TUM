# Realsense数据录制

启动realsense node

D455 launchfile `rs_d400_dmvio.launch`:

```xml
<launch>
<include file="$(find realsense2_camera)/launch/rs_camera.launch">  
   <arg name="enable_gyro" value="true" />  
   <arg name="enable_accel" value="true" />  
   <arg name="unite_imu_method" value="linear_interpolation" />  
   <arg name="color_width" value="640" />  
   <arg name="color_height" value="480" />  
   <arg name="color_fps" value="30" />  
   <arg name="enable_infra1" value="true" />  
   <arg name="enable_infra2" value="true" />  
   <arg name="infra_width" value="640" />  
   <arg name="infra_height" value="480" />  
   <arg name="infra_fps" value="30" />  
</include>  
<rosparam> /camera/stereo_module/emitter_enabled: 2</rosparam>
</launch>
```

run D455/D435i:

```shell
sudo cp rs_d455_dmvio.launch /opt/ros/melodic/share/realsense2_camera/launch/
source /opt/ros/melodic/setup.bash
roslaunch realsense2_camera rs_d400_dmvio.launch
```


#### 1.运行realsense_ros，录制话题


/camera/imu

/camera/infra1/camera_info

/camera/infra1/image_rect_raw

/camera/infra1/metadata

#### 2. 运行bag2dmvio.py生成图像数据

```
# python bag2dataset.py bag地址 数据集地址
# python bag2dataset.py **.bag ./
```

#### 3.运行interpolate_imu_file.py插值生成IMU数据

```
python interpolate_imu_file.py --input "imu_origin.txt" --times "cam0/times_nesc.txt" --output imu.txt
```

