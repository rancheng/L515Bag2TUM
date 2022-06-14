# Realsense数据录制

启动realsense node

D455:
```shell
roslaunch realsense2_camera rs_camera.launch enable_depth:=false enable_infra1:=true enable_gyro:=true enable_accel:=true unite_imu_method:=linear_interpolation
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

