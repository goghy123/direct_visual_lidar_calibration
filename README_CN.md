***
[项目地址](https://github.com/koide3/direct_visual_lidar_calibration)
[教程参考](https://koide3.github.io/direct_visual_lidar_calibration/)
***
# 零、快捷启动指令集

## 1. 数据集录制
创建数据集文件夹`ouster`
```SHELL
mkdir -p ./Projects/ouster
cd ./Projects/ouster
```

自定义数据集名称
```SHELL
ros2 bag record -o livox_realsense \
  /camera/camera/color/image_raw \
  /camera/camera/color/camera_info \
  /livox/lidar \
  --storage sqlite3
```

## 2. 数据预处理

```SHELL
cd ./Projects
ros2 run direct_visual_lidar_calibration preprocess ouster ouster_preprocessed -adv
```

## 3.  手动初始估计

```SHELL
cd ./Projects
ros2 run direct_visual_lidar_calibration initial_guess_manual ouster_preprocessed
```

## 4. 精细化计算结果

``` SHELL
cd ./Projects
ros2 run direct_visual_lidar_calibration calibrate ouster_preprocessed
```

## 5. 静态TF发布

```SHELL
python3 make_static_tf_cmd.py calib.json
```

## 6. 误差检测

```SHELL
python3 lidar_camera_overlay.py
```

启动可视化
```SHELL
ros2 run rqt_image_view rqt_image_view
```

## 7. 标定结果可视化

```SHELL
python3 viz_calib.py calib.json --out calib_frames.html
```

# 一、项目安装
## 1.安装所需依赖
```SHELL
sudo apt install libomp-dev libboost-all-dev libglm-dev libglfw3-dev libpng-dev libjpeg-dev
```

## 2.安装GTSAM
下载项目
```SHELL
git clone https://github.com/borglab/gtsam
cd gtsam && git checkout 4.2a9
```
编译项目
```SHELL
mkdir build && cd build
cmake .. -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
         -DGTSAM_BUILD_TESTS=OFF \
         -DGTSAM_WITH_TBB=OFF \
         -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF \
         -DGTSAM_USE_SYSTEM_EIGEN=ON
```

> [!NOTE]
>对于Ubuntu 22.04, 需要增加-DGTSAM_USE_SYSTEM_EIGEN=ON（已加入）

安装项目
```SHELL
make -j$(nproc)
sudo make install
```


## 3.安装Ceres
> [!NOTE]
>安装前需要安装glog

```SHELL
sudo apt-get install libgoogle-glog-dev
```

下载项目
```SHELL
git clone --recurse-submodules https://github.com/ceres-solver/ceres-solver
cd ceres-solver
```

```SHELL
git checkout e47a42c2957951c9fafcca9995d9927e15557069
```

编译并安装项目
```SHELL
mkdir build && cd build
cmake .. -DBUILD_EXAMPLES=OFF -DBUILD_TESTING=OFF -DUSE_CUDA=OFF
make -j$(nproc)
sudo make install
```

## 4.安装Iridescence for visualization
下载项目
```SHELL
git clone https://github.com/koide3/iridescence --recursive
```

编译并安装项目
```SHELL
mkdir iridescence/build && cd iridescence/build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
sudo make install
```

## 5.安装SuperGlue (可选)
安装依赖库
```SHELL
pip3 install numpy opencv-python torch matplotlib
```

下载项目
```SHELL
git clone https://github.com/magicleap/SuperGluePretrainedNetwork.git
```

添加进环境变量
```SHELL
echo 'export PYTHONPATH=$PYTHONPATH:/path/to/SuperGluePretrainedNetwork' >> ~/.bashrc
source ~/.bashrc
```

## 编译项目
### ROS1
```SHELL
cd ~/Projects/catkin_ws/src
git clone https://github.com/koide3/direct_visual_lidar_calibration.git --recursive
cd .. && catkin_make
```

### ROS2
```SHELL
cd ~/Projects/ros2_ws/src
git clone https://github.com/koide3/direct_visual_lidar_calibration.git --recursive
cd .. && colcon build
```

***
# 二、数据集制作
## 1.数据集标定所需：
```text
# 相机图像（二选一）
/camera/image_raw (sensor_msgs/Image)
/camera/image_rect (sensor_msgs/Image)

# 相机内参（必需）
/camera/camera_info (sensor_msgs/CameraInfo)

# LiDAR 点云（必需）
/lidar/points (sensor_msgs/PointCloud2)

```

## 2.注意事项
### 前提条件
- 相机的固有参数（即相机矩阵和畸变系数）需要事先校准。
- 摄像头和激光雷达必须牢固固定。

### 数据收集步骤
- 非重复扫描激光雷达（Livox）：等10~15秒不移动传感器。
- 旋转激光雷达（Ouster）：缓慢上下移动传感器**10秒**。如果你的激光雷达扫描线较少（例如16或32条），可以继续移动一点（例如**20~30秒**）。
> [!NOTE]
>- 校准至少只需一次rosbag，但建议携带多个（5~10个rosbag）以获得更好的校准效果。
>- 每次开始录制时，传感器必须保持静止，以确保第一点云和图像处于相同的姿态。

## 3.数据集录制
创建数据集文件夹`ouster`
```SHELL
mkdir -p ./Projects/ouster
cd ./Projects/ouster
```

- 使用默认数据集名称（时间戳命名）
```SHELL
ros2 bag record \
  /camera/camera/color/image_raw \
  /camera/camera/color/camera_info \
  /velodyne_points \
  --storage sqlite3
```

- 自定义数据集名称
```SHELL
ros2 bag record -o livox_realsense \
  /camera/camera/color/image_raw \
  /camera/camera/color/camera_info \
  /livox/lidar \
  --storage sqlite3
```

***
# 三、项目运行
## 1.数据预处理
定位到数据集文件夹同级目录下
```SHELL
cd ./Projects
```

运行以下指令
```SHELL
ros2 run direct_visual_lidar_calibration preprocess ouster ouster_preprocessed -adv
```
- -a : Detect points/image/camera_info topics automatically 
- -d : Use dynamic points integrator 
- -v : Enable visualization

> [!NOTE]
>如果出现`error while loading shared libraries: libiridescence.so: cannot open shared object file: No such file or directory`，原因是gtsam库装在了/usr/local/lib中，而软件只会去/usr/lib中寻找。按如下方式解决

1. 用vim打开ld.so.conf， 按i进入插入模式
```SHELL
sudo vim /etc/ld.so.conf
```

2. 把新共享库目录加入到共享库配置文件/etc/ld.so.conf中
```SHELL
sudo /sbin/ldconfig -v
```

## 2.初始估计
### 手动初始估计
定位到数据集文件夹同级目录下
```SHELL
cd ./Projects
```

运行以下指令
```SHELL
ros2 run direct_visual_lidar_calibration initial_guess_manual ouster_preprocessed
```

取点步骤
1. 右键点击点云上的一个三维点和图像上的一个二维点
2. 点击按钮`Add picked points`
3. 重复1和2，至少得3分。越多越好。）
4. 点击按钮即可初步猜测激光雷达相机变换`Estimate`
5. 通过更改投影结果来检查图像结果是否正常`blend_weight`
6. 点击按钮保存初始猜测`Save`

### 自动初始估计
定位到数据集文件夹同级目录下
```SHELL
cd ./Projects
```

运行以下指令
```SHELL
ros2 run direct_visual_lidar_calibration find_matches_superglue.py ouster_preprocessed
```

以及
```SHELL
ros2 run direct_visual_lidar_calibration initial_guess_auto ouster_preprocessed
```

## 3.精细化计算结果
定位到数据集文件夹同级目录下
```SHELL
cd ./Projects
```

运行以下指令
``` SHELL
ros2 run direct_visual_lidar_calibration calibrate ouster_preprocessed
```

## 4.检查计算结果
定位到数据集文件夹同级目录下
```SHELL
cd ./Projects
```

运行以下指令
``` SHELL
ros2 run direct_visual_lidar_calibration calibrate ouster_preprocessed
```

# 四、标定结果检测

## 1. 静态TF指令

控制台输入以下指令
```SHELL
python make_static_tf_cmd.py calib.json
```

>所用到的工具都在check文件夹下

## 2. 标定误差检测
。。原理懒得写了，具体用法见yaml文件。

控制台输入以下指令

```SHELL
python3 lidar_camera_overlay.py
```

启动可视化
```SHELL
ros2 run rqt_image_view rqt_image_view
```

## 3. 标定结果可视化

控制台输入以下指令

```SHELL
python3 viz_calib.py calib.json --out calib_frames.html
```

