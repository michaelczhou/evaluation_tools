# evaluation_tools
Some tools for the evaluation of odometry and SLAM.

## 1.convert
该文件夹为转换轨迹的程序.

- KITTI格式: 为matrix4f格式的pose的前12位
- TUM格式: 为time x y z qx qy qz qw

### 1.1kitti2TUM
将KITTI格式转化为Tum格式
#### 用法: 
```
./convert ${KITTIPOSE} ${time.txt} ${OUTPOSE}
```
convert1用于转换轨迹真值和灰度序列轨迹
convert2用于转换彩色序列轨迹
### 1.2kitti_poses_and_timestamps_to_tum.py
将KITTI格式转化为Tum格式的Python脚本
#### 用法：
```
python kitti_poses_and_timestamps_to_tum.py ${kittiPosefile} ${kittiTimefile} ${Tumfile}
```
### 1.3pose_format_convert
调整位姿文件的时间戳，已经保留小数的位数，目的是时间戳对齐．
### 1.4convertOurPose
转换gps pose文件为用于评估的tum格式文件. 在config文件中设置各个文件的路径,则可将激光轨迹和gps轨迹都转换为tum格式的轨迹,用于上述评估.
### 1.5rtkgps2pose
转换gps的msg为pose文件. 在脚本文件中设置bag包的名字,输出文件名字以及gps中心和传感器中心的偏移量参数,如果已经修正过,则将偏移量参数设置为0,运行完之后得到 time x y z yaw格式的轨迹文件
### 1.6convertpose.py
### 1.7kitti2bag
```
sudo apt-get install pip 
sudo pip install kitti2bag
```
将原始文件下载后，解压到同一个文件下（可将三个标定文件加入另一个文件夹下）,然后可开始转换。
```
kitti2bag -t 2011_10_03 -r 0027 raw_synced .
```
### 1.8 kitti_devkit
```
sudo apt-get install texlive-extra-utils
sudo apt-get install gnuplot
```
#### 1.8.1download KITTI odometry ground truth datas to "PATH/TO/WORKSPACE/dataset/poses"
#### 1.8.2build target :
g++ -O3 -DNDEBUG -o evaluate_odometry evaluate_odometry.cpp matrix.cpp
#### 1.8.3put target <evaluaate_odometry> to your SLAM eval workspace.
#### 1.8.4usage:
./evaluate_odometry {PATH/TO/YOUR/SLAM/RESULT} {KITTI_SEQUENCE_NUMBER}

for example:
./evaluate_odometry ./CameraTrajectory.txt 2

## 2.tools

### 2.1 evo-plot
安装：
```
sudo pip install evo --upgrade --no-binary evo
```
[evo_wiki](https://github.com/MichaelGrupp/evo/wiki)
#### 2.1.1画轨迹
```
evo_traj euroc data.csv --plot
evo_traj tum output.txt --ref=tum06.txt -p --plot_mode=xz
evo_traj tum output.txt --ref=tum00_gt.txt -p -a -v
```
-a 可自动对齐 -v 参数更详细
#### 2.1.2转轨迹格式
```
evo_traj euroc data.csv --save_as_tum (--save_as_bag --save_as_kitti)
```
#### 2.1.3计算ape,rpe,rmse
```
evo_ape tum tum06.txt output.txt -va --plot --plot_mode xy
```
#### 2.1.4绘制不同算法（ORB PTAM等）中与真实轨迹的图
```
evo_traj kitti KITTI_00_ORB.txt KITTI_00_SPTAM.txt --ref=KITTI_00_gt.txt -p --plot_mode=xz
```
#### 2.1.5调整显示尺寸
```
evo_config set plot_figsize 5 4.5
```
#### 2.1.6处理不同结果

*First trajectory (ORB Stereo):*

```
mkdir results

evo_ape kitti KITTI_00_gt.txt KITTI_00_ORB.txt -va --plot --plot_mode xz --save_results results/ORB.zip
```

*Second trajectory (S-PTAM):*

```
evo_ape kitti KITTI_00_gt.txt KITTI_00_SPTAM.txt -va --plot --plot_mode xz --save_results results/SPTAM.zip
```

Here, we use the results from above to generate a plot and a table:

```
evo_res results/*.zip -p --save_table results/table.csv
```

### 2.2 rgbd_benchmark_tools
[Useful tools for the RGB-D benchmark](https://vision.in.tum.de/data/datasets/rgbd-dataset/tools#evaluation)
#### 2.2.1associate.py
通过timestamp用于生成rgb和depth关联文件

```
python associate.py rgb.txt depth.txt > fr_pioneer_slam2.txt
```

#### 2.2.2评估标准
运行完rgbd_tum 后生成CameraTrajectory.txt

##### (1)ATE 绝对误差—evaluate_ate.py
适用于评估视觉SLAM系统
```
python evaluate_ate.py groundtruth.txt CameraTrajectory.txt
````
###### 输出RMSE/cm误差

```
python evaluate_ate.py groundtruth.txt CameraTrajectory.txt --plot result.png
````
###### 输出真实轨迹和预测轨迹以及误差
```
python evaluate_ate.py groundtruth.txt CameraTrajectory.txt --verbose
```
###### 输出所有误差，包含平均值，中值等

##### (2)RPE相对误差—evaluate_rpe.py
适用于评估视觉里程计的漂移量
##### (3)Generating a point cloud from images
generate_pointcloud.py
##### (4)Adding point clouds to ROS bag files
add_pointclouds_to_bagfile.py
##### (5)project_point_cloud_to_image.py
