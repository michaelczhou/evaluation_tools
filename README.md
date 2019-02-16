# evaluation_tools
Some tools for the evaluation of odometry and SLAM.

## 1.convert
该文件夹为转换轨迹的程序.

- KITTI格式: 为matrix4f格式的pose的前12位
- TUM格式: 为time x y z qx qy qz qw

### 1.1kitti2TUM
将KITTI格式转化为Tum格式
#### 用法: 
./convert ${KITTIPOSE} ${time.txt} ${OUTPOSE}
convert1用于转换轨迹真值和灰度序列轨迹
convert2用于转换彩色序列轨迹
### 1.2kitti_poses_and_timestamps_to_tum.py
将KITTI格式转化为Tum格式的Python脚本
#### 用法：
python kitti_poses_and_timestamps_to_tum.py ${kittiPosefile} ${kittiTimefile} ${Tumfile}
### 1.3pose_format_convert
调整位姿文件的时间戳，已经保留小数的位数，目的是时间戳对齐．
### 1.4convertOurPose
转换gps pose文件为用于评估的tum格式文件. 在config文件中设置各个文件的路径,则可将激光轨迹和gps轨迹都转换为tum格式的轨迹,用于上述评估.
### 1.5rtkgps2pose
转换gps的msg为pose文件. 在脚本文件中设置bag包的名字,输出文件名字以及gps中心和传感器中心的偏移量参数,如果已经修正过,则将偏移量参数设置为0,运行完之后得到 time x y z yaw格式的轨迹文件
### 1.6convertpose.py


## 2.tools
### 2.1evo
安装：
pip install evo --upgrade --no-binary evo
#### 2.1.1画轨迹
evo_traj euroc data.csv --plot
evo_traj tum output.txt --ref=tum06.txt -p --plot_mode=xz
#### 2.1.2转轨迹格式
evo_traj euroc data.csv --save_as_tum (--save_as_bag --save_as_kitti)
#### 2.1.3计算ape,rpe,rmse
evo_ape tum tum06.txt output.txt -va --plot --plot_mode xy
#### 2.1.4绘制不同算法（ORB PTAM等）中与真实轨迹的图
evo_traj kitti KITTI_00_ORB.txt KITTI_00_SPTAM.txt --ref=KITTI_00_gt.txt -p --plot_mode=xz
#### 2.1.5调整显示尺寸
evo_config set plot_figsize 5 4.5
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

### 2.2tum
