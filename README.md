# evaluation_tools
Some tools for the evaluation of odometry and SLAM.

## convert
### kitti2TUM
将KITTI格式转化为Tum格式
#### 用法: 
./convert ${KITTIPOSE} ${time.txt} ${OUTPOSE}
convert1用于转换轨迹真值和灰度序列轨迹
convert2用于转换彩色序列轨迹
### kitti_poses_and_timestamps_to_tum.py
将KITTI格式转化为Tum格式的Python脚本
#### 用法：
python kitti_poses_and_timestamps_to_tum.py ${kittiPosefile} ${kittiTimefile} ${Tumfile}
### pose_format_convert
调整位姿文件的时间戳，已经保留小数的位数，目的是时间戳对齐．
### convertOurPose
### rtkgps2pose
### convertpose.py

