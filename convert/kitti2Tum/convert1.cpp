//把kitti的groundtru类型的pose(时间戳 4X3的转移矩阵[去掉了0 0 0 1]) 转化为 Tum数据集类型的数据( 'timestamp tx ty tz qx qy qz qw' )

#include <iostream>
#include <iomanip> //要用到格式控制符
#include <fstream>
#include <boost/program_options.hpp>


#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <Eigen/Geometry>
#include <Eigen/Core>
#include <boost/format.hpp>  // for formating strings


#include <vector>
#include <string>

#include <sstream>
#include <boost/filesystem.hpp>
#include <algorithm>
#include <iterator>
#include <math.h>




using namespace std;


int loadKittiPoses(const string & file_name, vector<Eigen::Matrix4d> &poses) ;
int loadKittiTimes( const string & file_name,  vector<double> & timestamps);
int convert(  vector<Eigen::Matrix4d> & poses,  vector<double> & timestamps,  const string & filename);


int main( int argc, char** argv )
{

    if (argc != 4)
    {
        //example: ./project    kitti类型的pose.txt   kitti时间戳文件夹   要输出的文件名
        cerr << endl << "Usage: ./project    kittiPoseXX.txt     kittiTimeXX.txt    tumOutxx.txt " << endl;
        return 1;
    }

    string kittipath =  std::string(argv[1]);
    string kittTimepath =  std::string(argv[2]);
    string tumFileName = std::string(argv[3]);
    cout << kittipath << endl;
    cout << kittTimepath << endl;
    cout << tumFileName << endl;

    vector<Eigen::Matrix4d> poses;         // 相机位姿
    vector<double>  timestamps;// 时间戳


    // Eigen::Matrix4d Tr_velo_to_cam;
    // Tr_velo_to_cam  << -1.857739385241e-03,  -9.999659513510e-01,  -8.039975204516e-03,  -4.784029760483e-03,
    //                 -6.481465826011e-03, 8.051860151134e-03, -9.999466081774e-01, -7.337429464231e-02,
    //                 9.999773098287e-01, -1.805528627661e-03, -6.496203536139e-03, -3.339968064433e-01,
    //                 0, 0, 0, 1; ///从data_odometry_calib/05/calib.txt中读取的Tr( from velo to rectC0)

    //load  poses
    if (loadKittiPoses(kittipath,  poses) == 0 )
    {
        cerr << "cannot find pose file" << endl;
        return 0;
    }
    //load  poses
    if (loadKittiTimes(kittTimepath,  timestamps) == 0 )
    {
        cerr << "cannot find time file" << endl;
        return 0;
    }

    if ( convert( poses, timestamps, tumFileName) == 0)
    {
        cerr << "convert failed!!!" << endl;
        return 0;
    }


    cout << "It is done!------------------------------------------------" << endl;



    return 1;
}




int loadKittiPoses(const string &  file_name, vector<Eigen::Matrix4d> & poses)
{
    FILE *fp = fopen(file_name.c_str(), "r");
    if (!fp)
        return  0;
    while (!feof(fp)) {
        double P[3] [4];
        if (fscanf(fp, "%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf",
                   &P[0][0], &P[0][1], &P[0][2], &P[0][3],
                   &P[1][0], &P[1][1], &P[1][2], &P[1][3],
                   &P[2][0], &P[2][1], &P[2][2], &P[2][3] ) == 12) {
            Eigen::Matrix4d T;
            T << P[0][0], P[0][1], P[0][2], P[0][3],
            P[1][0], P[1][1], P[1][2], P[1][3],
            P[2][0], P[2][1], P[2][2], P[2][3],
            0, 0, 0, 1;
            poses.push_back(T);
        }
    }
    fclose(fp);
    return 1;

}

int loadKittiTimes( const string & file_name,  vector<double> & timestamps)
{
    FILE *fp = fopen(file_name.c_str(), "r");
    if (!fp)
        return  0;
    while (!feof(fp)) {
        double timestamp;
        if (fscanf(fp, "%lf ", &timestamp ) == 1) {
            timestamps.push_back(timestamp);
        }
    }
    fclose(fp);
    return 1;

}


int convert(  vector<Eigen::Matrix4d> & poses,  vector<double> & timestamps,  const string & filename)
{
    cout << endl << "Saving camera pose to " << filename << " ..." << endl;
    ofstream fileout;
    fileout.open(filename.c_str());
    fileout << "#Format: timestamp tx ty tz qx qy qz qw" << endl ;

    vector<double>::iterator itTimestamp = timestamps.begin();
    if (poses.size() != timestamps.size())
    {
        cerr << "!!!-----------poses.size() != timestamps.size()----------!!!" << endl;
        //return 0;
    }


//NOTE 这里以pose的size()为准
    for (vector<Eigen::Matrix4d>::iterator it = poses.begin(); it != poses.end(); it++, itTimestamp++)
    {
        Eigen::Matrix4d tmpPose = *it;
        Eigen::Matrix3d tmpRotationMatrix = tmpPose.block(0, 0, 3, 3);
        Eigen::Quaterniond q = Eigen::Quaterniond ( tmpRotationMatrix ); //// 请注意四元数 的顺序是(x,y,z,w), w 为实部，前三者为虚部
        //cout<< q.x() << " " << q.y() << " " << q.z() << " " << q.w() << endl;
        fileout << setprecision(6) << *itTimestamp << " " <<  setprecision(9) <<  tmpPose(0, 3) << " " << tmpPose(1, 3) << " " << tmpPose(2, 3) << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << endl;


    }

    fileout.close();
    cout << endl << "trajectory saved!" << endl;
    return 1;

}
