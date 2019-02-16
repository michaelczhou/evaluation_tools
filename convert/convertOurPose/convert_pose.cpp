#include <iostream>
#include <fstream>
#include <boost/program_options.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/eigen.hpp>
#include <boost/format.hpp>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <vector>
#include <string>

#include <sstream>
#include <boost/filesystem.hpp>
#include <algorithm>
#include <iterator>
#include <math.h>



using namespace std;
using namespace cv;


int loadPoses(string file_name, vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d> > &poses) {
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


int loadNdtPoses(string file_name, vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > &pose_xyz,vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > &pose_rpy) {
    FILE *fp = fopen(file_name.c_str(), "r");
    if (!fp)
        return  0;
    while (!feof(fp)) {
        double P[6];
        if (fscanf(fp, "%lf %lf %lf %lf %lf %lf",
                   &P[0], &P[1], &P[2], &P[3], &P[4], &P[5] ) == 6) {
            Eigen::Vector3d xyz,rpy;
            xyz<<P[0],P[1],P[2];
            rpy<<P[3],P[4],P[5];
            pose_rpy.push_back(rpy);
            pose_xyz.push_back(xyz);
        }
    }
    fclose(fp);
    return 1;

}

int loadGpsPoses(string file_name, vector<double> &stamp,vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > &pose,vector<double> &yaws) {
    FILE *fp = fopen(file_name.c_str(), "r");
    if (!fp)
        return  0;
    while (!feof(fp)) {
        double time;
        double P[4];
        if (fscanf(fp, "%lf %lf %lf %lf %lf",
                   &time, &P[0], &P[1], &P[2], &P[3]) == 5) {
            Eigen::Vector3d p;
            p<<P[0],P[1],P[2];
            stamp.push_back(time);
            pose.push_back(p);
            yaws.push_back(P[3]);
        }
    }

    fclose(fp);
    return 1;

}

void loadStamp(string file_name, vector<double>& stamp){
    FILE *fp = fopen(file_name.c_str(), "r");
    if (!fp){
        cout<<"open file fail: "<<file_name<<endl;
        return  ;
    }
    while (!feof(fp)) {
        double timestamp;
        if (fscanf(fp, "%lf", &timestamp ) == 1) {
            stamp.push_back(timestamp);
        }
    }
    fclose(fp);
    return ;
}

bool CheckGPS(Eigen::Vector3d pose){//python在处理时，如果某一帧gps是失锁的，就会赋值-1.
    if((pose(0)==-1.0)&&(pose(1)==-1.0)&&(pose(2)==-1.0)){
        return false;
    }else{
        return true;
    }
}

int main(int argc, char** argv){


    ofstream ofs;
    cv::FileStorage fsSettings("../config/config.yaml", cv::FileStorage::READ);
    const string gpsfile=fsSettings["gpsfile"];
    const string velodyneStampFile=fsSettings["velodyneStampFile"];
    const string velodynePoseFile=fsSettings["velodynePoseFile"];
    const string outGTFile=fsSettings["outGTFile"];
    const string outPoseFile=fsSettings["outPoseFile"];

#if 1   //transform gps
    vector<double> gpsstamp;
    vector<double> lvinstamp;
    vector<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d> > gpspose;
    vector<Eigen::Matrix4d,Eigen::aligned_allocator<Eigen::Matrix4d> > lvinspose;
    vector<double> yaws;
    loadGpsPoses(gpsfile,gpsstamp,gpspose,yaws);
    loadStamp(velodyneStampFile,lvinstamp);
    loadPoses(velodynePoseFile,lvinspose);
    cout<<"load done"<<endl;
    cout<<"gpsstamp.size() " << gpsstamp.size()<<" gpspose.size() "<<gpspose.size()<<" lvinstamp.size() "<<lvinstamp.size()<<endl;
    ofs.open(outGTFile,std::ios::out);
    ofstream lvinsofs;
    lvinsofs.open(outPoseFile,std::ios::out);

    Eigen::Quaterniond q;

    for(int i=0;i<lvinstamp.size();i++){//给每个激光帧打上gps的pose
        bool gpsvalid=true;
        int index=0;
        int j=0;
        while(gpsstamp[j]<=lvinstamp[i]){//找左边的帧
            j++;
            if(j==gpsstamp.size())
                break;
        }
        Eigen::Vector3d ap;//每一帧激光对应的真值
        if(j==0){//当激光数据比gps先到
            ap = gpspose[0];
        }else if(j==gpsstamp.size()){//当GPS结束，激光没有结束
            ap = gpspose[gpspose.size()-1];
        }else{//正常帧
            double a = (lvinstamp[i]-gpsstamp[j-1])/(gpsstamp[j]-gpsstamp[j-1]);
            ap << gpspose[j-1](0)+a*(gpspose[j](0)-gpspose[j-1](0)),
                  gpspose[j-1](1)+a*(gpspose[j](1)-gpspose[j-1](1)),
                  gpspose[j-1](2)+a*(gpspose[j](2)-gpspose[j-1](2));

            if(CheckGPS(gpspose[j-1])&&(!CheckGPS(gpspose[j]))){
                ap=gpspose[j-1];
            }else if(CheckGPS(gpspose[j])&&(!CheckGPS(gpspose[j-1]))){
                ap=gpspose[j];
            }
        }
        if((ap(0)==-1)&&(ap(1)==-1)&&(ap(2)==-1))
            gpsvalid=false;

        Eigen::Matrix3d rotation_matrix=lvinspose[i].block(0,0,3,3);
        q=Eigen::Quaterniond(rotation_matrix);
        //if(gpsvalid)
        lvinsofs<<setprecision(16)<<lvinstamp[i]<<" "<<setprecision(8)<<lvinspose[i](0,3)<<" "<<lvinspose[i](1,3)<<" "<<lvinspose[i](2,3)<<" "
          <<q.x()<<" "<<q.y()<<" "<<q.z()<<" "<<q.w()<<endl;
        if(gpsvalid)
            ofs<<setprecision(16)<<lvinstamp[i]<<" "<<setprecision(8)<<ap(0)<<" "<<ap(1)<<" "<<ap(2)<<" 0 0 0 1"<<endl;
    }
    ofs.close();


#endif



}

