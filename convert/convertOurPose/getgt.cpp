#include <iostream>
#include <fstream>

#include <vector>
#include <string>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/StdVector>

#include <iomanip>


#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>






using namespace std;


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

int loadStampPoses(string file_name, vector<double>& stamps,vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d> > &poses) {
    FILE *fp = fopen(file_name.c_str(), "r");
    if (!fp)
        return  0;
    while (!feof(fp)) {
        double stamp;
        double P[3] [4];
        if(fscanf(fp,"%lf",&stamp)==1){
            stamp/=1000;
            stamps.push_back(stamp);
        }
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

int main(int argc, char** argv){
//    vector<Eigen::Matrix4d,Eigen::aligned_allocator<Eigen::Matrix4d> > poses;
//    loadPoses("/home/ywl/ourdata/data060502/cam_gt_pose.txt",poses);
//    for(int i=0;i<poses.size();i++){
//        cout<<i<<endl;
//        Eigen::Matrix4d p=poses[i].inverse();
//        cout<<p<<endl;
//    }


#if 1
    string stampfile="/media/ywl/samsungT5/velodyne_points.txt";
    string lvinsposefile="/home/ywl/ourdata/trajectory/mapping_pose.txt";
    string mapposefile="/media/ywl/samsungT5/061403/mapping_pose.txt";
    string gtfile="/home/ywl/ourdata/trajectory/gt_pose.txt";
    vector<double> stamp;
    vector<double> mapstamp;
    vector<Eigen::Matrix4d,Eigen::aligned_allocator<Eigen::Matrix4d> > lvinspose;
    vector<Eigen::Matrix4d,Eigen::aligned_allocator<Eigen::Matrix4d> > mappose;
    loadPoses(lvinsposefile,lvinspose);
    loadStamp(stampfile,stamp);
    mapstamp.push_back(stamp[0]);
    mappose.push_back(lvinspose[0]);
    loadStampPoses(mapposefile,mapstamp,mappose);
//    int firstfram=10;
//    for(int i=1;i<mapstamp.size();i++){
//        mapstamp[i]+=stamp[firstfram];
//    }
//    cout<<fixed<<mapstamp[1]<<" "<<mapstamp[mapstamp.size()-1]<<endl;

    int lastindex=0;
    vector<Eigen::Matrix4d,Eigen::aligned_allocator<Eigen::Matrix4d> > gtpose;
    //gtpose.push_back(mappose[0]);
    for(int i=1;i<mapstamp.size();i++){
        int index=0;
        while(mapstamp[i]>stamp[index]){
            index++;
            if(index==stamp.size())
                break;
        }
        for(int j=lastindex;j<index;j++){
            Eigen::Matrix4d nowpose;
            nowpose=mappose[i-1]*lvinspose[lastindex].inverse()*lvinspose[j];
            gtpose.push_back(nowpose);
        }
        lastindex=index;
    }
    if(gtpose.size()<lvinspose.size()){
        for(int i=gtpose.size();i<lvinspose.size();i++){
            Eigen::Matrix4d nowpose;
            nowpose=mappose[mappose.size()-1]*lvinspose[gtpose.size()].inverse()*lvinspose[i];
            gtpose.push_back(nowpose);
        }
    }
    ofstream ofs;
    ofs.open(gtfile,std::ios::out);
    for(int i=0;i<gtpose.size();i++){
        Eigen::Matrix4d pose=gtpose[i];
        ofs<<fixed<<setprecision(8)<<pose(0,0)<<" "<<pose(0,1)<<" "<<pose(0,2)<<" "<<pose(0,3)<<" "<<
             pose(1,0)<<" "<<pose(1,1)<<" "<<pose(1,2)<<" "<<pose(1,3)<<" "<<
             pose(2,0)<<" "<<pose(2,1)<<" "<<pose(2,2)<<" "<<pose(2,3)<<endl;
    }

#endif


}
