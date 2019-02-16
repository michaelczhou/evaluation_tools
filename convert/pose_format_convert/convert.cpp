#include <string>
#include <iostream>
#include <fstream>

#include <Eigen/Core>
#include <Eigen/Geometry>

using namespace std;

// path to trajectory file
string fInputPath = "./tum_06_gt.txt";
string fOutPath = "./tum06.txt";
double init_time = 1317355712.01035; // sequence 04 --> 0 frame in the raw data 0930-16
                                // 1317354639.90

int main(int argc, char **argv)
{
    ofstream foutPose(fOutPath, ios::out);
    fstream fInPose(fInputPath);
    string single_line;
    double timeStamp;
    double tx, ty, tz, qx, qy, qz, qw;
    
    while( getline(fInPose, single_line) )
    {
        stringstream in_stream(single_line);
        in_stream >> timeStamp >> tx >> ty >> tz >> qx >> qy >> qz >> qw;

        foutPose.setf(ios::fixed, ios::floatfield);
        foutPose.precision(5);
        foutPose << timeStamp+init_time << " " << tx << " " << ty << " " << tz << " "
                << qx << " " << qy << " " << qz << " " << qw << endl;
    }

    foutPose.close();

    return 0;

}