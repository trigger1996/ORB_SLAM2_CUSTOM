#include "include/main.h"

using namespace std;
using namespace cv;
using namespace ORB_SLAM2;

namespace ORB_SLAM2 {

class __myslam : public System {

public:
    __myslam(const string &strVocFile, const string &strSettingsFile, const eSensor sensor, const bool bUseViewer = true) :
        System(strVocFile, strSettingsFile, sensor, bUseViewer) {

    }

protected:

};
}

int main(int argc, char **argv) {

    int stat;
    int i = 0;

    VideoCapture cap(0);
    if (cap.isOpened() == false) {
        cout << "camera opened failed" << endl;
        return -1;
    }

    Mat frame;
    Mat f_left, f_right;
    Mat Tcw, mtcw, mRwc;

    double time = 0;

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::__myslam SLAM(argv[1],argv[2],ORB_SLAM2::System::STEREO,false);

    __lidar_driver *lidar;
    lidar = new __lidar_driver;
    lidar->init();

    //__icp *_icp;
    //_icp = new __icp;

    GridMapping *map_global;
    map_global = new GridMapping(0, 2.2, -2.2, 200, 0.5, 80000, 150, 0);

    //vector<__scandot> data, data_last;
    //data.clear();
    //data_last.clear();

    double x_esti = 0, y_esti = 0;
    double robotTheta = 0;
    double pitch = 0, roll = 0, yaw = 0;            // 弧度制

    while (true) {

        cap >> frame;
        //resize(frame, frame, Size(640, 480));
        imshow("cam", frame);
        f_left = frame(Rect(0, 0, 320, 240));
        f_right = frame(Rect(320, 0, frame.cols - 320, frame.rows));
        //imshow("left", f_left);
        //imshow("right", f_right);

        time+=5000;
        Tcw = SLAM.TrackStereo(f_left, f_right, time);

        if (!Tcw.empty()) {
            mtcw = Tcw.rowRange(0,3).col(3);

            stat = lidar->grab_ScanData();
            if (stat == __SUCCEEDED) {
                Mat lidarimg;
                //lidar->fix_Data_withAHRS(pitch, roll, yaw, true);
                lidar->draw(lidarimg, lidar->laserArray, (char *)"lidar_img_raw", true);

                // XY方向位移
                x_esti =  mtcw.at<float>(1, 0) * 100.0f;     // y
                y_esti = -mtcw.at<float>(0, 0) * 100.0f;     // -x
                // 偏航角
                mRwc = Tcw.rowRange(0,3).colRange(0,3).t();
                vector<float> q = Converter::toQuaternion(mRwc);                                              // 旋转矩阵转四元数

                roll = atan2f(2.f * (q[2]*q[3] + q[0]*q[1]), q[0]*q[0] - q[1]*q[1] - q[2]*q[2] + q[3]*q[3]);
                pitch = asinf(2.f * (q[0]*q[2] - q[1]*q[3]));
                yaw = atan2f(2.f * (q[1]*q[2] + q[0]*q[3]), q[0]*q[0] + q[1]*q[1] - q[2]*q[2] - q[3]*q[3]);   // 四元数转欧拉角
                robotTheta = yaw * 180.0f / CV_PI;

                map_global->updateGridMap_Laser(x_esti, y_esti, robotTheta * (CV_PI / 180), lidar->laserArray);
                map_global->showGridMap_with_Traj("Laser Grid Map");
           }
           cout << "x: " << x_esti << " y: " << y_esti << " yaw: " << robotTheta << endl;
        }


        if (waitKey(5) == 27) {
            SLAM.Shutdown();
            return 0;
        }

    }

    // Stop all threads
    SLAM.Shutdown();

    waitKey(0);
    return 0;
}
