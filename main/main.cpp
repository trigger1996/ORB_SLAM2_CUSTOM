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

    cv::Mat get_Pos() {
        cv::Mat temp;
        temp = mpTracker->mCurrentFrame.GetCameraCenter();
        return temp.clone();
    }

protected:

};
}

int main(int argc, char **argv) {

    int stat;

    VideoCapture cap(0);
    if (cap.isOpened() == false) {
        cout << "camera opened failed" << endl;
        return -1;
    }
    Mat frame;
    Mat f_left, f_right;
    double time = 0;

    stat = skt.init();
    if (stat == -1) {
        cout << "udp created failed" << endl;
        return -1;
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::__myslam SLAM(argv[1],argv[2],ORB_SLAM2::System::STEREO,true);

    while (true) {

        cap >> frame;
        //resize(frame, frame, Size(640, 480));
        //imshow("cam", frame);
        f_right = frame(Rect(0, 0, 320, 240));
        f_left = frame(Rect(320, 0, frame.cols - 320, frame.rows));     // 这个相机左右是反的
        //imshow("left", f_left);
        //imshow("right", f_right);


        time+=5000;
        SLAM.TrackStereo(f_left, f_right, time);

        Mat cam_center = SLAM.get_Pos();
        if (!cam_center.empty()) {
            //cout << cam_center << endl;     // mm
            //cout << cam_center.at<float>(0, 0) << " " << cam_center.at<float>(0, 1) << " " << cam_center.at<float>(0, 2) << endl;
            skt.set_PosVector(cam_center.at<float>(0, 0), cam_center.at<float>(0, 1), cam_center.at<float>(0, 2));
            skt.send_Data();
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
