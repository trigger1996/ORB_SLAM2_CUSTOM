#include "include/main.h"

using namespace std;
using namespace cv;

int main(int argc, char **argv) {

    VideoCapture cap(0);
    if (cap.isOpened() == false) {
        cout << "camera opened failed" << endl;
        return -1;
    }

    Mat frame;

    double time = 0;

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR,true);

    while (true) {

        cap >> frame;
        resize(frame, frame, Size(480, 360));
        //imshow("cam", frame);

        time+=5000;
        SLAM.TrackMonocular(frame, time);

        if (waitKey(30) == 27) {
            SLAM.Shutdown();
            return 0;
        }

    }

    // Stop all threads
    SLAM.Shutdown();

    waitKey(0);
    return 0;
}
