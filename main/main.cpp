#include "include/main.h"

using namespace std;
using namespace cv;
using namespace ORB_SLAM2;

int serial_send( int fd, char *Data );

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

int main(int argc, char **argv)
{
    int fd;
    int num;
    struct termios oldstdio;
    
    float x, y, z;
    float x_last, y_last, z_last;
    float vx, vy, vz;
    char tx[200] = { 0 };
    char rx[200] = { 0 };

    VideoCapture cap(0);
    if (cap.isOpened() == false) {
        cout << "camera opened failed" << endl;
        return -1;
    }

    Mat frame;
    Mat f_left, f_right;
    
    double time = 0, dt = 0;
    struct timeval t_now = { 0 }, t_last = { 0 };
    
    fd = open("/dev/ttyAMA0", O_RDWR );
    if( -1==fd )
    {
        printf("cannot open /dev/ttyAMA0\r\n");
        return -1;
    }
    tcgetattr( fd, &oldstdio);
    cfsetispeed(&oldstdio, B115200);
    tcsetattr( fd, TCSANOW, &oldstdio);
    tcflush( fd, TCIFLUSH );

    
    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::__myslam SLAM(argv[1],argv[2],ORB_SLAM2::System::STEREO,true);       // true

    while (true) {
    
        cap >> frame;
        //resize(frame, frame, Size(640, 480));
        //imshow("cam", frame);
        f_right = frame(Rect(0, 0, 320, 240));
        f_left = frame(Rect(320, 0, frame.cols - 320, frame.rows));     // 这个相机左右是反的
        //imshow("left", f_left);
        //imshow("right", f_right);
        
        t_last = t_now;
        gettimeofday(&t_now, NULL);
        dt = t_now.tv_usec - t_last.tv_usec;
        if (dt < 0.)
            dt += 1e6;
        
        time += dt;
        Mat cam_center = SLAM.TrackStereo(f_left, f_right, time);

        cout << "dt: " << dt << endl;
        
         //Mat cam_center = SLAM.get_Pos();
        if (!cam_center.empty()) {
            //cout << cam_center << endl;     // mm
            x_last = x;
            y_last = y;
            z_last = z;
            x = cam_center.at<float>(0, 3);
            y = cam_center.at<float>(1, 3);
            z = cam_center.at<float>(2, 3);

            vx = (x - x_last) * 1e6 / dt;
            vy = (y - y_last) * 1e6 / dt;
            vz = (z - z_last) * 1e6 / dt;

            cout << "x: " << x << " y: " << y << " z: " << z << endl;

            memset(tx, 0, sizeof(char) * 120);
            sprintf(tx, "x:%f y:%f z:%f vx:%f vy:%f vz:%f\r\n", x, y, z, vx, vy, vz);
            num = serial_send(fd, tx);
        }
//        if (waitKey(5) == 27) {
//            SLAM.Shutdown();
//            return 0;
//        }
    }

    // Stop all threads
    SLAM.Shutdown();
    close(fd);
    
    close(fd);
    return 0;
}

int serial_send( int fd, char *Data )
{
    int string_num;
    string_num = strlen(Data);
    return  write( fd,Data, string_num );
}
