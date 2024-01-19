#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;


int frameWidth = -1;
int frameHeight = -1;


int main()
{
    cv::VideoCapture cap;
    cap.open(0);
    cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('Y', 'U', 'Y', '2'));
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 1920);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 1080);
    //cap.open("E:\\Desktop\\test1.mp4");

    if (cap.isOpened() == false) {
        printf("\r\n\r\n**************************************************************\r\n");
        printf("capture open failed!\r\n");
        printf("*************************************************************\r\n\r\n\r\n");
        return 0;
    }

    cv::Mat testFrame;
    cap >> testFrame;
    frameWidth = testFrame.cols;
    frameHeight = testFrame.rows;
    printf("\r\n\r\nFrame:%d*%d\r\n\r\n", frameWidth, frameHeight);


    cv::namedWindow("newFrame", cv::WINDOW_NORMAL);
    cv::resizeWindow("newFrame", cv::Size(800, 600));

    double startTime, endTime, totalTime;

    while (1)
    {   
        // 开始计时
        startTime = cv::getTickCount();

        // 获取新图像
        Mat newFrame;
        cap >> newFrame;
        if (!newFrame.empty()) 
        {
            // 结束计时
            endTime = cv::getTickCount();
            double fps = 1.0 / ((endTime - startTime) / cv::getTickFrequency());
            printf("FPS:%f\r\n", fps);

            imshow("newFrame", newFrame);
            int key = cv::waitKey(1);
            if (key == 27)
                break;

        } else {
            cout << "Empty frame!" << endl;
            return 0;
        }
    }
}

