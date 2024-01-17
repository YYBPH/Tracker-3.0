#include <opencv2/opencv.hpp>
#include "objectTracker.hpp"

using namespace std;
using namespace cv;

bool abandonFlag = false;


void click_and_crop(int event, int x, int y, int flags, void* param);

int main()
{
    cv::VideoCapture cap;

    cap.open("E:\\Desktop\\test1.mp4");

    if (cap.isOpened() == false) {
        printf("\r\n\r\n**************************************************************\r\n");
        printf("capture open failed!\r\n");
        printf("*************************************************************\r\n\r\n\r\n");
        return 0;
    }

    ObjectsTracker objectsTracker;

    cv::namedWindow("newFrame", cv::WINDOW_NORMAL);
    cv::setMouseCallback("newFrame", click_and_crop);
    cv::resizeWindow("newFrame", cv::Size(800, 600));

    while (1)
    {   
        // 获取新图像
        Mat newFrame;
        cap >> newFrame;

        if (!newFrame.empty()) 
        {
            cv::Mat frame;
            frame = objectsTracker.tracker(newFrame, abandonFlag);


            imshow("newFrame", frame);

            int key = cv::waitKey(1);
            if (key == 27)
                break;

        } else 
        {
            cout << "Empty frame!" << endl;
            return 0;
        }
    }
}


// 鼠标回调函数
void click_and_crop(int event, int x, int y, int flags, void* param) {
    if (event == cv::EVENT_MBUTTONDOWN) 
    {
        abandonFlag = true;
        std::cout << "cv::EVENT_FLAG_SHIFTKEY" << std::endl;
    }
}

