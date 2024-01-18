#include <opencv2/opencv.hpp>
#include "objectTracker.hpp"

using namespace std;
using namespace cv;


struct TrackerParam trackerParam;


int frameWidth = -1;
int frameHeight = -1;

void click_and_crop(int event, int x, int y, int flags, void* param);

int main()
{
    cv::VideoCapture cap;
    cap.open(0);
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
    cv::setMouseCallback("newFrame", click_and_crop);
    cv::resizeWindow("newFrame", cv::Size(800, 600));

    ObjectsTracker objectsTracker;

    while (1)
    {   
        // 获取新图像
        Mat newFrame;
        cap >> newFrame;

        if (!newFrame.empty()) 
        {
            trackerParam.newFrame = newFrame;
            cv::Mat frame = objectsTracker.tracker(&trackerParam);



            imshow("newFrame", frame);
            int key = cv::waitKey(1);
            if (key == 27)
                break;

        } else {
            cout << "Empty frame!" << endl;
            return 0;
        }
    }
}



cv::Rect MaskRectsDel;

// 鼠标回调函数
void click_and_crop(int event, int x, int y, int flags, void* param) {

    if (event == cv::EVENT_LBUTTONDOWN) {
        if (x < 20) x = 0;
        if (y < 20) y = 0;
        trackerParam.MaskRects.push_back(cv::Rect(x, y, 0, 0));
    }

    else if (event == cv::EVENT_LBUTTONUP) {
        trackerParam.MaskRects.back().width = x - trackerParam.MaskRects.back().x;
        trackerParam.MaskRects.back().height = y - trackerParam.MaskRects.back().y;

        if (frameWidth - x < 20) trackerParam.MaskRects.back().width = frameWidth;
        if (frameHeight - y < 20) trackerParam.MaskRects.back().height = frameHeight;
    }

    else if (event == cv::EVENT_RBUTTONDOWN) {
        MaskRectsDel = cv::Rect(x, y, 0, 0);
    }

    else if (event == cv::EVENT_RBUTTONUP) {
        MaskRectsDel.width = x - MaskRectsDel.x;
        MaskRectsDel.height= y - MaskRectsDel.y;

        // 计算矩形的四个边界
        int left1 = MaskRectsDel.x;
        int right1 = MaskRectsDel.x + MaskRectsDel.width;
        int top1 = MaskRectsDel.y;
        int bottom1 = MaskRectsDel.y + MaskRectsDel.height;

        for (size_t i = 0; i < trackerParam.MaskRects.size(); i++)
        {
            int left2 = trackerParam.MaskRects[i].x;
            int right2 = trackerParam.MaskRects[i].x + trackerParam.MaskRects[i].width;
            int top2 = trackerParam.MaskRects[i].y;
            int bottom2 = trackerParam.MaskRects[i].y + trackerParam.MaskRects[i].height;

            if (!(right1 < left2 || left1 > right2 || bottom1 < top2 || top1 > bottom2) ||      // 交集
                (left1 <= left2 && top1 <= top2 && right1 >= right2 && bottom1 >= bottom2))     // 包含
            {
                swap(trackerParam.MaskRects[i], trackerParam.MaskRects[trackerParam.MaskRects.size() - 1]);
                trackerParam.MaskRects.pop_back();
                i--;
            }
        }
    }

    else if (event == cv::EVENT_MBUTTONDOWN)
    {
        trackerParam.abandonFlag = true;
        std::cout << "cv::EVENT_FLAG_SHIFTKEY" << std::endl;
    }
}



