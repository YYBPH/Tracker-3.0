#include <opencv2/opencv.hpp>
#include <deque>
#include <thread>
#include <mutex>
#include <atomic>

#include <JetsonGPIO.h>
#include <chrono>

#include "objectTracker.hpp"
#include "serial2.h"

using namespace std;
using namespace cv;


/** serial ****************************/
char reciveBuf[10]{};
char return_msg[20]{};
serialPort myserial1;
serialPort myserial2;
const char* dev1 = "/dev/ttyUSB0";
const char* dev2 = "/dev/ttyUSB1";

/** cap ****************************/
cv::VideoCapture cap;
int frameWidth = -1;
int frameHeight = -1;

struct TrackerParam trackerParam ;

/** GPIO ****************************/
int inputPin = 18;
int outputPin = 23;



/** 多线程 ****************************/

deque<cv::Mat> frameDeque(5);
std::atomic<bool>  BREAK_FLAG = { false };
mutex frameMutex, flagMutex;

/** 函数声明 ****************************/
void GetFrameThread();
void TrackerThread();
void GPIOThread();

char* coordinate_conversion_X(double x, double y);
char* coordinate_conversion_Y(double x, double y);
void click_and_crop(int event, int x, int y, int flags, void* param);

int main()
{
    /*** cap ********************************************************************************/
    // cap.open("/home/nvidia/Desktop/testVideo.mp4");
    cap.open(0);
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 2560);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 1440);

    if (cap.isOpened() == false) {
        printf("\r\n\r\n**************************************************************\r\n");
        printf("capture open failed!\r\n");
        printf("*************************************************************\r\n\r\n\r\n");
        return 0;
    }

    /*** serial ********************************************************************************/
    if (myserial1.OpenPort(dev1) == false) {
        printf("\r\n\r\n**************************************************************\r\n");
        printf("Error opening Serial1.\r\n");
        printf("*************************************************************\r\n\r\n\r\n");
        return -1;
    };
    myserial1.setup(9600,0,8,1,'N'); 
    printf("serial1 opened!\r\n");

    if (myserial2.OpenPort(dev2) == false) {
        printf("\r\n\r\n**************************************************************\r\n");
        printf("Error opening Serial2.\r\n");
        printf("*************************************************************\r\n\r\n\r\n");
        return -1;
    };
    myserial2.setup(9600,0,8,1,'N'); 
    printf("serial2 opened!\r\n");

    /*** GPIO *****************************************************************************/
    GPIO::setmode(GPIO::BCM);           // 注意：BCM
    GPIO::setup(inputPin, GPIO::IN);


    cv::Mat testFrame;
    cap >> testFrame;
    frameWidth = testFrame.cols;
    frameHeight = testFrame.rows;
    printf("\r\n\r\nFrame:%d*%d\r\n\r\n", frameWidth, frameHeight);


    std::thread getFrameThread(GetFrameThread);
    std::thread trackerThread(TrackerThread);
    std::thread gpioThread(GPIOThread);


    getFrameThread.join();
    trackerThread.join();
    gpioThread.join();

    printf("\r\n\r\n**************************************************************\r\n");
    printf("programe end.\r\n");
    printf("*************************************************************\r\n\r\n\r\n");

}


void GetFrameThread()
{
    printf("GetFrameThread start!\r\n");
    while (!BREAK_FLAG)
    {
        cv::Mat newFrame;
        cap >> newFrame;

        frameMutex.lock();
        while (frameDeque.size() >= 3)
            frameDeque.pop_front();
        frameDeque.push_back(newFrame);
        frameMutex.unlock();
    }
}

void TrackerThread()
{
    printf("TrackerThread start!\r\n");
    
    // 延迟1秒
    std::this_thread::sleep_for(std::chrono::seconds(1));

    cv::namedWindow("newFrame", cv::WINDOW_NORMAL);
    cv::setMouseCallback("newFrame", click_and_crop);
    cv::resizeWindow("newFrame", cv::Size(800, 600));
    cv::createTrackbar("learn speed", "newFrame", &(trackerParam.learnSpeed), 30);

    ObjectsTracker objectsTracker;
    double startTime, endTime, totalTime;

    while (!BREAK_FLAG)
    {
        // 开始计时
        startTime = cv::getTickCount();

        // 获取新图像
        cv::Mat newFrame;
        frameMutex.lock();
        if(!frameDeque.empty())
        {
            newFrame = frameDeque.back();
            frameDeque.pop_back();
        }
        frameMutex.unlock();

        if (!newFrame.empty())
        {
            trackerParam.newFrame = newFrame;
            cv::Mat frame = objectsTracker.tracker(&trackerParam);
            

            cv::Point point = objectsTracker.getCoords();
            strcpy(reciveBuf, coordinate_conversion_X((double)point.x-1280.0, 720.0-(double)point.y));
            myserial1.writeBuffer((uint8_t*)reciveBuf, strlen(reciveBuf));
            printf("X:%s ", reciveBuf);

            strcpy(reciveBuf, coordinate_conversion_Y((double)point.x-1280.0, 720.0-(double)point.y));
            myserial2.writeBuffer((uint8_t*)reciveBuf, strlen(reciveBuf));
            printf("Y:%s \r\n", reciveBuf);


            // 结束计时
            endTime = cv::getTickCount();
            double fps = 1.0 / ((endTime - startTime) / cv::getTickFrequency());
            printf("FPS:%f\r\n", fps);

            imshow("newFrame", frame);
            int key = cv::waitKey(1);

            if (key == 27)
            {
                BREAK_FLAG = true;
                break;
            }

        }
        // else {
        //     cout << "Empty frame!" << endl;
        // }
    }

}

void GPIOThread()
{
    printf("GPIOThread start!\r\n");

    std::chrono::steady_clock::time_point pulseStart, pulseEnd;

    while (!BREAK_FLAG)
    {
        if(GPIO::input(inputPin) == GPIO::HIGH)
        {
            pulseStart = std::chrono::steady_clock::now();

            while(GPIO::input(inputPin) == GPIO::HIGH && !BREAK_FLAG){}

            pulseEnd = std::chrono::steady_clock::now();

            auto pulseDuration = std::chrono::duration_cast<std::chrono::microseconds>(pulseEnd - pulseStart);

            if(pulseDuration >= std::chrono::microseconds(990) && pulseDuration <= std::chrono::microseconds(1010))
            {
                flagMutex.lock();
                trackerParam.abandonFlag = true;
                flagMutex.unlock();
                printf("GPIO AbandonFlag\n\n");
            }
        }
    }
}



char* coordinate_conversion_X(double x, double y) {
    // 1920*1080:
    // 2560*1440: 1828
    double R = 1828.0;

    // atan返回弧度
    double alpha_level = atan(x / sqrt(R * R + y * y));
    // left:145     right:215
    alpha_level = ((alpha_level / 3.14159 * 180) + 145.0) / 360 * 16383;

    sprintf(return_msg, "RB%05d  ", int(alpha_level));
    // printf("X: %s  ", return_msg);

    return return_msg;
}

char* coordinate_conversion_Y(double x, double y) {
    double R = 1828.0;

    // sin函数输入弧度
    double _buf = sin(18.0 / 180.0 * 3.1415926 + atan(y / R));
    double alpha_vertical = asin((sqrt(R * R + y * y) * _buf) / sqrt(x * x + y * y + R * R));

    // 弧度转角度，再转码
    alpha_vertical = ((alpha_vertical / 3.14159 * 180.0) + 180.0) / 360.0 * 16383;

    sprintf(return_msg, "RB%05d  ", int(alpha_vertical));
    // printf("Y: %s  \n", return_msg);

    return return_msg;
}

// 鼠标回调函数
cv::Rect MaskRectsDel;
void click_and_crop(int event, int x, int y, int flags, void* param) {
    //最大速度调节：滚轮
    bool MAX_SPEED_FLAG = (event == cv::EVENT_MOUSEWHEEL);
    //最低速度调节：ctrl+滚轮
    bool MIN_SPEED_FLAG = (event == cv::EVENT_MOUSEWHEEL) && (flags & cv::EVENT_FLAG_CTRLKEY);

    if (event == cv::EVENT_LBUTTONDOWN) {
        if (x < 20) x = 0;
        if (y < 20) y = 0;
        trackerParam.MaskRects.push_back(cv::Rect(x, y, 0, 0));
    } else if (event == cv::EVENT_LBUTTONUP) {
        trackerParam.MaskRects.back().width = x - trackerParam.MaskRects.back().x;
        trackerParam.MaskRects.back().height = y - trackerParam.MaskRects.back().y;

        if (frameWidth - x < 20) trackerParam.MaskRects.back().width = frameWidth;
        if (frameHeight - y < 20) trackerParam.MaskRects.back().height = frameHeight;
    } else if (event == cv::EVENT_RBUTTONDOWN) {
        MaskRectsDel = cv::Rect(x, y, 0, 0);
    } else if (event == cv::EVENT_RBUTTONUP) {
        MaskRectsDel.width = x - MaskRectsDel.x;
        MaskRectsDel.height = y - MaskRectsDel.y;

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
    } else if (event == cv::EVENT_MBUTTONDOWN)
    {
        flagMutex.lock();
        trackerParam.abandonFlag = true;
        flagMutex.unlock();
        std::cout << "cv::EVENT_FLAG_SHIFTKEY" << std::endl;
    }
    else if (MIN_SPEED_FLAG) {
        int delta = cv::getMouseWheelDelta(flags);
        if (delta > 0) {
            trackerParam.minSpeed++;
            std::cout << "Mouse wheel scrolled up." << std::endl;
        }
        else if (delta < 0) {
            if (trackerParam.minSpeed > 0) {
                trackerParam.minSpeed--;
            }

            std::cout << "Mouse wheel scrolled down." << std::endl;
        }
    }
    else if (MAX_SPEED_FLAG) {
        int delta = cv::getMouseWheelDelta(flags);
        if (delta > 0) {
            trackerParam.maxSpeed++;
            std::cout << "Mouse wheel scrolled up." << std::endl;
        }
        else if (delta < 0) {
            if (trackerParam.maxSpeed > 0) {
                trackerParam.maxSpeed--;
            }

            std::cout << "Mouse wheel scrolled down." << std::endl;
        }
    }

}



