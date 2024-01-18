#ifndef __OBJECT_TRACKER2_H
#define  __OBJECT_TRACKER2_H

#include <opencv2/opencv.hpp>
#include <vector>
#include <float.h>

// #include "Special.hpp"
#include "special.hpp"
#include "special2.hpp"
#include "object.hpp"

using namespace std;
using namespace cv;

#define MAX_DIST 100
#define MAX_NUM	 500


#define SPECIAL_DEBUG


#define RED cv::Scalar(0, 0, 255)
#define BLUE cv::Scalar(255, 0, 0)
#define GREEN cv::Scalar(0, 255, 0)
#define BLACK cv::Scalar(0, 0, 0)
#define CYAN cv::Scalar(255, 255, 0)	// 青色



struct TrackerParam
{
	cv::Mat newFrame;
	int minSpeed;
	int maxSpeed;
	bool abandonFlag;
	std::vector<cv::Rect> MaskRects;
};



class ObjectsTracker
{
public:
	ObjectsTracker();
	~ObjectsTracker();

	cv::Mat tracker(TrackerParam* trackerParamAddr);


private:
    Ptr<BackgroundSubtractorMOG2> MOG2;

	Mat original_frame;


	Special special;		// 1号目标
	vector<Rect> rects;
    vector<Object> objects;

	// 标志位
	bool hasObjFlag;



private:
	cv::Rect findClostAndDel_Rects(const cv::Rect rect);
	cv::Rect findClostAndDel_Abandon(const cv::Rect rect);
	double calculateRectDistance(const cv::Rect& rect1, const cv::Rect& rect2);

};


#endif