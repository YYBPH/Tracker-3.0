#ifndef __OBJECT_TRACKER2_H
#define  __OBJECT_TRACKER2_H

#include <opencv2/opencv.hpp>
#include <vector>
#include <float.h>

// #include "Special.hpp"
#include "special.hpp"
#include "object.hpp"

using namespace std;
using namespace cv;



class ObjectsTracker
{
public:
	ObjectsTracker();
	~ObjectsTracker();

	cv::Mat tracker(Mat newframe);


private:
    Ptr<BackgroundSubtractorMOG2> MOG2;

	Mat original_frame;

	vector<Rect> rects;

	Special special;		// 1号目标
    vector<Object> objects;

	// 标志位
	bool hasObjFlag;

	// 阈值
	double maxDist;


	

private:
	cv::Rect findClostAndDel_Rects(const cv::Rect rect);
	cv::Rect findClostAndDel_Objects(const cv::Rect rect);
	double calculateRectDistance(const cv::Rect& rect1, const cv::Rect& rect2);

};


#endif