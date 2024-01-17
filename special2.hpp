#pragma once

#include <opencv2/opencv.hpp>
#include "KalmanFilter.hpp"

/*
Special默认构造函数：初始值（x,y,w,h,dx,dy）都为0.
ReinitKalmanFilter：初始值:newRect
*/

class Special2
{
public:
	Special2();
	~Special2();

	MyKalmanFilter kalman;
	int lastX;
	int lastY;
	cv::Rect newRect;
	int disap_times;


	void ReinitKalmanFilter(cv::Rect newRect);
	void kalmanFilter(cv::Rect newRect);

private:

};

