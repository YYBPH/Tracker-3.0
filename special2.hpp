#pragma once

#include <opencv2/opencv.hpp>
#include "KalmanFilter.hpp"

/*
SpecialĬ�Ϲ��캯������ʼֵ��x,y,w,h,dx,dy����Ϊ0.
ReinitKalmanFilter����ʼֵ:newRect
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

