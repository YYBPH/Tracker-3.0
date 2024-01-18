#ifndef __OBJECT_HPP
#define __OBJECT_HPP

#include <opencv2/opencv.hpp>
#include "KalmanFilter.hpp"

/*
Special默认构造函数：初始值（x,y,w,h,dx,dy）都为0.
ReinitKalmanFilter：初始值:newRect
*/

class Special
{
public:
	Special();
	~Special();

	double speed;
	int disap_times;
	cv::Rect newRect;  
    MyKalmanFilter kalman;


	void ReinitKalmanFilter(cv::Rect newRect);
	void kalmanFilter(cv::Rect newRect);

private:

};

#endif // !__OBJECT_HPP