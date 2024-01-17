#ifndef __OTHERS_HPP
#define __OTHERS_HPP

#include <opencv2/opencv.hpp>


class Object
{
public:
    int num;
    bool isUpdate;
    int apper_times;
    int disap_times;
    cv::Rect rect;
    
    Object();
    ~Object();

    void update(cv::Rect newRect);
    void disapper();
private:
};



#endif // !__OTHERS_HPP