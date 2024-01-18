#include "objectTracker.hpp"

// 腐蚀内核
Mat kernel_erode = (Mat_<uchar>(3, 3) <<  0, 1, 0,
                                          1, 0, 0,
                                          0, 0, 0);

Mat kernel_dilate1 = (Mat_<uchar>(3, 3) << 0, 1, 0,
                                          1, 0, 0,
                                          0, 0, 0);
// 膨胀内核
Mat kernel_dilate2 = (Mat_<uchar>(5, 5) <<  1, 1, 1, 1, 1,
                                           1, 1, 1, 1, 1,
                                           1, 1, 1, 1, 1,
                                           1, 1, 1, 1, 1,
                                           1, 1, 1, 1, 1);

ObjectsTracker::ObjectsTracker()
{
    this->MOG2 = createBackgroundSubtractorMOG2();
    this->hasObjFlag = false;
}

ObjectsTracker::~ObjectsTracker()
{
    
}

cv::Mat ObjectsTracker::tracker(TrackerParam* trackerParamAddr)
{
    this->original_frame = trackerParamAddr->newFrame;

    /**** 1. rects  *************************************/
    Mat grayFrame, mog2MaskFrame, erodeFrame, dilateFrame;
    this->MOG2->apply(this->original_frame, mog2MaskFrame, 0.01);
    erode(mog2MaskFrame, erodeFrame, kernel_erode, Point(-1, -1), 1, cv::BORDER_CONSTANT, cv::Scalar(0));
     
    /**** 1.5 屏蔽区  *************************************/
    for (size_t i = 0; i < trackerParamAddr->MaskRects.size(); i++)
    {
        cv::rectangle(erodeFrame, trackerParamAddr->MaskRects[i], BLACK, -1);
    }

    this->rects.clear();
    vector<vector<Point>> contours;
    findContours(erodeFrame, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
    for (size_t i = 0; i < contours.size(); i++) 
    {
        if (contourArea(contours[i]) < 4) continue;     
        Rect rect = boundingRect(contours[i]);
        this->rects.push_back(rect);
    }

    /** 2. 1号目标  *************************************/
    // 1号丢失处理 说明屏幕一个目标都没有，不用执行1号更新，切换命令
    if (this->hasObjFlag == false)
    {
        for (size_t i = 0; i < objects.size(); i++)
        {
            if (this->objects[i].apper_times > 0)
            {
                // 重置
                hasObjFlag = true;
                special.ReinitKalmanFilter(this->objects[i].rect);
                // 删除
                swap(this->objects[i], this->objects[this->objects.size() - 1]);
                this->objects.pop_back();
                printf("find special\r\n");
                break;
            }
        }
    }
    else
    {
        // 1号切换处理
        if (trackerParamAddr->abandonFlag)
        {
            trackerParamAddr->abandonFlag = false;
            printf("abandoning...\r\n");
            cv::Rect rect = findClostAndDel_Abandon(this->special.newRect);
            if (rect.width != 0 && rect.height != 0)
            {
                // 加
                Object object;
                object.num = -1;
                object.update(this->special.newRect);
                this->objects.push_back(object);
                printf("abandoned\r\n");
                // 更换
                special.ReinitKalmanFilter(rect);
            }
            else
                printf("disabandoned\r\n");
        }

        // 1号更新
        cv::Rect rect = findClostAndDel_Rects(this->special.newRect);
        if (rect.width != 0 && rect.height != 0)
        {
            this->special.disap_times = 0;
            this->special.kalmanFilter(rect);
        }
        else
        {
            this->special.kalmanFilter(this->special.newRect);
            this->special.disap_times++;
            printf("disap_times:%d\r\n", this->special.disap_times);
            if (this->special.disap_times > 5)
            {
                this->hasObjFlag = false;
                printf("special lost\r\n");
            }
        }

    }


    /** 3. objects  *************************************/
    // 更新
    for (size_t i = 0; i < objects.size(); i++)
    {
        cv::Rect rect = findClostAndDel_Rects(objects[i].rect);
        if (rect.width != 0 && rect.height != 0)
            objects[i].update(rect);
        else
            objects[i].disapper();
    }
    // 删除
    for (size_t i = 0; i < objects.size(); i++)
    {
        if (objects[i].disap_times > 5)
        {
            
            // earse时间复杂度太高
            swap(objects[i], objects[objects.size() - 1]);
            objects.pop_back();
            i--;
        }
    }
    // 新增
    for (size_t i = 0; i < this->rects.size(); i++)
    {
        if (this->rects.size() > MAX_NUM) break;
        Object object;
        object.update(this->rects[i]);
        this->objects.push_back(object);
    }

    /** 4. drawInfo  *************************************/
    for (size_t i = 0; i < this->objects.size(); i++)
    {
        if (this->objects[i].apper_times > 0) {
            cv::putText(original_frame, std::to_string(objects[i].num), cv::Point(objects[i].rect.x, objects[i].rect.y), cv::FONT_HERSHEY_SIMPLEX, 1, 0, 2);
            cv::rectangle(this->original_frame, this->objects[i].rect, BLUE, 2);
        }
         else
             cv::rectangle(this->original_frame, this->objects[i].rect, GREEN, 2);

        if (this->objects[i].num == -1)
            cv::rectangle(this->original_frame, this->objects[i].rect, BLACK, 2);         

    }

    if (hasObjFlag)
        cv::rectangle(this->original_frame, this->special.newRect, RED, 2);

    for (size_t i = 0; i < trackerParamAddr->MaskRects.size(); i++)
    {
        cv::rectangle(this->original_frame, trackerParamAddr->MaskRects[i], CYAN, 2);
    }


    return this->original_frame;
   

}



// 在this->rects中找到最近的方框,返回并删除
cv::Rect ObjectsTracker::findClostAndDel_Rects(const cv::Rect rect)
{
    double clostDist = DBL_MAX;
    Rect clostRect;

    int index = -1;
    for(size_t i = 0; i < this->rects.size(); i++)
    {
        double dist = calculateRectDistance(rect, this->rects[i]);
        if(dist < clostDist)
        {
            index = i;
            clostDist = dist;
            clostRect = this->rects[i];
        }
    }

    if(clostDist < MAX_DIST)
    {
        // earse时间复杂度太高
        swap(this->rects[index], this->rects[this->rects.size()-1]);
        this->rects.pop_back();
        return clostRect;
    }
    else
    {
        clostRect.width = 0;
        clostRect.height = 0;
        return clostRect;
    }
}


cv::Rect ObjectsTracker::findClostAndDel_Abandon(const cv::Rect rect)
{
    double clostDist = DBL_MAX;
    Rect clostRect;

    int index = -1;
    for (size_t i = 0; i < this->objects.size(); i++)
    {
        if (this->objects[i].num != -1)
        {
            double dist = calculateRectDistance(rect, this->objects[i].rect);
            if (dist < clostDist)
            {
                index = i;
                clostDist = dist;
                clostRect = this->objects[i].rect;
            }
        }
    }

    if (clostDist < DBL_MAX)
    {
        // earse时间复杂度太高
        swap(this->objects[index], this->objects[this->objects.size() - 1]);
        this->objects.pop_back();
        return clostRect;
    }
    else
    {
        clostRect.width = 0;
        clostRect.height = 0;
        return clostRect;
    }
}




// 计算两个矩形的中心点之间的欧氏距离
double ObjectsTracker::calculateRectDistance(const cv::Rect& rect1, const cv::Rect& rect2) {
    cv::Point2d center1(rect1.x + rect1.width / 2.0, rect1.y + rect1.height / 2.0);
    cv::Point2d center2(rect2.x + rect2.width / 2.0, rect2.y + rect2.height / 2.0);

    // 计算欧氏距离
    double distance = cv::norm(center1 - center2);

    return distance;
}


