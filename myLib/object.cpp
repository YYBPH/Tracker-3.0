#include "object.hpp"



Object::Object()
{
    this->num = 0;
    this->apper_times = 0;
    this->disap_times = 0;
    this->speed = -1;
    this->lastRect = cv::Rect(0, 0, 0, 0);
}

Object::~Object()
{
}


void Object::update(cv::Rect newRect)
{
    this->disap_times = 0;
    this->apper_times++;

    this->lastRect = this->rect;
    this->rect = newRect;

    double dx = this->rect.x - this->lastRect.x;
    double dy = this->rect.y - this->lastRect.y;
    this->speed = sqrt(dx * dx + dy * dy);
}

void Object::disapper()
{
    this->apper_times = 0;
    this->disap_times++;
}
