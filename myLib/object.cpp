#include "object.hpp"



Object::Object()
{
    this->num = 0;
    this->apper_times = 0;
    this->disap_times = 0;
}

Object::~Object()
{
}


void Object::update(cv::Rect newRect)
{
    this->disap_times = 0;
    this->apper_times++;
    this->rect = newRect;
}

void Object::disapper()
{
    this->apper_times = 0;
    this->disap_times++;
}
