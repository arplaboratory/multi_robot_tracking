#ifndef _DETECTION_H_
#define _DETECTION_H_

#include <iostream>
#include <opencv2/opencv.hpp>


class Detection
{
public:
  Detection(const float& _x, const float& _y, const int& _w, const int& _h)
    : m_x(_x), m_y(_y), m_w(_w), m_h(_h)
  {
    point = cv::Point2f(m_x, m_y);
    bbox = cv::Rect(m_x - (m_w >> 1), m_y - m_h, m_w, m_h);
  }
  const float x() const { return m_x; }
  const float y() const { return m_y; }
  const float w() const { return m_w; }
  const float h() const { return m_h; }
  const cv::Rect getRect() const { return bbox; }
  const cv::Point2f operator()() const
  {
    return point;
  }
  const Eigen::Vector2f getVect() const
  {
    Eigen::Vector2f p;
    p << x(), y();
    return p;
  }
  Detection& operator=(const Detection& d_copy)
  {
    this->m_x = d_copy.x();
    this->m_y = d_copy.y();
    this->m_w = d_copy.w();
    this->m_h = d_copy.h();
    return *this;
  }
private:
  float m_x, m_y;
  int m_w, m_h;
  cv::Point2f point;
  cv::Rect bbox;
};


#endif
