#include "utils.h"
#include<iostream>
int rectPerimeter(const cv::Rect &rect) {
  return (rect.width + rect.height) * 2;
}

bool rectContainsRect(
    const cv::Rect &parent,
    const cv::Rect &child) // contains or equals, but not associative
{
  return parent.contains(cv::Point(child.x, child.y)) &&
         parent.contains(
             cv::Point(child.x + child.width - 1, child.y + child.height - 1));
}

bool rectIntersectsRect(const cv::Rect &rect1, const cv::Rect &rect2) {
  return rect1.x <= rect2.x + rect2.width && rect1.x + rect1.width >= rect2.x &&
         rect1.y <= rect2.y + rect2.height && rect1.y + rect1.height >= rect2.y;
}

cv::Rect rectUnion(const cv::Rect &rect1,
                   const cv::Rect &rect2) // similar to cvMaxRect
{
  cv::Rect unionRect;
  unionRect.x = std::min(rect1.x, rect2.x);
  unionRect.y = std::min(rect1.y, rect2.y);
  unionRect.width =
      std::max(rect1.x + rect1.width, rect2.x + rect2.width) - unionRect.x;
  unionRect.height =
      std::max(rect1.y + rect1.height, rect2.y + rect2.height) - unionRect.y;
  return unionRect;
}

float rectOverlap(const cv::Rect &rect1, const cv::Rect &rect2) {
  float intArea = (rect1 & rect2).area();
  float unionArea = rect1.area() + rect2.area() - intArea;

  return intArea / unionArea;
}

float rectOverlap_enhanced(const cv::Rect &rect1, const cv::Rect &rect2) {
  float intArea = (rect1 & rect2).area();
  float unionArea = rect1.area() + rect2.area() - intArea;
  float cost = intArea / unionArea;
  cost += intArea / rect1.area();
  cost += intArea / rect2.area();
  return cost / 3.0;
}

float distance(const cv::Point2f &p1, const cv::Point2f &p2) {
  cv::Point2f p = p1 - p2;
  return sqrt(p.x * p.x + p.y * p.y);
}

float angle_cos(const cv::Point2f &p1, const cv::Point2f &p2) {
  return (p1.x * p2.x + p1.y * p2.y) / (std::sqrt(p1.x * p1.x + p1.y * p1.y) *
                                        std::sqrt(p2.x * p2.x + p2.y * p2.y));
}

float norm2(const cv::Point2f &p) { return std::sqrt(p.x * p.x + p.y * p.y); }

float angleCosComputation(const cv::Point2f &p1, const cv::Point2f &p2) {
  return std::acos( (p1.x * p2.x + p1.y * p2.y) / (std::sqrt(p1.x * p1.x + p1.y * p1.y) *
                                        std::sqrt(p2.x * p2.x + p2.y * p2.y)) ) * (180/M_PI);
}
