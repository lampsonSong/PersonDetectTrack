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

std::vector<double> calc_variance(const std::vector<cv::Point2d> &kp){
  int sz = kp.size();
  int non_zero = 0;
  double variance[2]={0,0};  
  double sum_x = 0,sum_y = 0;
  std::vector<double>var_;
  std::vector<double>dis;
  std::vector<double>kp_x;
  std::vector<double>kp_y;
  cv::Point2d median_val;
  std::vector<cv::Point2d>sel_kp;
  
  for(int i=0;i<sz;i++)
  {
    if(kp[i].x!=0&&kp[i].y!=0)
    { 
      kp_x.push_back(kp[i].x);
      kp_y.push_back(kp[i].y);
    }
  }
  
  std::sort(kp_x.begin(),kp_x.end());
  std::sort(kp_y.begin(),kp_y.end());

  if(kp_x.size()!=0&&kp_y.size()!=0)
  {
    if(kp_x.size()%2==0)
    {
      median_val.x = (kp_x[kp_x.size()/2]+kp_x[kp_x.size()/2-1])/2;
    }else{
      median_val.x = kp_x[kp_x.size()/2];
    }

    if(kp_y.size()%2==0)
    {
      median_val.y = (kp_y[kp_y.size()/2]+kp_y[kp_y.size()/2-1])/2;
    }else{
      median_val.y = kp_y[kp_y.size()/2];
    }
  }
  
  for(int i=0;i<kp.size();i++)
  {
    double diff_x,diff_y;
    diff_x = std::abs(kp[i].x-median_val.x);
    diff_y = std::abs(kp[i].y-median_val.y);
    if(diff_x<150 && diff_y<150 && kp[i].x>0 && kp[i].y>0)
    {
      sel_kp.push_back(kp[i]);
    }
  }
  
  for(int i = 0; i < sel_kp.size(); i++)
  {   
    if(sel_kp[i].x!=0 && sel_kp[i].y!=0 && median_val.x!=0 && median_val.y!=0)
    { 
      sum_x += std::pow(sel_kp[i].x-median_val.x,2);
      sum_y += std::pow(sel_kp[i].y-median_val.y,2);
      non_zero++;
    }
  }
  
  if(non_zero>1)
  {
    sum_x = std::sqrt(sum_x);
    sum_y = std::sqrt(sum_y);
    variance[0] = sum_x/(non_zero-1);
    variance[1] = sum_y/(non_zero-1);  
  }
  
  var_.push_back(variance[0]);
  var_.push_back(variance[1]);
  
  return var_;
}
