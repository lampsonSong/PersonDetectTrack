#include <opencv2/core.hpp>

int rectPerimeter(const cv::Rect &rect);

bool rectContainsRect(const cv::Rect &parent, const cv::Rect &child);

bool rectIntersectsRect(const cv::Rect &rect1, const cv::Rect &rect2);

cv::Rect rectUnion(const cv::Rect &rect1, const cv::Rect &rect2);

float rectOverlap(const cv::Rect &rect1, const cv::Rect &rect2);

float rectOverlap_enhanced(const cv::Rect &rect1, const cv::Rect &rect2);

float distance(const cv::Point2f &p1, const cv::Point2f &p2);

float angle_cos(const cv::Point2f &p1, const cv::Point2f &p2);

float norm2(const cv::Point2f &p);

float angleCosComputation(const cv::Point2f &p1, const cv::Point2f &p2);

std::vector<double> calc_variance(const std::vector<cv::Point2d> &kp);
