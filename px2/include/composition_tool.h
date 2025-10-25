// Rule-of-Thirds Composition Target
#pragma once
#include <opencv2/core.hpp>
#include <string>
#include <cctype>
#include <algorithm>

// --- 三分割(×3) 構図ターゲット(Rule-of-Thirds Composition Target) ---
enum class ThirdsTarget { LEFT, CENTER, RIGHT, AUTO };
static inline ThirdsTarget parseThirdsTarget(std::string s) {
  for (auto& c : s) c = std::tolower(c);
  std::cout << "s is " << s << std::endl;
  if (s == "left")   return ThirdsTarget::LEFT;
  if (s == "right")  return ThirdsTarget::RIGHT;
  if (s == "auto")   return ThirdsTarget::AUTO;
  return ThirdsTarget::CENTER;
}

static inline double thirdsX(int img_w, ThirdsTarget t, double bbox_cx /* >=0 */, bool use_auto_nearest = true) {
  const double left   = img_w / 3.0;
  const double center = img_w / 2.0;
  const double right  = (2.0 * img_w) / 3.0;
  switch (t) {
    case ThirdsTarget::LEFT:   return left;
    case ThirdsTarget::CENTER: return center;
    case ThirdsTarget::RIGHT:  return right;
    case ThirdsTarget::AUTO:
    default:
      if (!use_auto_nearest || bbox_cx < 0.0) return center;
      {
        double xs[3] = { left, center, right };
        double best = center, bd = std::numeric_limits<double>::infinity();
        for (double xi : xs) { double d = std::abs(bbox_cx - xi); if (d < bd) { bd = d; best = xi; } }
        return best;
      }
  }
}

static inline void drawThirdsOverlay(cv::Mat& img,
                                     const cv::Scalar& grid_color = {0,255,255},
                                     const cv::Scalar& mark_color = {0,0,255},
                                     int target_x = -1, int mark_y = -1) {
  const int w = img.cols, h = img.rows;
  const int x1 = w/3, x2 = (2*w)/3, y1 = h/3, y2 = (2*h)/3;
  cv::line(img, {x1,0}, {x1,h}, grid_color, 1, cv::LINE_AA);
  cv::line(img, {x2,0}, {x2,h}, grid_color, 1, cv::LINE_AA);
  cv::line(img, {0,y1}, {w,y1}, grid_color, 1, cv::LINE_AA);
  cv::line(img, {0,y2}, {w,y2}, grid_color, 1, cv::LINE_AA);
  if (target_x >= 0) {
    if (mark_y < 0) mark_y = h/2;
    cv::drawMarker(img, {target_x, mark_y}, mark_color, cv::MARKER_CROSS, 24, 2, cv::LINE_AA);
  }
}
