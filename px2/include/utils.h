#pragma once
#include <limits>
#include <string>
#include <vector>
#include <algorithm>

/*
cv::Mat frame_bgr = to_bgr8(frame, enforce_bgr8_);
cv::Mat frame_for_update = frame_bgr;
if (global_preproc_enable) {
  applyGlobalPreproc(frame_bgr, gamma_all, clahe_all, sharp_all, frame_for_update);
      }
*/

const double gamma_all  = 2.0;  
const bool   clahe_all  = true; 
const double sharp_all  = 0.1;   

static inline void applyGlobalPreproc(const cv::Mat& src_bgr,
                                      double gamma, bool use_clahe, double sharp_amount,
                                      cv::Mat& dst_bgr)
{
  CV_Assert(!src_bgr.empty() && src_bgr.type() == CV_8UC3);
  dst_bgr = src_bgr.clone();
  // gamma
  if (std::abs(gamma - 1.0) > 1e-3) {
    cv::Mat f; dst_bgr.convertTo(f, CV_32F, 1.0/255.0);
    cv::pow(f, gamma, f);
    f.convertTo(dst_bgr, CV_8U, 255.0);
  }

  // CLAHE（L*チャンネルのみ）
  if (use_clahe) {
    cv::Mat lab; cv::cvtColor(dst_bgr, lab, cv::COLOR_BGR2Lab);
    std::vector<cv::Mat> ch; cv::split(lab, ch);
    auto clahe = cv::createCLAHE(2.0, cv::Size(8,8)); // clip=2.0, tile=8x8
    clahe->apply(ch[0], ch[0]);
    cv::merge(ch, lab);
    cv::cvtColor(lab, dst_bgr, cv::COLOR_Lab2BGR);
  }

  if (sharp_amount > 1e-6) {
    cv::Mat blur; cv::GaussianBlur(dst_bgr, blur, cv::Size(0,0), 1.0);
    cv::addWeighted(dst_bgr, 1.0 + sharp_amount, blur, -sharp_amount, 0.0, dst_bgr);
  }
}


// Convert any common frame to CV_8UC3 (BGR). If enforce_bgr8=false, returns src.
static inline cv::Mat to_bgr8(const cv::Mat& src, bool enforce_bgr8)
{
  if (!enforce_bgr8) return src;
  if (src.type() == CV_8UC3) return src;

  cv::Mat tmp = src;
  // 1) Convert depth to 8U if needed
  if (src.depth() != CV_8U) {
    double alpha = 1.0, beta = 0.0;
    if (src.depth() == CV_16U)      alpha = 1.0 / 256.0;  // 16U -> 8U
    else if (src.depth() == CV_32F) alpha = 255.0;        // 32F -> 8U
    src.convertTo(tmp, CV_8U, alpha, beta);
  }

  // 2) Convert channels to 3 (BGR)
  cv::Mat dst;
  if (tmp.type() == CV_8UC2) {                     // YUYV (YUY2)
    cv::cvtColor(tmp, dst, cv::COLOR_YUV2BGR_YUY2);
    return dst;
  }
  const int ch = tmp.channels();
  if (ch == 3) {
    dst = tmp;                                      // already BGR8
  } else if (ch == 1) {
    cv::cvtColor(tmp, dst, cv::COLOR_GRAY2BGR);
  } else if (ch == 4) {
    cv::cvtColor(tmp, dst, cv::COLOR_BGRA2BGR);
  } else {
    // Fallback: keep first 3 channels
    std::vector<cv::Mat> channels;
    cv::split(tmp, channels);
    while (channels.size() < 3) channels.push_back(channels.back());
    cv::merge(std::vector<cv::Mat>{channels[0], channels[1], channels[2]}, dst);
  }
  return dst;
}


