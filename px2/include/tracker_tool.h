
#pragma once
#include <opencv2/tracking.hpp>
#include <opencv2/tracking/tracking_legacy.hpp>
#include <limits>
#include <string>
#include <vector>
#include <algorithm>
#include <cctype>

// ---- helpers to map string to KCF modes safely ----
static inline int kcfModeCN() {
#ifdef CV_VERSION_EPOCH
  return cv::TrackerKCF::CN;
#else
  return cv::TrackerKCF::CN;
#endif
}
static inline int kcfModeGRAY() {
#ifdef CV_VERSION_EPOCH
  return cv::TrackerKCF::GRAY;
#else
  return cv::TrackerKCF::GRAY;
#endif
}

struct KcfTuning {
  bool   enable              = true;   // enable tuned params instead of OpenCV defaults
  double interp_factor       = 0.012;  // slower model update -> less drift
  double detect_thresh       = 0.6;    // higher confidence to accept response
  double sigma               = 0.9;
  double output_sigma_factor = 0.1;
  double lambda              = 0.0001;
  bool   compress_feature    = true;   // feature PCA compression
  int    compressed_size     = 2;
  bool   use_cn              = true;   // use Color-Names in PCA branch
  bool   resize              = true;
  int    max_patch_size      = 80*80;  // limit ROI to keep memory bounded
} static g_kcf;


static cv::Ptr<cv::Tracker> make_tracker(const std::string& type)
{
  cv::Ptr<cv::Tracker> t;
  try {
    std::string up = type;
    std::transform(up.begin(), up.end(), up.begin(), ::toupper);
    if (up == "KCF")          {
      // Use tuned parameters if enabled
      if (g_kcf.enable) {
        cv::TrackerKCF::Params p;
        p.interp_factor       = g_kcf.interp_factor;
        p.detect_thresh       = g_kcf.detect_thresh;
        p.sigma               = g_kcf.sigma;
        p.output_sigma_factor = g_kcf.output_sigma_factor;
        p.lambda              = g_kcf.lambda;
        p.compress_feature    = g_kcf.compress_feature;
        p.compressed_size     = g_kcf.compressed_size;
        p.desc_npca           = kcfModeGRAY();
        p.desc_pca            = g_kcf.use_cn ? kcfModeCN() : kcfModeGRAY();
        p.resize              = g_kcf.resize;
        p.max_patch_size      = g_kcf.max_patch_size;
        t = cv::TrackerKCF::create(p);
        std::cout << "KCF con param es usado" << std::endl; 
      } else {
        t = cv::TrackerKCF::create();
      }
    }
    else if (up == "MOSSE")    t = cv::legacy::upgradeTrackingAPI(cv::legacy::TrackerMOSSE::create());
  } catch (const cv::Exception& e) {
    std::cerr << "Tracker creation failed (" << type << "): " << e.what() << std::endl;
    t.release();
  } catch (...) {
    std::cerr << "Tracker creation failed (" << type << "): unknown exception" << std::endl;
    t.release();
  }
  return t;  // may be empty if not available in your build
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


