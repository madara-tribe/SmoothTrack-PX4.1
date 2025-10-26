/*
cv::Mat frame_bgr = to_bgr8(frame, enforce_bgr8_);
cv::Mat frame_for_update = frame_bgr;
if (global_preproc_enable) {
  applyGlobalPreproc(frame_bgr, gamma_all, clahe_all, sharp_all, frame_for_update);
      }
bool ok = tracker && tracker->update(frame_for_update, track_box);
*/

const bool global_preproc_enable = false;
const double gamma_all  = 2.0;  
const bool   clahe_all  = false; 
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
