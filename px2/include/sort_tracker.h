#pragma once
#include <opencv2/opencv.hpp>
#include <vector>
#include <algorithm>
#include <limits>

// Minimal SORT-style multi-object tracker (header-only)
// - State: [cx, cy, w, h, vx, vy, vw, vh]
// - Association: greedy by IoU with threshold
// - Measurement: [cx, cy, w, h]
// This is intentionally lightweight for quick drop-in tests.

namespace sort {

struct Detection {
    cv::Rect2f box;
    float score = 0.f;
    int class_id = -1;
};

struct Track {
    int id = -1;
    cv::Rect2f box;
    float score = 0.f;
    int class_id = -1;
    int age = 0;
    int hits = 0;
    int time_since_update = 0;
    // Kalman
    cv::KalmanFilter kf;
    cv::Mat meas;
    bool kf_initialized = false;
};

struct Params {
    float iou_thr  = 0.3f;
    int   max_age  = 30;
    int   min_hits = 2;
    bool  class_aware = false; // set true if you want class-aware association
};

class SortTracker {
public:
    explicit SortTracker(const Params& p = Params()) : p_(p) {}

    // Update with this frame's detections, return current visible tracks
    std::vector<Track> update(const std::vector<Detection>& dets) {
        // 1) Predict step for all existing tracks
        for (auto& t : tracks_) {
            t.age++;
            t.time_since_update++;
            if (t.kf_initialized) {
                t.box = predict_box_(t);
            }
        }

        // 2) Greedy IoU association (optionally class-aware)
        const int T = (int)tracks_.size();
        const int D = (int)dets.size();
        std::vector<int> det_assigned(D, -1);
        std::vector<int> trk_assigned(T, -1);

        while (true) {
            float best_iou = p_.iou_thr;
            int best_t = -1, best_d = -1;
            for (int ti=0; ti<T; ++ti) {
                if (trk_assigned[ti] != -1) continue;
                for (int di=0; di<D; ++di) {
                    if (det_assigned[di] != -1) continue;
                    if (p_.class_aware &&
                        tracks_[ti].class_id != -1 &&
                        dets[di].class_id != -1 &&
                        tracks_[ti].class_id != dets[di].class_id) continue;
                    float iou = iou_(tracks_[ti].box, dets[di].box);
                    if (iou > best_iou) { best_iou = iou; best_t = ti; best_d = di; }
                }
            }
            if (best_t == -1 || best_d == -1) break;
            trk_assigned[best_t] = best_d;
            det_assigned[best_d] = best_t;
        }

        // 3) Update matched tracks
        for (int ti=0; ti<T; ++ti) {
            int di = trk_assigned[ti];
            if (di == -1) continue;
            correct_with_(tracks_[ti], dets[di].box);
            tracks_[ti].score = dets[di].score;
            tracks_[ti].class_id = dets[di].class_id;
            tracks_[ti].time_since_update = 0;
            tracks_[ti].hits++;
        }

        // 4) Create new tracks for unmatched detections
        for (int di=0; di<D; ++di) {
            if (det_assigned[di] != -1) continue;
            Track t;
            t.id = next_id_++;
            t.box = dets[di].box;
            t.score = dets[di].score;
            t.class_id = dets[di].class_id;
            init_kf_(t, t.box);
            t.age = 1;
            t.hits = 1;
            t.time_since_update = 0;
            tracks_.push_back(std::move(t));
        }

        // 5) Remove stale tracks
        tracks_.erase(std::remove_if(tracks_.begin(), tracks_.end(),
            [&](const Track& t){ return t.time_since_update > p_.max_age; }),
            tracks_.end());

        // 6) Visible subset
        std::vector<Track> visible;
        visible.reserve(tracks_.size());
        for (auto& t : tracks_) {
            if (t.hits >= p_.min_hits || t.time_since_update == 0) {
                visible.push_back(t);
            }
        }
        return visible;
    }

    const std::vector<Track>& all_tracks() const { return tracks_; }
    void reset() { tracks_.clear(); next_id_ = 1; }

private:
    Params p_;
    int next_id_ = 1;
    std::vector<Track> tracks_;

    static inline cv::Rect2f cxcywh_to_rect(float cx,float cy,float w,float h) {
        return cv::Rect2f(cx - 0.5f*w, cy - 0.5f*h, w, h);
    }
    static inline void rect_to_cxcywh(const cv::Rect2f& r,float& cx,float& cy,float& w,float& h) {
        cx = r.x + r.width*0.5f; cy = r.y + r.height*0.5f; w = r.width; h = r.height;
    }
    static inline float iou_(const cv::Rect2f& a, const cv::Rect2f& b) {
        float inter = (a & b).area();
        float uni = a.area() + b.area() - inter;
        return uni > 0 ? inter/uni : 0.f;
    }

    void init_kf_(Track& t, const cv::Rect2f& box, float dt=1.f) {
        /**
        @brief
        Initialize the Kalman filter for a newly created track.
        @details Creates the 8D state [cx, cy, w, h, vx, vy, vw, vh] and 
        sets the transition/measurement/noise covariances. The initial state is taken from the 
        first measurement (bbox); timing to call is when a track exists but its KF has not 
        been initialized yet.
        **/
        t.kf = cv::KalmanFilter(8,4,0, CV_32F);
        t.kf.transitionMatrix = (cv::Mat_<float>(8,8) <<
            1,0,0,0, dt,0, 0,0,
            0,1,0,0, 0, dt,0,0,
            0,0,1,0, 0,0, dt,0,
            0,0,0,1, 0,0, 0, dt,
            0,0,0,0, 1,0, 0,0,
            0,0,0,0, 0,1, 0,0,
            0,0,0,0, 0,0, 1,0,
            0,0,0,0, 0,0, 0,1);
        t.kf.measurementMatrix = cv::Mat::zeros(4,8,CV_32F);
        t.kf.measurementMatrix.at<float>(0,0)=1;
        t.kf.measurementMatrix.at<float>(1,1)=1;
        t.kf.measurementMatrix.at<float>(2,2)=1;
        t.kf.measurementMatrix.at<float>(3,3)=1;

        setIdentity(t.kf.processNoiseCov,     cv::Scalar(1e-2f));
        setIdentity(t.kf.measurementNoiseCov, cv::Scalar(1e-1f));
        setIdentity(t.kf.errorCovPost,        cv::Scalar(1.f));

        t.meas = cv::Mat::zeros(4,1,CV_32F);
        float cx,cy,w,h; rect_to_cxcywh(box,cx,cy,w,h);
        t.kf.statePost = (cv::Mat_<float>(8,1) << cx,cy,w,h, 0,0,0,0);
        t.kf_initialized = true;
    }

    cv::Rect2f predict_box_(Track& t) {
        /**
        @brief Propagate the track state one step forward (KF predict) and return the predicted box.
        @details Runs kf.predict() to obtain the a priori (predicted) state for this frame 
        and converts it back to a rectangle in image coordinates. The predicted box is used 
        as the reference for association (IoU gating) against detections.
        @note Call order in a frame: predict → associate → correct.
        **/
        cv::Mat pred = t.kf.predict();
        float cx=pred.at<float>(0), cy=pred.at<float>(1);
        float w =pred.at<float>(2), h =pred.at<float>(3);
        return cxcywh_to_rect(cx,cy,w,h);
    }

    void correct_with_(Track& t, const cv::Rect2f& z) {
        /**
        @brief Update the predicted state with a matched detection (KF correct/update).
        @details
        Converts the detection bbox to a measurement [cx, cy, w, h] and applies kf.correct(measurement).
         This pulls the track back to the observed position
        **/
        if (!t.kf_initialized) init_kf_(t, z);
        float cx,cy,w,h; rect_to_cxcywh(z,cx,cy,w,h);
        t.meas.at<float>(0)=cx; t.meas.at<float>(1)=cy;
        t.meas.at<float>(2)=w;  t.meas.at<float>(3)=h;
        cv::Mat est = t.kf.correct(t.meas);
        float ecx=est.at<float>(0), ecy=est.at<float>(1);
        float ew =est.at<float>(2), eh =est.at<float>(3);
        t.box = cxcywh_to_rect(ecx,ecy,ew,eh);
    }
};

} // namespace sort