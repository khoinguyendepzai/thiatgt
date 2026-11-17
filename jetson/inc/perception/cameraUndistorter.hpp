#pragma once

#include <opencv2/core.hpp>

namespace perception {

// Calib ống kính camera bám làn (main.cpp: kCameraCalibration). Chỉ đúng với độ phân giải, sensor mode và flip-method
// của kLaneCameraSource; đổi pipeline thì phải calib lại (calib/calib_camera.py).
// Homography (ảnh đã nắn méo -> mặt đường) không còn ở Jetson: PC (main.py, GroundCalibration) dò làn và dùng nó.
struct CameraCalibration {
    cv::Size           imageSize;      // cỡ ảnh camera lúc calib
    cv::Matx33d        cameraMatrix;   // [fx 0 cx; 0 fy cy; 0 0 1] (pixel)
    cv::Vec<double, 5> distCoeffs;     // k1, k2, p1, p2, k3
};

// Thư viện chỉnh quang sai thấu kính camera (Lens Undistortion).
// Tính toán bảng remap một lần trong constructor, sau đó nắn méo từng frame bằng remap().
class CameraUndistorter {
public:
    CameraUndistorter(const cv::Size&           imageSize,
                      const cv::Matx33d&        cameraMatrix,
                      const cv::Vec<double, 5>& distCoeffs);

    explicit CameraUndistorter(const CameraCalibration& calib);

    // Nắn méo frame thô -> ảnh đã nắn thẳng (undistorted)
    void undistort(const cv::Mat& raw, cv::Mat& undistorted) const;
    cv::Mat undistort(const cv::Mat& raw) const;

    const cv::Mat& map1() const { return map1_; }
    const cv::Mat& map2() const { return map2_; }
    cv::Size image_size() const { return imageSize_; }

private:
    void init_maps(const cv::Matx33d& cameraMatrix, const cv::Vec<double, 5>& distCoeffs);

    cv::Size imageSize_;
    cv::Mat  map1_;
    cv::Mat  map2_;
};

} // namespace perception
