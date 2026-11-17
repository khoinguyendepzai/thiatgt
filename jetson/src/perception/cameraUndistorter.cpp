#include <perception/cameraUndistorter.hpp>

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

namespace perception {

CameraUndistorter::CameraUndistorter(const cv::Size&           imageSize,
                                     const cv::Matx33d&        cameraMatrix,
                                     const cv::Vec<double, 5>& distCoeffs)
    : imageSize_(imageSize)
{
    init_maps(cameraMatrix, distCoeffs);
}

CameraUndistorter::CameraUndistorter(const CameraCalibration& calib)
    : imageSize_(calib.imageSize)
{
    init_maps(calib.cameraMatrix, calib.distCoeffs);
}

void CameraUndistorter::init_maps(const cv::Matx33d& cameraMatrix, const cv::Vec<double, 5>& distCoeffs) {
    cv::initUndistortRectifyMap(cv::Mat(cameraMatrix), cv::Mat(distCoeffs), cv::noArray(),
                                cv::Mat(cameraMatrix), imageSize_, CV_16SC2,
                                map1_, map2_);
}

void CameraUndistorter::undistort(const cv::Mat& raw, cv::Mat& undistorted) const {
    cv::remap(raw, undistorted, map1_, map2_, cv::INTER_LINEAR);
}

cv::Mat CameraUndistorter::undistort(const cv::Mat& raw) const {
    cv::Mat result;
    undistort(raw, result);
    return result;
}

} // namespace perception

