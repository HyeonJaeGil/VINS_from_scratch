#pragma once
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <vector>

class FeatureDetector
{
public:

    enum class DetectorType{
        SIFT,
        SURF,
        BRIEF,
        ORB
    };

    FeatureDetector() = default;
    FeatureDetector(DetectorType);
    FeatureDetector(cv::Mat& image){image_ = image.clone(); 
        // cv::resize(image_, image_, cv::Size(640, 480));
    }

    void DetectFeatureInImage(cv::Mat image);
    void DetectFeatureInImage();
    void VisualizeDetectedImage();

private:
    cv::Mat image_;
    cv::Ptr<cv::SIFT> detector_= cv::SIFT::create(400);
    std::vector<cv::KeyPoint> keypoints_;
    
};