#include "feature_detector.h"

void FeatureDetector::DetectFeatureInImage()
{
    if(image_.empty()) return;
    detector_->detect(image_, keypoints_);
}

void FeatureDetector::VisualizeDetectedImage()
{
    cv::drawKeypoints(image_, keypoints_, image_);
    cv::imshow("test", image_);
    cv::waitKey();
}

