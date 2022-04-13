#include "feature_detector.h"


int main()
{
    cv::Mat input_image = cv::imread("/home/hj/Pictures/test.jpg");

    FeatureDetector feature_detector(input_image);
    feature_detector.DetectFeatureInImage();
    feature_detector.VisualizeDetectedImage();

    return 0;
}