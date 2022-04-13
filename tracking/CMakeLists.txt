project(feature_detector)
cmake_minimum_required(VERSION 3.16)
find_package(OpenCV 3 REQUIRED)

add_executable(feature_detector_test feature_detector_test.cc feature_detector.cc)
target_link_libraries(feature_detector_test ${OpenCV_LIBS})