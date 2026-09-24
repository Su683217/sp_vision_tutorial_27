#ifndef CAMERA_HPP
#define CAMERA_HPP
#include "hikrobot/include/MvCameraControl.h"
#include <opencv2/opencv.hpp>

class Camera
{
public:
    Camera();
    ~Camera();
    void read(cv::Mat & img);

private:
    void * handle_;
    cv::Mat transfer(MV_FRAME_OUT& raw);
};

#endif // CAMERA_HPP