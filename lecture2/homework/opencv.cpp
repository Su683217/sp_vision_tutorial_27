#include "io/camera.hpp"
#include "tasks/yolo.hpp"
#include "tasks/apriltag_detector.hpp"
#include "opencv2/opencv.hpp"
#include "tools/img_tools.hpp"

int main()
{
    // 初始化相机、yolo类
    Camera camera;
    auto_charge::AprilTagDetector detector("./configs/yolo.yaml");
    cv::Mat img;

    while (1) {
        // 调用相机读取图像
        camera.read(img);
        if (img.empty()) {
            continue;
        }

        // 调用yolo识别opencv标志
        auto tags = detector.detect(img);

        for (const auto & tag : tags) {
            tools::draw_points(img, tag.corners, {0, 255, 0});
            tools::draw_text(img, std::to_string(tag.id), tag.center, {0, 255, 0});
        }

        // 显示图像
        cv::resize(img, img , cv::Size(640, 480));
        cv::imshow("img", img);
        if (cv::waitKey(0) == 'q') {
            break;
        }
    }

    return 0;
}
