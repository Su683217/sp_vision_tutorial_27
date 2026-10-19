#include <chrono>
#include <opencv2/opencv.hpp>

#include "io/camera.hpp"
#include "io/gimbal/gimbal.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"

const std::string keys =
  "{help h usage ? | | 输出命令行参数说明}"
  "{@config-path   | | yaml配置文件路径 }";

using namespace std::chrono_literals;

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  auto config_path = cli.get<std::string>("@config-path");
  if (cli.has("help") || !cli.has("@config-path")) {
    cli.printMessage();
    return 0;
  }

  // 初始化工具类
  tools::Exiter exiter;
  tools::Plotter plotter;   // 注意plotter工具的使用

  // 初始化io类
  io::Camera camera(config_path);
  io::Gimbal gimbal(config_path);

  // 初始化auto_aim类
  auto_aim::YOLO yolo(config_path, true);
  auto_aim::Solver solver(config_path);

  cv::Mat img;
  Eigen::Quaterniond q;
  std::chrono::steady_clock::time_point t;

  while (!exiter.exit()) {
    // Your code start

    // 1. 读取图像，并取得拍摄时刻的云台姿态
    camera.read(img, t);
    q = gimbal.q(t);
    solver.set_R_gimbal2world(q);

    // 2. 识别装甲板
    auto armors = yolo.detect(img);
    cv::waitKey(1);  // 刷新 YOLO 的调试窗口

    nlohmann::json data;
    data["armor_num"] = armors.size();

    // 没有识别到装甲板时不控制云台
    if (armors.empty()) {
      gimbal.send(false, false, 0, 0);
      plotter.plot(data);
      continue;
    }

    // 3. 解算装甲板在世界坐标系下的位置
    auto armor = armors.front();
    solver.solve(armor);

    // 4. 计算云台目标角度（下位机约定 pitch 向上为负）
    double yaw = armor.ypd_in_world[0];
    double pitch = -armor.ypd_in_world[1];

    // 5. 控制云台，不开火
    gimbal.send(true, false, yaw, pitch);

    // 6. 输出发送给云台的控制命令，以及云台当前角度用于对比（单位：度）
    auto state = gimbal.state();
    data["cmd_yaw"] = yaw * 57.3;
    data["cmd_pitch"] = pitch * 57.3;
    data["gimbal_yaw"] = state.yaw * 57.3;
    data["gimbal_pitch"] = state.pitch * 57.3;
    data["distance"] = armor.ypd_in_world[2];
    plotter.plot(data);

    // Your code end
  }

  return 0;
}