#include <chrono>
#include <opencv2/opencv.hpp>

#include "io/camera.hpp"
#include "io/gimbal/gimbal.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tasks/auto_aim/target.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"
#include "tools/trajectory.hpp"
#include "tools/yaml.hpp"

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
  tools::Plotter plotter;  // 注意plotter工具的使用

  // 初始化io类
  io::Camera camera(config_path);
  io::Gimbal gimbal(config_path);

  // 初始化auto_aim类
  auto_aim::YOLO yolo(config_path, true);
  auto_aim::Solver solver(config_path);

  cv::Mat img;
  Eigen::Quaterniond q;
  std::chrono::steady_clock::time_point t;

  // 读取瞄准和开火参数（角度单位：度）
  auto yaml = tools::load(config_path);
  auto yaw_offset = tools::read<double>(yaml, "yaw_offset") / 57.3;
  auto pitch_offset = tools::read<double>(yaml, "pitch_offset") / 57.3;
  auto predict_delay = tools::read<double>(yaml, "predict_delay");
  auto fire_threshold = tools::read<double>(yaml, "fire_threshold") / 57.3;

  auto_aim::Target target;
  bool tracking = false;
  std::chrono::steady_clock::time_point track_start, last_seen;

  while (!exiter.exit()) {
    // Your code start

    // 1. 读取图像，并取得拍摄时刻的云台姿态
    camera.read(img, t);
    q = gimbal.q(t);
    solver.set_R_gimbal2world(q);

    // 2. 识别装甲板，跟踪中只接受同一辆车的装甲板
    auto armors = yolo.detect(img);
    cv::waitKey(1);  // 刷新 YOLO 的调试窗口

    auto_aim::Armor * armor = nullptr;
    for (auto & a : armors) {
      if (!tracking || a.name == target.name) {
        armor = &a;
        break;
      }
    }

    // 3. 用 EKF 跟踪整车：首次看到时初始化，之后每帧先预测再更新
    if (armor != nullptr) {
      solver.solve(*armor);
      if (!tracking) {
        target = auto_aim::Target(*armor, t);
        tracking = true;
        track_start = t;
      } else {
        target.predict(t);
        target.update(*armor);
      }
      last_seen = t;
    } else if (tracking) {
      target.predict(t);  // 换板时短暂看不到装甲板，只做预测
    }

    // 丢失超过 0.5 s 或滤波发散时，放弃当前目标
    if (tracking && (tools::delta_time(t, last_seen) > 0.5 || target.diverged())) {
      tracking = false;
    }

    nlohmann::json data;
    data["armor_num"] = armors.size();
    data["tracking"] = tracking ? 1 : 0;

    if (!tracking) {
      gimbal.send(false, false, 0, 0);
      plotter.plot(data);
      continue;
    }

    // 4. 预测子弹到达时整车的状态，选出最正对我们的装甲板作为瞄准点
    // 飞行时间取决于瞄准点，瞄准点又取决于飞行时间，迭代几次求解
    auto bullet_speed = gimbal.state().bullet_speed;
    if (bullet_speed < 10) bullet_speed = 23;  // 下位机未上报弹速时使用默认值

    Eigen::Vector4d aim_xyza;
    double fly_time = 0;
    double traj_pitch = 0;
    bool solvable = true;
    for (int i = 0; i < 3; i++) {
      auto future = target;
      future.predict(predict_delay + fly_time);

      auto center = future.ekf_x();
      auto center_yaw = std::atan2(center[2], center[0]);
      double min_error = 1e10;
      for (const auto & xyza : future.armor_xyza_list()) {
        auto error = std::abs(tools::limit_rad(xyza[3] - center_yaw));
        if (error < min_error) {
          min_error = error;
          aim_xyza = xyza;
        }
      }

      tools::Trajectory traj(bullet_speed, aim_xyza.head(2).norm(), aim_xyza[2]);
      solvable = !traj.unsolvable;
      if (!solvable) break;
      fly_time = traj.fly_time;
      traj_pitch = traj.pitch;
    }

    if (!solvable) {
      gimbal.send(false, false, 0, 0);
      plotter.plot(data);
      continue;
    }

    // 5. 计算云台目标角度（下位机约定 pitch 向上为负）
    double yaw = tools::limit_rad(std::atan2(aim_xyza[1], aim_xyza[0]) + yaw_offset);
    double pitch = -(traj_pitch + pitch_offset);

    // 6. 开火判断：跟踪超过 1 s 使转速收敛，且云台 yaw 已对准瞄准点
    auto gimbal_yaw = tools::eulers(q, 2, 1, 0)[0];
    auto yaw_error = tools::limit_rad(gimbal_yaw - yaw);
    bool fire = target.is_converged() && tools::delta_time(t, track_start) > 1.0 &&
                std::abs(yaw_error) < fire_threshold;

    gimbal.send(true, fire, yaw, pitch);

    // 7. 输出控制命令和 EKF 拟合的角速度（角度单位：度）
    auto x = target.ekf_x();
    data["cmd_yaw"] = yaw * 57.3;
    data["cmd_pitch"] = pitch * 57.3;
    data["gimbal_yaw"] = gimbal_yaw * 57.3;
    data["yaw_error"] = yaw_error * 57.3;
    data["fire"] = fire ? 1 : 0;
    data["w"] = x[7];
    data["r"] = x[8];
    data["distance"] = aim_xyza.head(3).norm();
    data["fly_time"] = fly_time;
    data["bullet_speed"] = bullet_speed;
    plotter.plot(data);

    // Your code end
  }

  return 0;
}