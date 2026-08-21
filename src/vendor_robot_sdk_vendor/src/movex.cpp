#include <array>
#include <chrono>
#include <cstring>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

extern "C" {
#include "basestruct.h"
#include "robotapi.h"
}

namespace {
const char * result_name(CRresult result)
{
  switch (result) {
    case success: return "success";
    case operate_timeout: return "operation timed out";
    case robotmode_error: return "invalid robot mode";
    case para_error: return "invalid parameter";
    case no_handle: return "invalid handle";
    default: return "SDK error";
  }
}

bool check(CRresult result, const char * operation)
{
  if (result == success) return true;
  std::cerr << operation << " failed (" << static_cast<int>(result) << ": "
            << result_name(result) << ")\n";
  return false;
}
}  // namespace

int main(int argc, char ** argv)
{
  const std::string ip = argc > 1 ? argv[1] : "192.168.6.6";
  const int port = argc > 2 ? std::stoi(argv[2]) : 2323;
  const std::string password = argc > 3 ? argv[3] : "";

  RobotHandle handle = 0;
  if (!check(cr_create_robot(&handle, ip.c_str(), port, password.c_str()), "connect")) return 1;
  struct HandleGuard {
    RobotHandle handle;
    ~HandleGuard() { if (handle != 0) cr_destroy_robot(handle); }
  } guard{handle};

  std::array<double, ROB_AXIS_NUM> joints{};
  std::array<double, ROB_AXIS_NUM> start_pose{};
  if (!check(cr_get_jointActualPos(handle, joints.data()), "read joint position") ||
    !check(cr_get_tcpActualPose(handle, start_pose.data()), "read TCP pose")) return 1;

  constexpr int point_count = 5;
  std::vector<PathPoint> points(static_cast<size_t>(point_count));
  for (int i = 0; i < point_count; ++i) {
    std::memcpy(points[i].pose, start_pose.data(), sizeof(points[i].pose));
    std::memcpy(points[i].jointpos, joints.data(), sizeof(points[i].jointpos));
    points[i].pose[0] += (i % 2 == 0 ? 50.0 : -50.0);
    points[i].pose[1] += 50.0 * i;
  }

  PathData path_data{};
  path_data.pathPoints = points.data();
  path_data.pathPointsNum = point_count;
  path_data.exjNum = 0;
  PathDownloadData download{};
  download.pathData = path_data;
  download.pathPara.index = 1;
  download.pathPara.moveType = 3;  // Movex
  download.pathPara.speed = 50.0;  // mm/s
  download.pathPara.acc = 100.0;   // mm/s^2
  download.pathPara.blendRadius = 10.0;

  if (!check(cr_path_download(handle, download), "download Movex path") ||
    !check(cr_path_action(handle, download.pathPara.index, 1), "start Movex path")) return 1;

  std::cout << "Movex path started on " << ip << ".\n";
  std::this_thread::sleep_for(std::chrono::seconds(5));
  if (!check(cr_path_action(handle, download.pathPara.index, 0), "stop Movex path")) return 1;
  return 0;
}
