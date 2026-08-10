#define private public
#include "vendor_robot_driver_core/rtde_client.hpp"
#undef private

#include <atomic>
#include <cassert>
#include <chrono>
#include <cstring>
#include <thread>

using namespace vendor_robot_driver_core;

static std::atomic<int> g_create{0}, g_destroy{0}, g_receive{0}, g_stop{0}, g_servoj{0};
static std::atomic<int> g_receive_fail_after{-1};
static std::atomic<int> g_nonzero_after{0};
static std::atomic<RtdeResult> g_stop_result{RTDE_ERR_NONE};

extern "C" {
RtdeResult cd_rtde_get_sdk_version(uint8_t *v) { std::strcpy(reinterpret_cast<char*>(v), "mock"); return RTDE_ERR_NONE; }
RtdeResult cd_rtde_create(RtdeHandle *h, const char*, const char*, uint8_t*, int64_t, int) { *h=++g_create; return RTDE_ERR_NONE; }
RtdeResult cd_rtde_destroy(RtdeHandle) { ++g_destroy; return RTDE_ERR_NONE; }
RtdeResult cd_rtde_output_setup(RtdeHandle, char*, int, int *id) { *id=1; return RTDE_ERR_NONE; }
RtdeResult cd_rtde_output_control(RtdeHandle, uint32_t, int) { return RTDE_ERR_NONE; }
RtdeResult cd_rtde_output_data_receive(RtdeHandle, uint32_t, void *data, int size) {
  int n=++g_receive;
  if (g_receive_fail_after >= 0 && n > g_receive_fail_after) return RTDE_ERR_SOCKET_ERROR;
  std::memset(data, 0, size);
  if (n >= g_nonzero_after && g_nonzero_after > 0) {
    const double value = 10.0;
    std::memcpy(static_cast<uint8_t *>(data) + 4, &value, sizeof(value));
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(2));
  return RTDE_ERR_NONE;
}
RtdeResult cd_rtde_alldigitalinputbits_get(RtdeHandle, uint64_t *v) {*v=0; return RTDE_ERR_NONE;}
RtdeResult cd_rtde_alldigitaloutputbits_get(RtdeHandle, uint64_t *v) {*v=0; return RTDE_ERR_NONE;}
RtdeResult cd_rtde_reseterror(RtdeHandle) {return RTDE_ERR_NONE;}
RtdeResult cd_rtde_percentvelocity_set(RtdeHandle, double) {return RTDE_ERR_NONE;}
RtdeResult cd_rtde_standarddigitalout_set(RtdeHandle,uint8_t,uint8_t){return RTDE_ERR_NONE;}
RtdeResult cd_rtde_configurabledigitalout_set(RtdeHandle,uint8_t,uint8_t){return RTDE_ERR_NONE;}
RtdeResult cd_rtde_tooldigitalout_set(RtdeHandle,uint8_t,uint8_t){return RTDE_ERR_NONE;}
RtdeResult cd_rtde_servoj(RtdeHandle,uint8_t,double[6],uint8_t,double*,double){++g_servoj; return RTDE_ERR_NONE;}
RtdeResult cd_rtde_servo_stop(RtdeHandle,double){++g_stop; return g_stop_result.load();}
}

static RtdeConfig cfg() { RtdeConfig c; c.robot_ip="x"; c.io_poll_rate_hz=0; c.command_watchdog=std::chrono::milliseconds(5); return c; }

int main() {
  {
    g_create=0; g_destroy=0; g_receive=0; g_receive_fail_after=0; g_nonzero_after=0;
    RtdeClient c(cfg()); c.running_=true;
    assert(!c.connect_and_synchronize());
    assert(g_create==1 && g_destroy==1 && c.handle_ < 0);
  }
  {
    g_create=0; g_destroy=0; g_receive=0; g_receive_fail_after=-1; g_nonzero_after=8;
    RtdeClient c(cfg()); c.running_=true;
    assert(c.connect_and_synchronize());
    auto s=c.joint_state();
    assert(s.valid && s.position_rad[0] > 0.1 && s.sequence==1);
    c.close_session(false);
    assert(g_destroy==1);
  }
  {
    g_stop=0; g_servoj=0; g_stop_result=RTDE_ERR_SOCKET_ERROR;
    RtdeClient c(cfg());
    c.handle_=1; c.output_recipe_id_=1; c.servoj_succeeded_in_session_=true;
    c.servo_enabled_=true;
    c.command_.valid=true;
    c.command_.stamp=std::chrono::steady_clock::now()-std::chrono::seconds(1);
    c.servo_enable_stamp_=std::chrono::steady_clock::now()-std::chrono::seconds(1);
    assert(!c.send_pending_commands());
    assert(!c.servo_enabled_.load());
    c.submit_joint_position({0,0,0,0,0,0});
    assert(!c.send_pending_commands());
    assert(g_servoj==0 && g_stop>=1);
  }
  {
    g_stop=0;
    RtdeClient c(cfg());
    c.handle_=1; c.servoj_succeeded_in_session_=false; c.stop_sent_=true;
    c.request_safe_stop();
    assert(c.send_pending_commands());
    assert(g_stop==0);
    assert(c.safe_stop_handled_sequence_==c.safe_stop_request_sequence_.load());
  }
}
