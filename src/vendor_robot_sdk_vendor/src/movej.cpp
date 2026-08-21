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
  
  
}