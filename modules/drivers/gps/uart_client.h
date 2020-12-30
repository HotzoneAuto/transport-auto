#pragma once

#include <iostream>
#include <mutex>
#include <vector>

#include "cyber/cyber.h"

#include "modules/common/uart.h"

namespace apollo {
namespace drivers {
namespace gps {

#define BUFF_LENGTH 300

using apollo::cyber::Async;
using apollo::cyber::Yield;

class UartClient {
 public:
  bool Init();
  bool GetData(char *data);
  void Close();
  void Start();
  void RecvThreadFunc();

 private:
  char uart_buf[BUFF_LENGTH];
  Uart uart_dev = Uart("ttyTHS2");
  bool is_running;
  std::mutex uart_mutex;
  std::shared_ptr<Uart> dev_p;
};

}  // namespace gps
}  // namespace drivers
}  // namespace apollo