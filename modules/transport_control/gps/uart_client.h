#define BUFF_LENGTH 300
#include <iostream>
#include <vector>
#include <mutex>
#include "cyber/component/component.h"
#include "cyber/component/timer_component.h"
#include "cyber/cyber.h"
#include "modules/common/uart.h"

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
 Uart uart_dev=Uart("ttyTHS2");
 bool is_running;
 std::mutex uart_mutex;
 std::shared_ptr<Uart> dev_p;

};
