#include "uart_client.h"

#include <cstdio>
#include <iostream>
#include <string>
#include <cstring>
using std::cout;
using std::endl;
using namespace std;

bool UartClient::Init() {
  memset(uart_buf,0,sizeof(uart_buf));
  uart_dev.SetOpt(9600, 8, 'N' ,1);
  is_running=0;
  return true;
}

bool UartClient::GetData(char *data) {  // Timer callback
  {
    std::lock_guard<mutex> guard(uart_mutex);
    strcpy(data,uart_buf);
  }
  return true;
}
void UartClient::Close() {  // shutdown
  is_running=0;
}
void UartClient::Start(){
  is_running=1;
  AINFO<<"Uart Started";
  auto async_result_ = Async(&UartClient::RecvThreadFunc, this);
}
void UartClient::RecvThreadFunc(){
  while(is_running){
    {
      char buf[BUFF_LENGTH];
      memset(buf,0,sizeof(buf));
      if(uart_dev.Read(buf,sizeof(uart_buf)) >= 0)
      {
        std::lock_guard<mutex> guard(uart_mutex);
        strcpy(uart_buf,buf);
      }else
      {
        AERROR << "Uart Read Error";
        return;
      }
      Yield();
    }
  }
}
