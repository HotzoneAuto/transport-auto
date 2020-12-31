#pragma once

#include <string>
#include <fstream>

#include "cyber/cyber.h"

namespace apollo {
namespace common {
namespace file {

class File {
 public:
   bool open_file(std::string name);
   bool write_file(std::string msg);
   bool read_file();
   bool close_file();
 private:
   std::fstream f; 
};

}  // namespace file
}  // namespace common
}  // namespace apollo

