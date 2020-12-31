#include "modules/common/file/file.h"

namespace apollo {
namespace common {
namespace file {

   bool File::open_file(std::string name) {
      f.open(name, std::ios::out | std::ios::trunc);
   }

   bool File::write_file(std::string msg) {
      if (!f.is_open()) {
        AERROR << "file is not open";
        return false;
      }
      f << msg << std::endl;
      return true;
   }

   bool File::read_file() {}

   bool File::close_file() {
      if (!f.is_open()) {
        AERROR << "file is not open";
        return false;
      }
      f.close();
   }

}  // namespace time
}  // namespace common
}  // namespace apollo

