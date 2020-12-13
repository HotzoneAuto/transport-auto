load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "transportgps_message_manager",
    srcs = [
        "transportgps_message_manager.cc",
    ],
    hdrs = [
        "transportgps_message_manager.h",
    ],
    deps = [
        "//modules/drivers/canbus/common:canbus_common",
        "//modules/transport_can/proto:chassis_detail_cc_proto",
        "//modules/drivers/canbus/can_comm:message_manager_base",
        "//modules/transport_can/vehicle/transportgps/protocol:canbus_transportgps_protocol",
    ],
)

cpplint()
