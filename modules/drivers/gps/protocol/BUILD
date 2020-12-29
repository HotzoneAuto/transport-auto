load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "canbus_transportgps_protocol",
    srcs = glob([
        "*.cc",
    ]),
    hdrs = glob([
        "*.h",
    ]),
    deps = [
        "//modules/drivers/canbus/common:canbus_common",
        "//modules/drivers/gps/proto:gps_cc_proto",
        "//modules/drivers/canbus/can_comm:message_manager_base",
    ],
)

cpplint()
