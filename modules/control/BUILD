load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "transport_control.so",
    linkopts = ["-shared"],
    linkstatic = False,
    deps = [":transport_controllib"],
)

cc_library(
    name = "transport_controllib",
    srcs = ["transport_control.cc"],
    hdrs = ["transport_control.h"],
    deps = [
        "//cyber",
        "//modules/drivers/gps:gps_protocol",
        "//modules/drivers/gps/proto:gps_cc_proto",
        "//modules/control/proto:control_command_cc_proto",
        "//modules/control/proto:control_setting_conf_cc_proto",
        "//modules/planning/proto:trajectory_cc_proto",
    ],
)

cpplint()