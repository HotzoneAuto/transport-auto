load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "transport_planning.so",
    linkopts = ["-shared"],
    linkstatic = False,
    deps = [":transport_planninglib"],
)

cc_library(
    name = "transport_planninglib",
    srcs = ["transport_planning.cc"],
    hdrs = ["transport_planning.h"],
    deps = [
        "//cyber",
        "//modules/drivers/gps:GPSproto",
        "//modules/drivers/gps/proto:gps_cc_proto",
        "//modules/planning/proto:planning_setting_conf_cc_proto",
        "//modules/planning/proto:trajs_cc_proto",
        "//modules/common/file:file",
    ],
)

cpplint()