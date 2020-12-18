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
	"//modules/transport_can/proto:chassis_detail_cc_proto",
	"//modules/transport_can/proto:control_command_cc_proto",
	"//modules/transport_control/gps:GPSproto",
    ],
)

cpplint()
