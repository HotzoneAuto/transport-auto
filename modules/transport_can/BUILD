load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "transport_can.so",
    linkopts = ["-shared"],
    linkstatic = False,
    deps = [":transport_canlib"],
)

cc_library(
    name = "transport_canlib",
    srcs = ["transport_can.cc"],
    hdrs = ["transport_can.h"],
    deps = [
        "//cyber",
        "//modules/drivers/canbus/can_client/socket:socket_can_client_raw",
        "//modules/drivers/canbus/proto:can_card_parameter_cc_proto",
        "//modules/drivers/canbus/can_comm:can_receiver",
        "//modules/drivers/canbus/can_comm:can_sender",
        "//modules/transport_can/vehicle/transport:transport_message_manager",
        "//modules/transport_can/vehicle/transport:transport_controller",
	"//modules/transport_can/vehicle/transportgps:transportgps_message_manager",
    ],
)

cpplint()
