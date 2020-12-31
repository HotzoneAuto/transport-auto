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
    srcs = ["transport_canbus_component.cc"],
    hdrs = ["transport_canbus_component.h"],
    deps = [
        ":transport_controller",
        ":transport_message_manager",
        "//cyber",
        "//modules/drivers/canbus/can_client/socket:socket_can_client_raw",
        "//modules/drivers/canbus/can_comm:can_receiver",
        "//modules/drivers/canbus/can_comm:can_sender",
        "//modules/drivers/canbus/proto:can_card_parameter_cc_proto",
    ],
)

cc_library(
    name = "transport_message_manager",
    srcs = ["transport_message_manager.cc"],
    hdrs = ["transport_message_manager.h"],
    deps = [
        "//cyber",
        "//modules/drivers/canbus/can_comm:message_manager_base",
        "//modules/canbus/proto:chassis_detail_cc_proto",
        "//modules/canbus/protocol:canbus_transport_protocol",
    ],
)

cc_library(
    name = "transport_controller",
    srcs = [
        "transport_controller.cc",
    ],
    hdrs = [
        "transport_controller.h",
    ],
    deps = [
        ":transport_message_manager",
        "//modules/drivers/canbus/can_comm:can_sender",
        "//modules/drivers/canbus/common:canbus_common",
        "//modules/control/proto:control_command_cc_proto",
        "//modules/canbus/proto:transport_can_conf_cc_proto",
        "//modules/canbus/protocol:canbus_transport_protocol",
    ],
)

cpplint()
