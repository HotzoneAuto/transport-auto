load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])


cc_library(
    name = "uartlib",
    srcs = ["uart_client.cc"],
    hdrs = ["uart_client.h"],
    deps = [
        "//cyber",
        "//modules/common:uart",
    ],
)
cc_library(
    name = "GPSproto",
    srcs = ["GPSproto.cc"],
    hdrs = ["GPSproto.h"],
    deps = [
        "//cyber",
    ],
)

cc_binary(
    name = "transport_gps.so",
    linkopts = ["-shared"],
    linkstatic = False,
    deps = [":transport_gpslib"],
)

cc_library(
    name = "transport_gpslib",
    srcs = ["gps_component.cc"],
    hdrs = ["gps_component.h"],
    deps = [
        ":message_manager",
        "//cyber",
        "//modules/drivers/canbus/can_client/socket:socket_can_client_raw",
        "//modules/drivers/canbus/can_comm:can_receiver",
        "//modules/drivers/canbus/can_comm:can_sender",
    ],
)

cc_library(
    name = "message_manager",
    srcs = [
        "message_manager.cc",
    ],
    hdrs = [
        "message_manager.h",
    ],
    deps = [
        "//modules/drivers/canbus/common:canbus_common",
        "//modules/drivers/gps/proto:gps_cc_proto",
        "//modules/drivers/canbus/can_comm:message_manager_base",
        "//modules/drivers/gps/protocol:canbus_transportgps_protocol",
    ],
)

cpplint()
