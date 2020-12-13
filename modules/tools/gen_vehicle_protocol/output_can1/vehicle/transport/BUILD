load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "transport_vehicle_factory",
    srcs = [
        "transport_vehicle_factory.cc",
    ],
    hdrs = [
        "transport_vehicle_factory.h",
    ],
    deps = [
        ":transport_controller",
        ":transport_message_manager",
        "//modules/canbus/vehicle:abstract_vehicle_factory",
    ],
)

cc_library(
    name = "transport_message_manager",
    srcs = [
        "transport_message_manager.cc",
    ],
    hdrs = [
        "transport_message_manager.h",
    ],
    deps = [
        "//modules/drivers/canbus/common:canbus_common",
        "//modules/canbus/proto:canbus_proto",
        "//modules/drivers/canbus/can_comm:message_manager_base",
        "//modules/canbus/vehicle/transport/protocol:canbus_transport_protocol",
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
        "//modules/canbus/proto:canbus_proto",
        "//modules/drivers/canbus/can_comm:message_manager_base",
        "//modules/canbus/vehicle:vehicle_controller_base",
        "//modules/canbus/vehicle/transport/protocol:canbus_transport_protocol",
    ],
)

cpplint()
