load("//tools:cpplint.bzl", "cpplint")
package(default_visibility = ["//visibility:public"])
cc_library(
    name = "transport_message_manager",
    srcs = ["transport_message_manager.cc"],
    hdrs = ["transport_message_manager.h"],
    deps = [
        "//cyber",
	"//modules/drivers/canbus/can_comm:message_manager_base",
	"//modules/transport_can/vehicle/transport/protocol:canbus_transport_protocol",
	"//modules/transport_can/proto:chassis_detail_cc_proto",
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
        "//modules/transport_can/proto:control_command_cc_proto",
        "//modules/transport_can/vehicle/transport/protocol:canbus_transport_protocol",
    ],
)


cpplint()
