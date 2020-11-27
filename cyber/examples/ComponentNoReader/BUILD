load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "libComponentNoReader.so",
    linkopts = ["-shared"],
    linkstatic = False,
    deps = [":ComponentNoReader_lib"],
)

cc_library(
    name = "ComponentNoReader_lib",
    srcs = ["ComponentNoReader.cc"],
    hdrs = ["ComponentNoReader.h"],
    deps = [
        "//cyber",
        "//cyber/examples/proto:examples_cc_proto",
        "//modules/drivers/canbus/proto:can_card_parameter_cc_proto",
        "//modules/drivers/canbus/can_client/socket:socket_can_client_raw",
        "//modules/drivers/canbus/can_comm:can_receiver",
       
    ],
)

cpplint()
