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
    srcs = ["GPSproto.cpp"],
    hdrs = ["GPSproto.h"],
    deps = [
        "//cyber",
    ],
)

cpplint()
