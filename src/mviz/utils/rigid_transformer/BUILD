load("@rules_cc//cc:defs.bzl", "cc_library")

cc_library(
    name = "rigid_transformer",
    srcs = ['rigid_transformer.cpp'],
    hdrs = ["rigid_transformer.h"],
    deps = [
        "@com_gitlab_eigen3//:eigen",
        "@libglog//:libglog",
    ],
    visibility = ["//visibility:public"],
)