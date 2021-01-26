# SPDX-License-Identifier: BSD-3-Clause

cmake_minimum_required(VERSION 3.13.1)
include($ENV{ZEPHYR_BASE}/cmake/app/boilerplate.cmake NO_POLICY_SCOPE)
project(greybus-net)

zephyr_include_directories(../../../../../include)

include(../../../../cmake/greybus.cmake)

if(CONFIG_GREYBUS_CLICK_MANIFEST_BUILTIN)
    mnfs_to_mnfb(
    ../../../../scripts/manifesto/manifests/${CONFIG_GREYBUS_MIKROBUS_CLICK1_NAME}.mnfs
    ${ZEPHYR_BINARY_DIR}/include/generated/greybus_click_fragment1.mnfb
    )

    mnfs_to_mnfb(
    ../../../../scripts/manifesto/manifests/${CONFIG_GREYBUS_MIKROBUS_CLICK2_NAME}.mnfs
    ${ZEPHYR_BINARY_DIR}/include/generated/greybus_click_fragment2.mnfb
    )

    generate_inc_file_for_target(
    app
    ${ZEPHYR_BINARY_DIR}/include/generated/greybus_click_fragment1.mnfb
    ${ZEPHYR_BINARY_DIR}/include/generated/greybus_click_fragment1_mnfb.inc
    )

    generate_inc_file_for_target(
    app
    ${ZEPHYR_BINARY_DIR}/include/generated/greybus_click_fragment2.mnfb
    ${ZEPHYR_BINARY_DIR}/include/generated/greybus_click_fragment2_mnfb.inc
    )
endif()

target_sources(app PRIVATE src/main.c)
