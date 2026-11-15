# ESP-IDF project root for the BC2 USB BLE advertiser firmware.
cmake_minimum_required(VERSION 3.16)

# Own components live in ./components and are picked up automatically.
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(bc2-usb-advertiser)
