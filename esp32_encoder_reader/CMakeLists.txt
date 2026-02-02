# ESP32 多摩川编码器读取项目

cmake_minimum_required(VERSION 3.16)

# 设置目标芯片 (可选: esp32, esp32s2, esp32s3, esp32c3)
set(IDF_TARGET "esp32s3" CACHE STRING "Target chip")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

project(encoder_reader)
