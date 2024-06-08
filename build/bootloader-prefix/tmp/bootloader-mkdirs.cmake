# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/home/levireg/esp/esp-idf/components/bootloader/subproject"
  "/home/levireg/esp/freertos_lvgl/build/bootloader"
  "/home/levireg/esp/freertos_lvgl/build/bootloader-prefix"
  "/home/levireg/esp/freertos_lvgl/build/bootloader-prefix/tmp"
  "/home/levireg/esp/freertos_lvgl/build/bootloader-prefix/src/bootloader-stamp"
  "/home/levireg/esp/freertos_lvgl/build/bootloader-prefix/src"
  "/home/levireg/esp/freertos_lvgl/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/home/levireg/esp/freertos_lvgl/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/home/levireg/esp/freertos_lvgl/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
