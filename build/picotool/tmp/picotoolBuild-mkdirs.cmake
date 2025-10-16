# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/home/samcv/pico/pico_quiros/build/_deps/picotool-src"
  "/home/samcv/pico/pico_quiros/build/_deps/picotool-build"
  "/home/samcv/pico/pico_quiros/build/_deps"
  "/home/samcv/pico/pico_quiros/build/picotool/tmp"
  "/home/samcv/pico/pico_quiros/build/picotool/src/picotoolBuild-stamp"
  "/home/samcv/pico/pico_quiros/build/picotool/src"
  "/home/samcv/pico/pico_quiros/build/picotool/src/picotoolBuild-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/home/samcv/pico/pico_quiros/build/picotool/src/picotoolBuild-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/home/samcv/pico/pico_quiros/build/picotool/src/picotoolBuild-stamp${cfgdir}") # cfgdir has leading slash
endif()
