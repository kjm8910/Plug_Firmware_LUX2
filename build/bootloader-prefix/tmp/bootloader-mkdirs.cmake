# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/Users/jm.kim/Desktop/carrot/esp-idf/components/bootloader/subproject"
  "/Users/jm.kim/Desktop/carrot/mosa/build/bootloader"
  "/Users/jm.kim/Desktop/carrot/mosa/build/bootloader-prefix"
  "/Users/jm.kim/Desktop/carrot/mosa/build/bootloader-prefix/tmp"
  "/Users/jm.kim/Desktop/carrot/mosa/build/bootloader-prefix/src/bootloader-stamp"
  "/Users/jm.kim/Desktop/carrot/mosa/build/bootloader-prefix/src"
  "/Users/jm.kim/Desktop/carrot/mosa/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/Users/jm.kim/Desktop/carrot/mosa/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/Users/jm.kim/Desktop/carrot/mosa/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
