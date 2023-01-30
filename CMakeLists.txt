# For more information about build system see
# https://docs.espressif.com/projects/esp-idf/en/latest/api-guides/build-system.html
# The following five lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.5)


# COMPONENT_DIRS: default <PROJECT_DIR>/components
# set(COMPONENT_DIRS ./components)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

# application project cmake
include(${CMAKE_SOURCE_DIR}/app_proj.cmake)

# set project name
project(mosa)

# application image for main server
app_proj_show(CONFIG_APP_PRODUCT_MODE)

# end of file
