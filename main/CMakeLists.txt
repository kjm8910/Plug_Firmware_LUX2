

#list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR})
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_LIST_DIR})

# mosa.cmake
include(mosa)


# application component
idf_component_register(SRCS ${mosa_srcs}
                    INCLUDE_DIRS ${mosa_include_dirs}
                    REQUIRES ${mosa_requires})


# compile options
target_compile_options(${COMPONENT_LIB} PRIVATE
        ${mosa_compile_options})


# compile definitions
target_compile_definitions(${COMPONENT_LIB} PRIVATE
        ${mosa_compile_definitions})

# prebuilt library
add_prebuilt_library(APP ${PREBUILT_DIR}/components/libAPP.a
	PRIV_REQUIRES bt esp_adc_cal driver nvs_flash)

add_prebuilt_library(AT ${PREBUILT_DIR}/components/libAT.a)
add_prebuilt_library(checksum ${PREBUILT_DIR}/components/libchecksum.a)
add_prebuilt_library(dcom ${PREBUILT_DIR}/components/libdcom.a)
add_prebuilt_library(utimer ${PREBUILT_DIR}/components/libutimer.a)
add_prebuilt_library(jsmn ${PREBUILT_DIR}/components/libjsmn.a)
add_prebuilt_library(nmea ${PREBUILT_DIR}/components/libnmea.a)
add_prebuilt_library(pdulib ${PREBUILT_DIR}/components/libpdulib.a)
add_prebuilt_library(polltimer ${PREBUILT_DIR}/components/libpolltimer.a)
add_prebuilt_library(umd5 ${PREBUILT_DIR}/components/libumd5.a)
add_prebuilt_library(ustring ${PREBUILT_DIR}/components/libustring.a)
add_prebuilt_library(gray ${PREBUILT_DIR}/components/libgray.a)

# `main` calls a function from the library, so link it to `main`
target_link_libraries(${COMPONENT_LIB} PRIVATE
        APP
        AT
        checksum
        dcom
        jsmn
        nmea
        pdulib
        polltimer
        umd5
        ustring
        utimer
        gray)

# end of file
