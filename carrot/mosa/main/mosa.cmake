
# MOS application cmake
#

set(PREBUILT_DIR prebuilt)

# mosa source files
set(mosa_srcs
   main.c
   global.c
   cmd.c
   IMU.c
   GPS.c
   time.c
   )


# mos include directories
set(mosa_include_dirs
   include
   ${PREBUILT_DIR}/include
   )


# component requires
set(mosa_requires
   )

# app compile options
set(mosa_compile_options 
   )


# app compile definitions
set(mosa_compile_definitions 
   )


# end of file
