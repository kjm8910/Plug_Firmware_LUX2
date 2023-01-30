
# source files
set(srcs
   init.c
   gps.c
   imu.c
   util.c
   _kf.c
   _matrix.c
   imu_correct.c
   imu_dist_loop.c
   imu_dist.c
   )

# include directories
set(include_dirs 
    include
    )

set(priv_include_dirs
   )


# dependency
set(priv_requires
   )


# registers the component to the build system
idf_component_register(SRCS ${srcs}
                    INCLUDE_DIRS ${include_dirs}
                    PRIV_INCLUDE_DIRS ${priv_include_dirs}
                    PRIV_REQUIRES ${priv_requires}
                    )

# end of file
