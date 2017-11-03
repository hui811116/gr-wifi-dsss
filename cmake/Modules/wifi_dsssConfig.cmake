INCLUDE(FindPkgConfig)
PKG_CHECK_MODULES(PC_WIFI_DSSS wifi_dsss)

FIND_PATH(
    WIFI_DSSS_INCLUDE_DIRS
    NAMES wifi_dsss/api.h
    HINTS $ENV{WIFI_DSSS_DIR}/include
        ${PC_WIFI_DSSS_INCLUDEDIR}
    PATHS ${CMAKE_INSTALL_PREFIX}/include
          /usr/local/include
          /usr/include
)

FIND_LIBRARY(
    WIFI_DSSS_LIBRARIES
    NAMES gnuradio-wifi_dsss
    HINTS $ENV{WIFI_DSSS_DIR}/lib
        ${PC_WIFI_DSSS_LIBDIR}
    PATHS ${CMAKE_INSTALL_PREFIX}/lib
          ${CMAKE_INSTALL_PREFIX}/lib64
          /usr/local/lib
          /usr/local/lib64
          /usr/lib
          /usr/lib64
)

INCLUDE(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(WIFI_DSSS DEFAULT_MSG WIFI_DSSS_LIBRARIES WIFI_DSSS_INCLUDE_DIRS)
MARK_AS_ADVANCED(WIFI_DSSS_LIBRARIES WIFI_DSSS_INCLUDE_DIRS)

