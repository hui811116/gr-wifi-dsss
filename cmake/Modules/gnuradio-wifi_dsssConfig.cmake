find_package(PkgConfig)

PKG_CHECK_MODULES(PC_GR_WIFI_DSSS gnuradio-wifi_dsss)

FIND_PATH(
    GR_WIFI_DSSS_INCLUDE_DIRS
    NAMES gnuradio/wifi_dsss/api.h
    HINTS $ENV{WIFI_DSSS_DIR}/include
        ${PC_WIFI_DSSS_INCLUDEDIR}
    PATHS ${CMAKE_INSTALL_PREFIX}/include
          /usr/local/include
          /usr/include
)

FIND_LIBRARY(
    GR_WIFI_DSSS_LIBRARIES
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

include("${CMAKE_CURRENT_LIST_DIR}/gnuradio-wifi_dsssTarget.cmake")

INCLUDE(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(GR_WIFI_DSSS DEFAULT_MSG GR_WIFI_DSSS_LIBRARIES GR_WIFI_DSSS_INCLUDE_DIRS)
MARK_AS_ADVANCED(GR_WIFI_DSSS_LIBRARIES GR_WIFI_DSSS_INCLUDE_DIRS)
