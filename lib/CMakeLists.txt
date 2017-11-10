# Copyright 2011,2012,2016 Free Software Foundation, Inc.
#
# This file is part of GNU Radio
#
# GNU Radio is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 3, or (at your option)
# any later version.
#
# GNU Radio is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with GNU Radio; see the file COPYING.  If not, write to
# the Free Software Foundation, Inc., 51 Franklin Street,
# Boston, MA 02110-1301, USA.

########################################################################
# Setup library
########################################################################
include(GrPlatform) #define LIB_SUFFIX

include_directories(${Boost_INCLUDE_DIR} ${VOLK_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS} ${VOLK_LIBRARY_DIRS})

list(APPEND wifi_dsss_sources
    ppdu_prefixer.cc
    ppdu_chip_mapper_bc_impl.cc
    chip_sync_c_impl.cc
)

set(wifi_dsss_sources "${wifi_dsss_sources}" PARENT_SCOPE)
if(NOT wifi_dsss_sources)
	MESSAGE(STATUS "No C++ sources... skipping lib/")
	return()
endif(NOT wifi_dsss_sources)

add_library(gnuradio-wifi_dsss SHARED ${wifi_dsss_sources})
target_link_libraries(gnuradio-wifi_dsss ${Boost_LIBRARIES} ${GNURADIO_ALL_LIBRARIES} ${VOLK_LIBRARIES})
set_target_properties(gnuradio-wifi_dsss PROPERTIES DEFINE_SYMBOL "gnuradio_wifi_dsss_EXPORTS")

if(APPLE)
    set_target_properties(gnuradio-wifi_dsss PROPERTIES
        INSTALL_NAME_DIR "${CMAKE_INSTALL_PREFIX}/lib"
    )
endif(APPLE)

########################################################################
# Install built library files
########################################################################
include(GrMiscUtils)
GR_LIBRARY_FOO(gnuradio-wifi_dsss RUNTIME_COMPONENT "wifi_dsss_runtime" DEVEL_COMPONENT "wifi_dsss_devel")

########################################################################
# Build and register unit test
########################################################################
include(GrTest)

include_directories(${CPPUNIT_INCLUDE_DIRS})

list(APPEND test_wifi_dsss_sources
    ${CMAKE_CURRENT_SOURCE_DIR}/test_wifi_dsss.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/qa_wifi_dsss.cc
)

add_executable(test-wifi_dsss ${test_wifi_dsss_sources})

target_link_libraries(
  test-wifi_dsss
  ${GNURADIO_RUNTIME_LIBRARIES}
  ${Boost_LIBRARIES}
  ${CPPUNIT_LIBRARIES}
  gnuradio-wifi_dsss
)

GR_ADD_TEST(test_wifi_dsss test-wifi_dsss)

########################################################################
# Print summary
########################################################################
message(STATUS "Using install prefix: ${CMAKE_INSTALL_PREFIX}")
message(STATUS "Building for version: ${VERSION} / ${LIBVER}")

