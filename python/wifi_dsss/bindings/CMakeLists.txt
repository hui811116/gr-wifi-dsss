# Copyright 2020 Free Software Foundation, Inc.
#
# This file is part of GNU Radio
#
# SPDX-License-Identifier: GPL-3.0-or-later
#

########################################################################
# Check if there is C++ code at all
########################################################################
if(NOT wifi_dsss_sources)
    message(STATUS "No C++ sources... skipping python bindings")
    return()
endif(NOT wifi_dsss_sources)

########################################################################
# Check for pygccxml
########################################################################
gr_python_check_module_raw("pygccxml" "import pygccxml" PYGCCXML_FOUND)

include(GrPybind)

########################################################################
# Python Bindings
########################################################################

list(APPEND wifi_dsss_python_files
    ppdu_prefixer_python.cc
    chip_sync_c_python.cc
    ppdu_chip_mapper_bc_python.cc python_bindings.cc)

gr_pybind_make_oot(wifi_dsss ../../.. gr::wifi_dsss "${wifi_dsss_python_files}")

# copy bindings extension for use in QA test module
add_custom_command(
    TARGET wifi_dsss_python
    POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:wifi_dsss_python>
            ${PROJECT_BINARY_DIR}/test_modules/gnuradio/wifi_dsss/)

install(
    TARGETS wifi_dsss_python
    DESTINATION ${GR_PYTHON_DIR}/gnuradio/wifi_dsss
    COMPONENT pythonapi)
