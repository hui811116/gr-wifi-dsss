/* -*- c++ -*- */

#define WIFI_DSSS_API

%include "gnuradio.i"			// the common stuff

//load generated python docstrings
%include "wifi_dsss_swig_doc.i"

%{
#include "wifi_dsss/ppdu_prefixer.h"
#include "wifi_dsss/ppdu_chip_mapper_bc.h"
#include "wifi_dsss/chip_sync_c.h"
%}


%include "wifi_dsss/ppdu_prefixer.h"
GR_SWIG_BLOCK_MAGIC2(wifi_dsss, ppdu_prefixer);%include "wifi_dsss/ppdu_chip_mapper_bc.h"
GR_SWIG_BLOCK_MAGIC2(wifi_dsss, ppdu_chip_mapper_bc);
%include "wifi_dsss/chip_sync_c.h"
GR_SWIG_BLOCK_MAGIC2(wifi_dsss, chip_sync_c);
