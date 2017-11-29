/* -*- c++ -*- */
/* 
 * Copyright 2017 <+YOU OR YOUR COMPANY+>.
 * 
 * This is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3, or (at your option)
 * any later version.
 * 
 * This software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this software; see the file COPYING.  If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street,
 * Boston, MA 02110-1301, USA.
 */

#ifndef INCLUDED_WIFI_DSSS_PPDU_CHIP_MAPPER_BC_IMPL_H
#define INCLUDED_WIFI_DSSS_PPDU_CHIP_MAPPER_BC_IMPL_H

#include <wifi_dsss/ppdu_chip_mapper_bc.h>

namespace gr {
  namespace wifi_dsss {

    class ppdu_chip_mapper_bc_impl : public ppdu_chip_mapper_bc
    {
      enum DSSSRATE{
        LONG1M,
        LONG2M,
        LONG5_5M,
        LONG11M,
        SHORT2M,
        SHORT5_5M,
        SHORT11M
      };
     private:
      int d_count;
      int d_copy;
      int d_preCnt;
      int d_rate;
      int d_append;
      pmt::pmt_t d_rate_tag;
      bool d_preType;
      float d_phase_acc;
      float d_rateVal;
      const pmt::pmt_t d_lentag;
      const pmt::pmt_t d_name;
      int chipGen(gr_complex* out,const unsigned char* in,int noutput_items,int nin,int& nconsume);
      int cck_5_5M_chips(gr_complex* out,unsigned char byte, bool even);
      int cck_11M_chips(gr_complex* out,unsigned char byte, bool even);
      int dqpsk_2M_chips(gr_complex* out,unsigned char byte, bool even);
      int dbpsk_1M_chips(gr_complex* out,unsigned char byte, bool even);
      int (ppdu_chip_mapper_bc_impl::*d_chip_mapper)(gr_complex* out,unsigned char byte, bool even);
      int nout_check() const;
      int update_tag(int total_bytes) const;
     public:
      ppdu_chip_mapper_bc_impl(int rate, const std::string& lentag);
      ~ppdu_chip_mapper_bc_impl();

      // Where all the action really happens
      void forecast (int noutput_items, gr_vector_int &ninput_items_required);

      int general_work(int noutput_items,
           gr_vector_int &ninput_items,
           gr_vector_const_void_star &input_items,
           gr_vector_void_star &output_items);
    };

  } // namespace wifi_dsss
} // namespace gr

#endif /* INCLUDED_WIFI_DSSS_PPDU_CHIP_MAPPER_BC_IMPL_H */

