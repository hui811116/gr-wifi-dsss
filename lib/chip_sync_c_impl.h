/* -*- c++ -*- */
/* 
 * Copyright 2017 Teng-Hui Huang.
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

#ifndef INCLUDED_WIFI_DSSS_CHIP_SYNC_C_IMPL_H
#define INCLUDED_WIFI_DSSS_CHIP_SYNC_C_IMPL_H

#include <wifi_dsss/chip_sync_c.h>

namespace gr {
  namespace wifi_dsss {
    static const gr_complex d_cck5_5_chips[4][8] = {
{gr_complex(0.0,1.0),gr_complex(1.0,0.0),gr_complex(0.0,1.0),gr_complex(-1.0,0.0),gr_complex(0.0,1.0),gr_complex(1.0,0.0),gr_complex(-0.0,-1.0),gr_complex(1.0,0.0)},
{gr_complex(-0.0,-1.0),gr_complex(1.0,0.0),gr_complex(-0.0,-1.0),gr_complex(-1.0,0.0),gr_complex(-0.0,-1.0),gr_complex(1.0,0.0),gr_complex(0.0,1.0),gr_complex(1.0,0.0)},
{gr_complex(-0.0,-1.0),gr_complex(-1.0,0.0),gr_complex(-0.0,-1.0),gr_complex(1.0,0.0),gr_complex(0.0,1.0),gr_complex(1.0,0.0),gr_complex(-0.0,-1.0),gr_complex(1.0,0.0)},
{gr_complex(0.0,1.0),gr_complex(-1.0,0.0),gr_complex(0.0,1.0),gr_complex(1.0,0.0),gr_complex(-0.0,-1.0),gr_complex(1.0,0.0),gr_complex(0.0,1.0),gr_complex(1.0,0.0)}
};
    static const gr_complex d_cck11_chips[64][8] = {
{gr_complex(1.0,0.0),gr_complex(1.0,0.0),gr_complex(1.0,0.0),gr_complex(-1.0,0.0),gr_complex(1.0,0.0),gr_complex(1.0,0.0),gr_complex(-1.0,0.0),gr_complex(1.0,0.0)},
{gr_complex(-1.0,0.0),gr_complex(1.0,0.0),gr_complex(-1.0,0.0),gr_complex(-1.0,0.0),gr_complex(-1.0,0.0),gr_complex(1.0,0.0),gr_complex(1.0,-0.0),gr_complex(1.0,0.0)},
{gr_complex(0.0,1.0),gr_complex(1.0,0.0),gr_complex(0.0,1.0),gr_complex(-1.0,0.0),gr_complex(0.0,1.0),gr_complex(1.0,0.0),gr_complex(-0.0,-1.0),gr_complex(1.0,0.0)},
{gr_complex(-0.0,-1.0),gr_complex(1.0,0.0),gr_complex(-0.0,-1.0),gr_complex(-1.0,0.0),gr_complex(-0.0,-1.0),gr_complex(1.0,0.0),gr_complex(0.0,1.0),gr_complex(1.0,0.0)},
{gr_complex(-1.0,0.0),gr_complex(-1.0,0.0),gr_complex(1.0,0.0),gr_complex(-1.0,0.0),gr_complex(-1.0,0.0),gr_complex(-1.0,0.0),gr_complex(-1.0,0.0),gr_complex(1.0,0.0)},
{gr_complex(1.0,-0.0),gr_complex(-1.0,0.0),gr_complex(-1.0,0.0),gr_complex(-1.0,0.0),gr_complex(1.0,-0.0),gr_complex(-1.0,0.0),gr_complex(1.0,-0.0),gr_complex(1.0,0.0)},
{gr_complex(-0.0,-1.0),gr_complex(-1.0,0.0),gr_complex(0.0,1.0),gr_complex(-1.0,0.0),gr_complex(-0.0,-1.0),gr_complex(-1.0,0.0),gr_complex(-0.0,-1.0),gr_complex(1.0,0.0)},
{gr_complex(0.0,1.0),gr_complex(-1.0,0.0),gr_complex(-0.0,-1.0),gr_complex(-1.0,0.0),gr_complex(0.0,1.0),gr_complex(-1.0,0.0),gr_complex(0.0,1.0),gr_complex(1.0,0.0)},
{gr_complex(0.0,1.0),gr_complex(0.0,1.0),gr_complex(1.0,0.0),gr_complex(-1.0,0.0),gr_complex(0.0,1.0),gr_complex(0.0,1.0),gr_complex(-1.0,0.0),gr_complex(1.0,0.0)},
{gr_complex(-0.0,-1.0),gr_complex(0.0,1.0),gr_complex(-1.0,0.0),gr_complex(-1.0,0.0),gr_complex(-0.0,-1.0),gr_complex(0.0,1.0),gr_complex(1.0,-0.0),gr_complex(1.0,0.0)},
{gr_complex(-1.0,0.0),gr_complex(0.0,1.0),gr_complex(0.0,1.0),gr_complex(-1.0,0.0),gr_complex(-1.0,0.0),gr_complex(0.0,1.0),gr_complex(-0.0,-1.0),gr_complex(1.0,0.0)},
{gr_complex(1.0,-0.0),gr_complex(0.0,1.0),gr_complex(-0.0,-1.0),gr_complex(-1.0,0.0),gr_complex(1.0,-0.0),gr_complex(0.0,1.0),gr_complex(0.0,1.0),gr_complex(1.0,0.0)},
{gr_complex(-0.0,-1.0),gr_complex(-0.0,-1.0),gr_complex(1.0,0.0),gr_complex(-1.0,0.0),gr_complex(-0.0,-1.0),gr_complex(-0.0,-1.0),gr_complex(-1.0,0.0),gr_complex(1.0,0.0)},
{gr_complex(0.0,1.0),gr_complex(-0.0,-1.0),gr_complex(-1.0,0.0),gr_complex(-1.0,0.0),gr_complex(0.0,1.0),gr_complex(-0.0,-1.0),gr_complex(1.0,-0.0),gr_complex(1.0,0.0)},
{gr_complex(1.0,-0.0),gr_complex(-0.0,-1.0),gr_complex(0.0,1.0),gr_complex(-1.0,0.0),gr_complex(1.0,-0.0),gr_complex(-0.0,-1.0),gr_complex(-0.0,-1.0),gr_complex(1.0,0.0)},
{gr_complex(-1.0,0.0),gr_complex(-0.0,-1.0),gr_complex(-0.0,-1.0),gr_complex(-1.0,0.0),gr_complex(-1.0,0.0),gr_complex(-0.0,-1.0),gr_complex(0.0,1.0),gr_complex(1.0,0.0)},
{gr_complex(-1.0,0.0),gr_complex(-1.0,0.0),gr_complex(-1.0,0.0),gr_complex(1.0,-0.0),gr_complex(1.0,0.0),gr_complex(1.0,0.0),gr_complex(-1.0,0.0),gr_complex(1.0,0.0)},
{gr_complex(1.0,-0.0),gr_complex(-1.0,0.0),gr_complex(1.0,-0.0),gr_complex(1.0,-0.0),gr_complex(-1.0,0.0),gr_complex(1.0,0.0),gr_complex(1.0,-0.0),gr_complex(1.0,0.0)},
{gr_complex(-0.0,-1.0),gr_complex(-1.0,0.0),gr_complex(-0.0,-1.0),gr_complex(1.0,-0.0),gr_complex(0.0,1.0),gr_complex(1.0,0.0),gr_complex(-0.0,-1.0),gr_complex(1.0,0.0)},
{gr_complex(0.0,1.0),gr_complex(-1.0,0.0),gr_complex(0.0,1.0),gr_complex(1.0,-0.0),gr_complex(-0.0,-1.0),gr_complex(1.0,0.0),gr_complex(0.0,1.0),gr_complex(1.0,0.0)},
{gr_complex(1.0,-0.0),gr_complex(1.0,-0.0),gr_complex(-1.0,0.0),gr_complex(1.0,-0.0),gr_complex(-1.0,0.0),gr_complex(-1.0,0.0),gr_complex(-1.0,0.0),gr_complex(1.0,0.0)},
{gr_complex(-1.0,0.0),gr_complex(1.0,-0.0),gr_complex(1.0,-0.0),gr_complex(1.0,-0.0),gr_complex(1.0,-0.0),gr_complex(-1.0,0.0),gr_complex(1.0,-0.0),gr_complex(1.0,0.0)},
{gr_complex(0.0,1.0),gr_complex(1.0,-0.0),gr_complex(-0.0,-1.0),gr_complex(1.0,-0.0),gr_complex(-0.0,-1.0),gr_complex(-1.0,0.0),gr_complex(-0.0,-1.0),gr_complex(1.0,0.0)},
{gr_complex(-0.0,-1.0),gr_complex(1.0,-0.0),gr_complex(0.0,1.0),gr_complex(1.0,-0.0),gr_complex(0.0,1.0),gr_complex(-1.0,0.0),gr_complex(0.0,1.0),gr_complex(1.0,0.0)},
{gr_complex(-0.0,-1.0),gr_complex(-0.0,-1.0),gr_complex(-1.0,0.0),gr_complex(1.0,-0.0),gr_complex(0.0,1.0),gr_complex(0.0,1.0),gr_complex(-1.0,0.0),gr_complex(1.0,0.0)},
{gr_complex(0.0,1.0),gr_complex(-0.0,-1.0),gr_complex(1.0,-0.0),gr_complex(1.0,-0.0),gr_complex(-0.0,-1.0),gr_complex(0.0,1.0),gr_complex(1.0,-0.0),gr_complex(1.0,0.0)},
{gr_complex(1.0,-0.0),gr_complex(-0.0,-1.0),gr_complex(-0.0,-1.0),gr_complex(1.0,-0.0),gr_complex(-1.0,0.0),gr_complex(0.0,1.0),gr_complex(-0.0,-1.0),gr_complex(1.0,0.0)},
{gr_complex(-1.0,0.0),gr_complex(-0.0,-1.0),gr_complex(0.0,1.0),gr_complex(1.0,-0.0),gr_complex(1.0,-0.0),gr_complex(0.0,1.0),gr_complex(0.0,1.0),gr_complex(1.0,0.0)},
{gr_complex(0.0,1.0),gr_complex(0.0,1.0),gr_complex(-1.0,0.0),gr_complex(1.0,-0.0),gr_complex(-0.0,-1.0),gr_complex(-0.0,-1.0),gr_complex(-1.0,0.0),gr_complex(1.0,0.0)},
{gr_complex(-0.0,-1.0),gr_complex(0.0,1.0),gr_complex(1.0,-0.0),gr_complex(1.0,-0.0),gr_complex(0.0,1.0),gr_complex(-0.0,-1.0),gr_complex(1.0,-0.0),gr_complex(1.0,0.0)},
{gr_complex(-1.0,0.0),gr_complex(0.0,1.0),gr_complex(-0.0,-1.0),gr_complex(1.0,-0.0),gr_complex(1.0,-0.0),gr_complex(-0.0,-1.0),gr_complex(-0.0,-1.0),gr_complex(1.0,0.0)},
{gr_complex(1.0,-0.0),gr_complex(0.0,1.0),gr_complex(0.0,1.0),gr_complex(1.0,-0.0),gr_complex(-1.0,0.0),gr_complex(-0.0,-1.0),gr_complex(0.0,1.0),gr_complex(1.0,0.0)},
{gr_complex(0.0,1.0),gr_complex(0.0,1.0),gr_complex(0.0,1.0),gr_complex(-0.0,-1.0),gr_complex(1.0,0.0),gr_complex(1.0,0.0),gr_complex(-1.0,0.0),gr_complex(1.0,0.0)},
{gr_complex(-0.0,-1.0),gr_complex(0.0,1.0),gr_complex(-0.0,-1.0),gr_complex(-0.0,-1.0),gr_complex(-1.0,0.0),gr_complex(1.0,0.0),gr_complex(1.0,-0.0),gr_complex(1.0,0.0)},
{gr_complex(-1.0,0.0),gr_complex(0.0,1.0),gr_complex(-1.0,0.0),gr_complex(-0.0,-1.0),gr_complex(0.0,1.0),gr_complex(1.0,0.0),gr_complex(-0.0,-1.0),gr_complex(1.0,0.0)},
{gr_complex(1.0,-0.0),gr_complex(0.0,1.0),gr_complex(1.0,-0.0),gr_complex(-0.0,-1.0),gr_complex(-0.0,-1.0),gr_complex(1.0,0.0),gr_complex(0.0,1.0),gr_complex(1.0,0.0)},
{gr_complex(-0.0,-1.0),gr_complex(-0.0,-1.0),gr_complex(0.0,1.0),gr_complex(-0.0,-1.0),gr_complex(-1.0,0.0),gr_complex(-1.0,0.0),gr_complex(-1.0,0.0),gr_complex(1.0,0.0)},
{gr_complex(0.0,1.0),gr_complex(-0.0,-1.0),gr_complex(-0.0,-1.0),gr_complex(-0.0,-1.0),gr_complex(1.0,-0.0),gr_complex(-1.0,0.0),gr_complex(1.0,-0.0),gr_complex(1.0,0.0)},
{gr_complex(1.0,-0.0),gr_complex(-0.0,-1.0),gr_complex(-1.0,0.0),gr_complex(-0.0,-1.0),gr_complex(-0.0,-1.0),gr_complex(-1.0,0.0),gr_complex(-0.0,-1.0),gr_complex(1.0,0.0)},
{gr_complex(-1.0,0.0),gr_complex(-0.0,-1.0),gr_complex(1.0,-0.0),gr_complex(-0.0,-1.0),gr_complex(0.0,1.0),gr_complex(-1.0,0.0),gr_complex(0.0,1.0),gr_complex(1.0,0.0)},
{gr_complex(-1.0,0.0),gr_complex(-1.0,0.0),gr_complex(0.0,1.0),gr_complex(-0.0,-1.0),gr_complex(0.0,1.0),gr_complex(0.0,1.0),gr_complex(-1.0,0.0),gr_complex(1.0,0.0)},
{gr_complex(1.0,-0.0),gr_complex(-1.0,0.0),gr_complex(-0.0,-1.0),gr_complex(-0.0,-1.0),gr_complex(-0.0,-1.0),gr_complex(0.0,1.0),gr_complex(1.0,-0.0),gr_complex(1.0,0.0)},
{gr_complex(-0.0,-1.0),gr_complex(-1.0,0.0),gr_complex(-1.0,0.0),gr_complex(-0.0,-1.0),gr_complex(-1.0,0.0),gr_complex(0.0,1.0),gr_complex(-0.0,-1.0),gr_complex(1.0,0.0)},
{gr_complex(0.0,1.0),gr_complex(-1.0,0.0),gr_complex(1.0,-0.0),gr_complex(-0.0,-1.0),gr_complex(1.0,-0.0),gr_complex(0.0,1.0),gr_complex(0.0,1.0),gr_complex(1.0,0.0)},
{gr_complex(1.0,-0.0),gr_complex(1.0,-0.0),gr_complex(0.0,1.0),gr_complex(-0.0,-1.0),gr_complex(-0.0,-1.0),gr_complex(-0.0,-1.0),gr_complex(-1.0,0.0),gr_complex(1.0,0.0)},
{gr_complex(-1.0,0.0),gr_complex(1.0,-0.0),gr_complex(-0.0,-1.0),gr_complex(-0.0,-1.0),gr_complex(0.0,1.0),gr_complex(-0.0,-1.0),gr_complex(1.0,-0.0),gr_complex(1.0,0.0)},
{gr_complex(0.0,1.0),gr_complex(1.0,-0.0),gr_complex(-1.0,0.0),gr_complex(-0.0,-1.0),gr_complex(1.0,-0.0),gr_complex(-0.0,-1.0),gr_complex(-0.0,-1.0),gr_complex(1.0,0.0)},
{gr_complex(-0.0,-1.0),gr_complex(1.0,-0.0),gr_complex(1.0,-0.0),gr_complex(-0.0,-1.0),gr_complex(-1.0,0.0),gr_complex(-0.0,-1.0),gr_complex(0.0,1.0),gr_complex(1.0,0.0)},
{gr_complex(-0.0,-1.0),gr_complex(-0.0,-1.0),gr_complex(-0.0,-1.0),gr_complex(0.0,1.0),gr_complex(1.0,0.0),gr_complex(1.0,0.0),gr_complex(-1.0,0.0),gr_complex(1.0,0.0)},
{gr_complex(0.0,1.0),gr_complex(-0.0,-1.0),gr_complex(0.0,1.0),gr_complex(0.0,1.0),gr_complex(-1.0,0.0),gr_complex(1.0,0.0),gr_complex(1.0,-0.0),gr_complex(1.0,0.0)},
{gr_complex(1.0,-0.0),gr_complex(-0.0,-1.0),gr_complex(1.0,-0.0),gr_complex(0.0,1.0),gr_complex(0.0,1.0),gr_complex(1.0,0.0),gr_complex(-0.0,-1.0),gr_complex(1.0,0.0)},
{gr_complex(-1.0,0.0),gr_complex(-0.0,-1.0),gr_complex(-1.0,0.0),gr_complex(0.0,1.0),gr_complex(-0.0,-1.0),gr_complex(1.0,0.0),gr_complex(0.0,1.0),gr_complex(1.0,0.0)},
{gr_complex(0.0,1.0),gr_complex(0.0,1.0),gr_complex(-0.0,-1.0),gr_complex(0.0,1.0),gr_complex(-1.0,0.0),gr_complex(-1.0,0.0),gr_complex(-1.0,0.0),gr_complex(1.0,0.0)},
{gr_complex(-0.0,-1.0),gr_complex(0.0,1.0),gr_complex(0.0,1.0),gr_complex(0.0,1.0),gr_complex(1.0,-0.0),gr_complex(-1.0,0.0),gr_complex(1.0,-0.0),gr_complex(1.0,0.0)},
{gr_complex(-1.0,0.0),gr_complex(0.0,1.0),gr_complex(1.0,-0.0),gr_complex(0.0,1.0),gr_complex(-0.0,-1.0),gr_complex(-1.0,0.0),gr_complex(-0.0,-1.0),gr_complex(1.0,0.0)},
{gr_complex(1.0,-0.0),gr_complex(0.0,1.0),gr_complex(-1.0,0.0),gr_complex(0.0,1.0),gr_complex(0.0,1.0),gr_complex(-1.0,0.0),gr_complex(0.0,1.0),gr_complex(1.0,0.0)},
{gr_complex(1.0,-0.0),gr_complex(1.0,-0.0),gr_complex(-0.0,-1.0),gr_complex(0.0,1.0),gr_complex(0.0,1.0),gr_complex(0.0,1.0),gr_complex(-1.0,0.0),gr_complex(1.0,0.0)},
{gr_complex(-1.0,0.0),gr_complex(1.0,-0.0),gr_complex(0.0,1.0),gr_complex(0.0,1.0),gr_complex(-0.0,-1.0),gr_complex(0.0,1.0),gr_complex(1.0,-0.0),gr_complex(1.0,0.0)},
{gr_complex(0.0,1.0),gr_complex(1.0,-0.0),gr_complex(1.0,-0.0),gr_complex(0.0,1.0),gr_complex(-1.0,0.0),gr_complex(0.0,1.0),gr_complex(-0.0,-1.0),gr_complex(1.0,0.0)},
{gr_complex(-0.0,-1.0),gr_complex(1.0,-0.0),gr_complex(-1.0,0.0),gr_complex(0.0,1.0),gr_complex(1.0,-0.0),gr_complex(0.0,1.0),gr_complex(0.0,1.0),gr_complex(1.0,0.0)},
{gr_complex(-1.0,0.0),gr_complex(-1.0,0.0),gr_complex(-0.0,-1.0),gr_complex(0.0,1.0),gr_complex(-0.0,-1.0),gr_complex(-0.0,-1.0),gr_complex(-1.0,0.0),gr_complex(1.0,0.0)},
{gr_complex(1.0,-0.0),gr_complex(-1.0,0.0),gr_complex(0.0,1.0),gr_complex(0.0,1.0),gr_complex(0.0,1.0),gr_complex(-0.0,-1.0),gr_complex(1.0,-0.0),gr_complex(1.0,0.0)},
{gr_complex(-0.0,-1.0),gr_complex(-1.0,0.0),gr_complex(1.0,-0.0),gr_complex(0.0,1.0),gr_complex(1.0,-0.0),gr_complex(-0.0,-1.0),gr_complex(-0.0,-1.0),gr_complex(1.0,0.0)},
{gr_complex(0.0,1.0),gr_complex(-1.0,0.0),gr_complex(-1.0,0.0),gr_complex(0.0,1.0),gr_complex(-1.0,0.0),gr_complex(-0.0,-1.0),gr_complex(0.0,1.0),gr_complex(1.0,0.0)}
};
    class chip_sync_c_impl : public chip_sync_c
    {
      enum RXSTATE{
        SEARCH,
        SYNC,
        PSDU
      };
      enum PSDU_TYPE{
        LONG1M,
        DSSS2M,
        CCK5_5M,
        CCK11M
      };
     private:
      gr::thread::mutex d_mutex;
      bool d_changeType;

      bool d_chip_sync;
      float d_threshold;
      bool d_preType;
      // for receiver
      int d_rx_state;
      uint64_t d_hdr_buf;
      unsigned int d_hdr_reg;
      uint16_t d_hdr_crc;

      int d_chip_wait;
      // for psdu
      uint8_t d_sig_dec;
      uint8_t d_service_dec;
      uint8_t d_length_dec;
      float d_rate_val;
      int d_bit_len;
      int d_psdu_bytes_len;
      int d_psdu_bit_cnt;
      // for sync
      float d_phase;
      float d_freq;
      float d_alpha;
      float d_beta;
      float d_crit;
      float d_loopbw;

      uint16_t d_sync_reg;
      uint8_t d_byte_reg;
      int d_bit_cnt;
      gr_complex d_prev_sym;
      uint16_t d_sync_word;
      unsigned char d_buf[2048];
      const pmt::pmt_t d_psdu_out;
      gr_complex* d_chip_buf;
      int d_chip_cnt;
      int d_psdu_chip_size;
      int d_psdu_type;
      int d_psdu_sym_cnt;
      // descrambler
      uint8_t d_des_state;
      uint8_t descrambler(const uint8_t& rawbit);
      void enter_search();
      void enter_sync();
      void enter_psdu();
      bool check_hdr();
      uint16_t psdu_get_symbol(const gr_complex* in,bool isEven);
      uint8_t ppdu_get_symbol(const gr_complex* in);
      void psdu_write_bits(const uint16_t& outByte);

      gr_complex pll_bpsk(const gr_complex& in);
      gr_complex pll_qpsk(const gr_complex& in);
      void reset_pll();
     public:
      chip_sync_c_impl(bool longPre, float threshold);
      ~chip_sync_c_impl();
      void set_preamble_type(bool islong);
      // Where all the action really happens
      void forecast (int noutput_items, gr_vector_int &ninput_items_required);

      int general_work(int noutput_items,
           gr_vector_int &ninput_items,
           gr_vector_const_void_star &input_items,
           gr_vector_void_star &output_items);
    };

  } // namespace wifi_dsss
} // namespace gr

#endif /* INCLUDED_WIFI_DSSS_CHIP_SYNC_C_IMPL_H */

