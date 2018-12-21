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

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <gnuradio/io_signature.h>
#include "chip_sync_c_impl.h"
#include <volk/volk.h>
#include <gnuradio/math.h>
#include <gnuradio/expj.h>

namespace gr {
  namespace wifi_dsss {
    #define d_debug 0
    #define dout d_debug && std::cout
    #define TWO_PI M_PI*2.0f
    static const float d_barker[11]={1,-1,1,1,-1,1,1,1,-1,-1,-1};
    static const uint8_t d_dqpsk_2m_map[4] = {0x00,0x02,0x03,0x01};
    static const uint8_t d_cck_dqpsk_map[2][4] = {
                                                  {0x00,0x02,0x03,0x01},
                                                  {0x03,0x01,0x00,0x02}
                                                };
    static const float d_cck_thres_adjust = 16.0*std::sqrt(22.0)/121.0;
    static inline float phase_wrap(float phase)
    {
      while(phase>TWO_PI)
        phase-= TWO_PI;
      while(phase<-TWO_PI)
        phase+= TWO_PI;
      return phase;
    }
    static inline float freq_limit(float freq)
    {
      float x1 = std::fabs(freq+ 1.0);
      float x2 = std::fabs(freq- 1.0);
      x1-=x2;
      return 0.5 * x1;
    }
    chip_sync_c::sptr
    chip_sync_c::make(bool longPre,float threshold)
    {
      return gnuradio::get_initial_sptr
        (new chip_sync_c_impl(longPre, threshold));
    }

    /*
     * The private constructor
     */
    chip_sync_c_impl::chip_sync_c_impl(bool longPre,float threshold)
      : gr::block("chip_sync_c",
              gr::io_signature::make(1, 1, sizeof(gr_complex)),
              gr::io_signature::make(0, 0, 0)),
              d_preType(longPre),
              d_psdu_out(pmt::mp("psdu_out"))
    {
      d_changeType = true;
      d_preType = !longPre;
      
      message_port_register_out(d_psdu_out);
      enter_search();
      d_chip_buf = (gr_complex*) volk_malloc(sizeof(gr_complex)*64,volk_get_alignment());
      if(threshold<0){
        throw std::invalid_argument("Threshold should be positive number");
      }else if(threshold>11){
        d_threshold = std::sqrt(11.0);
      }else{
        d_threshold = threshold / std::sqrt(11.0);
      }
      d_crit = std::sqrt(0.5);
      d_loopbw = 0.0314;
      float denum = 1 + 2*d_crit*d_loopbw + d_loopbw*d_loopbw;
      d_alpha = 4*d_crit*d_loopbw/denum;
      d_beta = 4*d_loopbw*d_loopbw/denum;
      
    }

    /*
     * Our virtual destructor.
     */
    chip_sync_c_impl::~chip_sync_c_impl()
    {
      volk_free(d_chip_buf);
    }

    void
    chip_sync_c_impl::forecast (int noutput_items, gr_vector_int &ninput_items_required)
    {
      ninput_items_required[0] = noutput_items + 11;
    }

    void
    chip_sync_c_impl::set_preamble_type(bool islong)
    {
      gr::thread::scoped_lock guard(d_mutex);
      if(islong == d_preType){
        d_changeType = false;
      }else{
        d_changeType = true;
      }
    }
    void
    chip_sync_c_impl::enter_search()
    {
      if(d_changeType){
        d_changeType = false;
        d_preType = !d_preType;
      }
      d_sync_word = (d_preType)? 0x05CF : 0xF3A0;
      d_des_state = (d_preType)? 0x1B : 0x6C;
      d_hdr_bps = (d_preType)? 1 : 2;
      d_hdr_get_bits = (d_preType)? &chip_sync_c_impl::get_symbol_dbpsk : 
                                  &chip_sync_c_impl::get_symbol_dqpsk ;
      d_chip_sync = false;
      d_rx_state = SEARCH;
      d_prev_sym = gr_complex(1.0,0);
      reset_pll();
    }
    void
    chip_sync_c_impl::enter_sync()
    {
      d_hdr_reg = 0x00000000;
      d_hdr_crc = 0x0000;
      d_bit_cnt = 0;
      d_chip_wait = 0;
      d_rx_state = SYNC;
    }
    void
    chip_sync_c_impl::enter_psdu()
    {
      d_bit_cnt =0;
      d_rx_state = PSDU;
      d_psdu_sym_cnt =0;
      d_psdu_bit_cnt =0;
    }
    uint8_t
    chip_sync_c_impl::descrambler(const uint8_t& rawbit)
    {
      uint8_t x1=(d_des_state>>3) & 0x01;
      uint8_t x2=(d_des_state>>6) & 0x01;
      d_des_state = (d_des_state<<1) | rawbit;
      return x1^ x2 ^ rawbit;
    }
    bool
    chip_sync_c_impl::check_hdr()
    {
      // crc check
      uint16_t crc_init = 0xffff;
      for(int i=0;i<32;++i){
        uint16_t newBit = (d_hdr_reg>>i) & 0x0001;
        uint16_t newLsb = (crc_init>>15) ^ newBit;
        uint16_t tmpXor = (newLsb)? 0x0810 : 0x0000;
        crc_init ^= tmpXor;
        crc_init = (crc_init<<1) | newLsb;
      }
      if( (crc_init ^d_hdr_crc) == 0xffff){
        // passed
      }else{
        return false;
      }
      // crc passed
      // reading header information
      d_sig_dec = d_hdr_reg & 0xff;
      d_service_dec = (d_hdr_reg >> 8) & 0xff;
      d_length_dec = (d_hdr_reg>>16) & 0xffff;
      // find signal field
      if(d_sig_dec == 0x0A){
        if(!d_preType){
          return false;
        }else{
          // 1m
          d_rate_val = 1.0;
          d_psdu_bytes_len = (int) floor(d_length_dec/8.0);
          d_psdu_chip_size = 11;
          d_psdu_type = LONG1M;
          d_get_symbol_fptr = & chip_sync_c_impl::get_symbol_dbpsk;
          dout<<"Header Checked, Rate LONG DSSS1M detected, byte_len="<<d_psdu_bytes_len<<std::endl;
        }
      }else if(d_sig_dec == 0x14){
        // 2m
        d_rate_val = 2.0;
        d_psdu_bytes_len = (int) floor(d_length_dec * d_rate_val/8.0);
        d_psdu_chip_size = 11;
        d_psdu_type = DSSS2M;
        d_get_symbol_fptr = & chip_sync_c_impl::get_symbol_dqpsk;
        dout<<"Header Checked, Rate DSSS2M detected, byte_len="<<d_psdu_bytes_len<<std::endl;
      }else if(d_sig_dec == 0x37){
        // 5.5m
        d_rate_val = 5.5;
        d_psdu_bytes_len =(int) floor(d_length_dec * d_rate_val/8.0);
        d_psdu_chip_size = 8;
        d_psdu_type = CCK5_5M;
        d_get_symbol_fptr = & chip_sync_c_impl::get_symbol_cck5_5;
        dout<<"Header Checked, Rate DSSS5.5M detected, byte_len="<<d_psdu_bytes_len<<std::endl;
      }else if(d_sig_dec == 0x6e){
        // 11m
        d_rate_val = 11;
        // first check service length field
        d_psdu_bytes_len = (int) floor(d_length_dec * 11/8.0);
        // check here
        if( (d_service_dec >> 6 ) & 0x01 ){
          dout<<"Receiver: DSSS11M extended bits true"<<std::endl;
        }else{
          dout<<"Receiver: DSSS11M extended bits false"<<std::endl;
        }
        d_psdu_bytes_len = ( (d_service_dec>>6) & 0x01)? d_psdu_bytes_len-1 : d_psdu_bytes_len;
        d_psdu_chip_size = 8;
        d_psdu_type = CCK11M;
        d_get_symbol_fptr = & chip_sync_c_impl::get_symbol_cck11;
        dout<<"Header Checked, Rate DSSS11M detected, byte_len="<<d_psdu_bytes_len<<std::endl;
      }else{
        return false;
      }
      return true;
    }
    uint16_t
    chip_sync_c_impl::get_symbol_dbpsk(const gr_complex* in, bool isEven)
    {
      gr_complex tmpVal,diff,in_eg;
      float phase_diff;
      volk_32fc_x2_conjugate_dot_prod_32fc(&in_eg,in,in,11);
      volk_32fc_32f_dot_prod_32fc(&tmpVal,in,d_barker,11);
      tmpVal/= (std::sqrt(in_eg)+gr_complex(1e-8,0));
      tmpVal = pll_bpsk(tmpVal);
      if(std::abs(tmpVal)>=d_threshold){
        diff = tmpVal * std::conj(d_prev_sym);
        d_prev_sym = tmpVal;
        phase_diff = std::atan2(diff.imag(),diff.real())/(0.5*M_PI);
        return (std::fabs(phase_diff)>1.0)? 0x01 : 0x00;
      }
      return 0xffff;
    }
    uint16_t
    chip_sync_c_impl::get_symbol_dqpsk(const gr_complex* in, bool isEven)
    {
      int max_idx =0;
      gr_complex tmpVal,diff,in_eg;
      float phase_diff;
      volk_32fc_x2_conjugate_dot_prod_32fc(&in_eg,in,in,11);
      volk_32fc_32f_dot_prod_32fc(&tmpVal, in,d_barker,11);
      tmpVal/=(std::sqrt(in_eg)+gr_complex(1e-8,0));
      tmpVal = pll_qpsk(tmpVal);
      if(std::abs(tmpVal)>=d_threshold){
        diff = tmpVal * std::conj(d_prev_sym);
        d_prev_sym = tmpVal;
        phase_diff = std::atan2(diff.imag(),diff.real());
        phase_diff = (phase_diff<0)? phase_diff + TWO_PI : phase_diff;
        max_idx = (int) std::round(phase_diff/(0.5*M_PI))%4;
        return d_dqpsk_2m_map[max_idx];
      }
      return 0xffff;
    }
    uint16_t
    chip_sync_c_impl::get_symbol_cck5_5(const gr_complex* in, bool isEven)
    {
      float max_corr = 0;
      uint8_t max_idx =0;
      gr_complex tmpVal,holdVal,diff,in_eg;
      float phase_diff;
      for(int i=0;i<4;++i){
        volk_32fc_x2_conjugate_dot_prod_32fc(&holdVal, in, d_cck5_5_chips[i],8);
        if(std::abs(holdVal)>max_corr){
          max_corr = std::abs(holdVal);
          max_idx = (uint8_t) i;
          tmpVal = holdVal;
        }
      }
      volk_32fc_x2_conjugate_dot_prod_32fc(&in_eg,in,in,8);
      tmpVal/=(std::sqrt(in_eg)+gr_complex(1e-8,0));
      tmpVal = pll_qpsk(tmpVal);
      if(std::abs(tmpVal)>= d_threshold*d_cck_thres_adjust){
        diff = tmpVal * std::conj(d_prev_sym);
        d_prev_sym = tmpVal;
        phase_diff = std::atan2(diff.imag(),diff.real());
        phase_diff = (phase_diff<0)? phase_diff + TWO_PI : phase_diff;
        uint8_t cckd01 = (uint8_t) std::round(phase_diff/(0.5*M_PI))%4;
        cckd01 = (isEven)? d_cck_dqpsk_map[0][cckd01] : d_cck_dqpsk_map[1][cckd01];
        return  (cckd01 | (max_idx<<2) ) & 0x0f;
      }
      return 0xffff;
    }
    uint16_t
    chip_sync_c_impl::get_symbol_cck11(const gr_complex* in, bool isEven)
    {
      float max_corr = 0;
      uint8_t max_idx =0;
      gr_complex tmpVal,holdVal,diff,in_eg;
      float phase_diff;
      for(int i=0;i<64;++i){
        volk_32fc_x2_conjugate_dot_prod_32fc(&holdVal,in,d_cck11_chips[i],8);
        if(std::abs(holdVal)>max_corr){
          max_corr = std::abs(holdVal);
          max_idx = (uint8_t) i;
          tmpVal = holdVal;
        }
      }
      volk_32fc_x2_conjugate_dot_prod_32fc(&in_eg,in,in,8);
      tmpVal/=(std::sqrt(in_eg)+gr_complex(1e-8,0));
      tmpVal = pll_qpsk(tmpVal);
      if(std::abs(tmpVal)>= d_threshold*d_cck_thres_adjust){
        diff = tmpVal * std::conj(d_prev_sym);
        d_prev_sym = tmpVal;
        phase_diff = std::atan2(diff.imag(),diff.real());
        phase_diff = (phase_diff<0)? phase_diff + TWO_PI : phase_diff;
        uint8_t cckd01 = (uint8_t) std::round(phase_diff/(0.5*M_PI))%4;
        cckd01 = (isEven)? d_cck_dqpsk_map[0][cckd01] : d_cck_dqpsk_map[1][cckd01];
        return ( cckd01 | (max_idx<<2) ) & 0xff;
      }
      return 0xffff;
    }
    void
    chip_sync_c_impl::psdu_write_bits(const uint16_t& outByte)
    {
      uint8_t boffset = d_psdu_bit_cnt %8;
      uint8_t bpos = d_psdu_bit_cnt/8;
      uint8_t curByte = outByte & 0x00ff;
      uint8_t debit;
      d_buf[bpos] = (boffset==0)? 0x00:d_buf[bpos];
      switch(d_psdu_type){
        case LONG1M:
          debit = descrambler(curByte);
          d_buf[bpos] |= (debit<<boffset);
          d_psdu_bit_cnt++;
        break;
        case DSSS2M:
          for(int i=0;i<2;++i){
            debit = descrambler( (curByte>>i) & 0x01 );
            d_buf[bpos] |= (debit<<boffset++);
          }
          d_psdu_bit_cnt+=2;
        break;
        case CCK5_5M:
          for(int i=0;i<4;++i){
            debit = descrambler( (curByte>>i) & 0x01 );
            d_buf[bpos] |= (debit<<boffset++);
          }
          d_psdu_bit_cnt+=4;
        break;
        case CCK11M:
          for(int i=0;i<8;++i){
            debit = descrambler( (curByte>>i) & 0x01 );
            d_buf[bpos] |= (debit<<boffset++);
          }
          d_psdu_bit_cnt+=8;
        break;
        default:
          throw std::runtime_error("undefined state");
        break;
      }
    }
    gr_complex
    chip_sync_c_impl::pll_bpsk(const gr_complex& in)
    {
      gr_complex nco_est = in * gr_expj(-d_phase);
      float error = real(nco_est)*imag(nco_est);
      d_freq = d_freq + error * d_beta;
      d_phase = d_phase + d_freq + error* d_alpha;
      d_phase = phase_wrap(d_phase);
      d_freq = freq_limit(d_freq);
      return nco_est;
    }
    gr_complex
    chip_sync_c_impl::pll_qpsk(const gr_complex& in)
    {
      gr_complex nco_est = in * gr_expj(-d_phase);
      float error = ((nco_est.real()>0)? 1.0 : -1.0 )*nco_est.imag() - 
          ((nco_est.imag()>0)?1.0:-1.0)*nco_est.real();
      d_freq = d_freq + error * d_beta;
      d_phase = d_phase + d_freq + error*d_alpha;
      d_phase = phase_wrap(d_phase);
      d_freq = freq_limit(d_freq);
      return nco_est;
    }
    void
    chip_sync_c_impl::reset_pll()
    {
      d_phase =0;
      d_freq = 0;
    }
    int
    chip_sync_c_impl::general_work (int noutput_items,
                       gr_vector_int &ninput_items,
                       gr_vector_const_void_star &input_items,
                       gr_vector_void_star &output_items)
    {
      const gr_complex *in = (const gr_complex *) input_items[0];
      int nin = ninput_items[0]-11;
      int ncon = 0;
      gr_complex autoVal,diff, in_eg;
      float phase_diff;
      uint16_t tmpbit;
      if(d_changeType){
        enter_search();
        consume_each(0);
        return 0;
      }
      while(ncon<nin){
        switch(d_rx_state){
          case SEARCH:
            while(ncon<nin){
              if(!d_chip_sync){
                volk_32fc_x2_conjugate_dot_prod_32fc(&in_eg,&in[ncon],&in[ncon],11);
                volk_32fc_32f_dot_prod_32fc(&autoVal,&in[ncon++],d_barker,11);
                autoVal/=(std::sqrt(in_eg)+gr_complex(1e-8,0)); // avoiding overflow
                if(std::abs(autoVal)>=d_threshold){
                  d_chip_sync = true;
                  d_chip_cnt = 0;
                  d_prev_sym = pll_bpsk(autoVal);
                }
              }else{
                d_chip_cnt++;
                if(d_chip_cnt==11){
                  d_chip_cnt =0;
                  tmpbit = get_symbol_dbpsk(&in[ncon++],true);
                  if(tmpbit==0xffff){
                    d_chip_sync= false;
                    d_prev_sym = gr_complex(1.0,0);
                    reset_pll();
                  }else{
                    uint8_t deBit = descrambler( (uint8_t) tmpbit & 0x01);
                    d_sync_reg = (d_sync_reg<<1) | deBit;
                    if(d_sync_reg == d_sync_word){
                      enter_sync();
                      break;
                    }
                  }
                }else{
                  ncon++;
                }
              }
            }
          break;
          case SYNC:
            while(ncon<nin){
              if(d_chip_wait==0){
                d_chip_cnt++;
                if(d_chip_cnt==11){
                  d_chip_cnt =0;
                  tmpbit = (*this.*d_hdr_get_bits)(&in[ncon++],true);
                  if(tmpbit == 0xffff){
                    enter_search();
                    break;
                  }else{
                    for(int i=0;i<d_hdr_bps;i++){
                      uint8_t oneBit = (tmpbit>>i & 0x01);
                      uint8_t deBit = descrambler(oneBit);
                      if(d_bit_cnt<32){
                        d_hdr_reg |= (deBit<<d_bit_cnt);
                      }else{
                        d_hdr_crc |= (deBit<<(d_bit_cnt-32));
                      }
                      d_bit_cnt++;
                    }
                    if(d_bit_cnt == 48){
                      d_bit_cnt =0;
                      // including crc
                      if(check_hdr()){
                        d_chip_wait = 10;
                      }else{
                        enter_search();
                        break;
                      }
                    }
                  }
                }else{
                  ncon++;
                }
              }else{
                // sync passed, but waiting due to rate change
                ncon++;
                d_chip_wait--;
                if(d_chip_wait==0){
                  enter_psdu();
                  d_chip_cnt = d_psdu_chip_size-1;
                  break;
                }
              }
              
            }
          break;
          case PSDU:
            while(ncon<nin){
              d_chip_cnt++;
              if(d_chip_cnt == d_psdu_chip_size){
                d_chip_cnt =0;
                uint16_t outByte = (*this.*d_get_symbol_fptr)(&in[ncon++] , ( (d_psdu_sym_cnt++)%2 == 0) );
                if(outByte==0xffff){
                  enter_search();
                  break;
                }else{
                  // insert bits into buffer 
                  psdu_write_bits(outByte);
                  if(d_psdu_bit_cnt==d_psdu_bytes_len*8){
                    // complete reception
                    pmt::pmt_t psdu_msg = pmt::make_blob(d_buf,d_psdu_bytes_len);
                    message_port_pub(d_psdu_out,pmt::cons(pmt::PMT_NIL,psdu_msg));
                    enter_search();
                    break;
                  }
                }
              }else{
                ncon++;
              }
            }
          break;
          default:
            throw std::runtime_error("undefined state");
          break;
        }  
      }
      consume_each (ncon);
      return 0;
    }

  } /* namespace wifi_dsss */
} /* namespace gr */

