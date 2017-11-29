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

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <gnuradio/io_signature.h>
#include "ppdu_chip_mapper_bc_impl.h"
#include <gnuradio/math.h>
#include <gnuradio/expj.h>
#include <volk/volk.h>

namespace gr {
  namespace wifi_dsss {
    #define d_debug 0
    #define dout d_debug && std::cout
    #define TWO_PI M_PI*2.0f 
    #define LONG_PREAMBLE_LENGTH 24
    #define SHORT_PREAMBLE_LENGTH 15

    #define APPENDED_CHIPS 11
    inline float phase_wrap(float phase)
    {
      while(phase>=TWO_PI)
        phase -= TWO_PI;
      while(phase<=-TWO_PI)
        phase += TWO_PI;
      return phase;
    }
    inline void cck_gen(gr_complex* out,float p1,float p2, float p3, float p4)
    {
      out[0] = gr_expj(phase_wrap(p1+p2+p3+p4));
      out[1] = gr_expj(phase_wrap(p1+p3+p4));
      out[2] = gr_expj(phase_wrap(p1+p2+p4));
      out[3] = gr_expj(phase_wrap(p1+p4+M_PI));
      out[4] = gr_expj(phase_wrap(p1+p2+p3));
      out[5] = gr_expj(phase_wrap(p1+p3));
      out[6] = gr_expj(phase_wrap(p1+p2+M_PI));
      out[7] = gr_expj(phase_wrap(p1));
    }
    int 
    ppdu_chip_mapper_bc_impl::nout_check() const
    {
      switch(d_rate){
        case LONG1M:
          return 88;
        break;
        case LONG2M:
        case SHORT2M:
          return 44;
        break;
        case LONG5_5M:
        case SHORT5_5M:
          return 16;
        break;
        case LONG11M:
        case SHORT11M:
          return 8;
        break;
        default:
          throw std::runtime_error("Undefined state");
        break;
      }
    }
    static const float d_barker[11]={1,-1,1,1,-1,1,1,1,-1,-1,-1};
    static const float d_cck_dqpsk_phase[4][2]={ {0,M_PI},
                                             {1.5*M_PI,0.5*M_PI},
                                             {0.5*M_PI,1.5*M_PI},
                                             {M_PI,0}
                                                    };
    static const float d_dbpsk_phase[2]={0,M_PI};
    static const float d_dqpsk_phase[4]={0,1.5*M_PI,0.5*M_PI,M_PI};
    static const float d_cck_qpsk[4] = {0,M_PI,0.5*M_PI,1.5*M_PI};

    static const gr_complex d_append_symbols[11] = {gr_complex(1,0),gr_complex(-1,0),gr_complex(1,0),
                                                    gr_complex(1,0),gr_complex(-1,0),gr_complex(1,0),
                                                    gr_complex(1,0),gr_complex(1,0),gr_complex(-1,0),
                                                    gr_complex(-1,0),gr_complex(-1,0)};
    ppdu_chip_mapper_bc::sptr
    ppdu_chip_mapper_bc::make(int rate, const std::string& lentag)
    {
      return gnuradio::get_initial_sptr
        (new ppdu_chip_mapper_bc_impl(rate,lentag));
    }

    /*
     * The private constructor
     */
    ppdu_chip_mapper_bc_impl::ppdu_chip_mapper_bc_impl(
      int rate, const std::string& lentag)
      : gr::block("ppdu_chip_mapper_bc",
              gr::io_signature::make(1, 1, sizeof(char)),
              gr::io_signature::make(1, 1, sizeof(gr_complex))),
              d_lentag(pmt::intern(lentag)),
              d_name(pmt::intern(alias()))
    {
      switch(rate)
      {
        case LONG1M:
          d_rate_tag = pmt::intern("LONG1M");
          d_rateVal = 1;
          d_preCnt = LONG_PREAMBLE_LENGTH;
          d_chip_mapper = &ppdu_chip_mapper_bc_impl::dbpsk_1M_chips;
          d_preType = true;
        break;
        case LONG2M:
          d_rate_tag = pmt::intern("LONG2M");
          d_rateVal = 2;
          d_preCnt = LONG_PREAMBLE_LENGTH;
          d_chip_mapper = &ppdu_chip_mapper_bc_impl::dqpsk_2M_chips;
          d_preType = true;
        break;
        case LONG5_5M:
          d_rate_tag = pmt::intern("LONG5_5M");
          d_rateVal = 5.5;
          d_preCnt = LONG_PREAMBLE_LENGTH;
          d_chip_mapper = &ppdu_chip_mapper_bc_impl::cck_5_5M_chips;
          d_preType = true;
        break;
        case LONG11M:
          d_rate_tag = pmt::intern("LONG11M");
          d_rateVal = 11;
          d_preCnt = LONG_PREAMBLE_LENGTH;
          d_chip_mapper = &ppdu_chip_mapper_bc_impl::cck_11M_chips;
          d_preType = true;
        break;
        case SHORT2M:
          d_rate_tag = pmt::intern("SHORT2M");
          d_rateVal = 2;
          d_preCnt = LONG_PREAMBLE_LENGTH;
          d_chip_mapper = &ppdu_chip_mapper_bc_impl::dqpsk_2M_chips;
          d_preType = false;
        break;
        case SHORT5_5M:
          d_rate_tag = pmt::intern("SHORT5_5M");
          d_rateVal = 5.5;
          d_preCnt = LONG_PREAMBLE_LENGTH;
          d_chip_mapper = &ppdu_chip_mapper_bc_impl::cck_5_5M_chips;
          d_preType = false;
        break;
        case SHORT11M:
          d_rate_tag = pmt::intern("SHORT11M");
          d_rateVal = 11;
          d_preCnt = LONG_PREAMBLE_LENGTH;
          d_chip_mapper = &ppdu_chip_mapper_bc_impl::cck_11M_chips;
          d_preType = false;
        break;
        default:
          throw std::invalid_argument("Unrecognized rate type");
        break;
      }
      d_rate = rate;
      d_count =0;
      d_append = APPENDED_CHIPS;
      set_tag_propagation_policy(TPP_DONT);
    }

    /*
     * Our virtual destructor.
     */
    ppdu_chip_mapper_bc_impl::~ppdu_chip_mapper_bc_impl()
    {
    }

    void
    ppdu_chip_mapper_bc_impl::forecast (int noutput_items, gr_vector_int &ninput_items_required)
    {
      if(d_preType && d_copy<LONG_PREAMBLE_LENGTH){
        ninput_items_required[0] = (int) floor(noutput_items/(float)88.0);
      }else if(!d_preType && d_copy<SHORT_PREAMBLE_LENGTH){
        ninput_items_required[0] = (int) floor(noutput_items/(float)88.0);
      }else{
        switch(d_rate){
          case LONG1M:
          case LONG2M:
          case SHORT2M:
            ninput_items_required[0] = (int) floor(noutput_items/(float)88.0*d_rate);
          break;
          case LONG5_5M:
          case SHORT5_5M:
            ninput_items_required[0] = (int) floor(noutput_items/(float)16.0);
          break;
          case LONG11M:
          case SHORT11M:
            ninput_items_required[0] = (int) floor(noutput_items/(float)8.0);
          break;
          default:
            throw std::runtime_error("undefined rate");
          break;
        }
      }
    }
    int
    ppdu_chip_mapper_bc_impl::cck_5_5M_chips(gr_complex* out, uint8_t byte, bool even)
    {
      float phase_00, phase_10;
      float phase_b2, phase_b4;
      float phase_q2, phase_q4;
      phase_00 = d_cck_dqpsk_phase[byte&0x03][0];
      phase_10 = d_cck_dqpsk_phase[ (byte>>4) & 0x03][1];
      phase_b2 = ((byte>>2) & 0x01)? 1.5*M_PI : 0.5*M_PI;
      phase_b4 = ((byte>>3) & 0x01)? M_PI : 0;
      phase_q2 = ((byte>>6) & 0x01)? 1.5*M_PI : 0.5*M_PI;
      phase_q4 = ((byte>>7) & 0x01)? M_PI : 0;
      d_phase_acc+=phase_00;
      d_phase_acc = phase_wrap(d_phase_acc);
      cck_gen(out,d_phase_acc,phase_b2,0,phase_b4);
      d_phase_acc+=phase_10;
      d_phase_acc = phase_wrap(d_phase_acc);
      cck_gen(out+8,d_phase_acc,phase_q2,0,phase_q4);
      return 16;
    }
    int
    ppdu_chip_mapper_bc_impl::cck_11M_chips(gr_complex* out, uint8_t byte, bool even)
    {
      float phase_0;
      float other[3];
      if(even){
        phase_0 = d_cck_dqpsk_phase[byte&0x03][0];
      }else{
        phase_0 = d_cck_dqpsk_phase[byte&0x03][1];
      }
      for(int i=0;i<3;++i){
        uint8_t tmpBit = (byte>>((i+1)*2)) & 0x03;
        other[i] = d_cck_qpsk[tmpBit];
      }
      d_phase_acc+=phase_0;
      d_phase_acc = phase_wrap(d_phase_acc);
      cck_gen(out,d_phase_acc,other[0],other[1],other[2]);
      return 8;
    }
    int
    ppdu_chip_mapper_bc_impl::dqpsk_2M_chips(gr_complex* out, uint8_t byte, bool even)
    {
      gr_complex tmpChip;
      for(int i=0;i<4;++i){
        uint8_t tmp_bits = (byte>>(2*i)) & 0x03;
        d_phase_acc += d_dqpsk_phase[tmp_bits];
        d_phase_acc = phase_wrap(d_phase_acc);
        tmpChip =gr_expj(d_phase_acc);
        for(int j=0;j<11;++j){
          out[i*11+j] = tmpChip * d_barker[j];
        }
      }
      return 44;
    }
    int
    ppdu_chip_mapper_bc_impl::dbpsk_1M_chips(gr_complex* out, uint8_t byte, bool even)
    {
      gr_complex tmpChip;
      for(int i=0;i<8;++i){
        uint8_t tmp_bit = (byte>>i) & 0x01;
        d_phase_acc += d_dbpsk_phase[tmp_bit];
        d_phase_acc = phase_wrap(d_phase_acc);
        tmpChip = gr_expj(d_phase_acc);
        // can use volk
        // current for simplicity, use simple method
        for(int j=0;j<11;++j){
          out[i*11+j]= tmpChip * d_barker[j];
        }
      }
      return 88;
    }
    int
    ppdu_chip_mapper_bc_impl::chipGen(
      gr_complex* out, 
      const unsigned char* in, 
      int noutput_items,
      int nin,
      int& nconsume)
    {
      int nout = 0;
      nconsume = 0;
      while(nconsume<nin && nout<noutput_items && d_copy<d_count)
      {
        if( (d_preType && d_copy<LONG_PREAMBLE_LENGTH)||
            (!d_preType && d_copy<SHORT_PREAMBLE_LENGTH) ){
          if(nout+88 >noutput_items){
            return nout;
          }else{
            nout += dbpsk_1M_chips(out+nout,in[nconsume++],true);
            d_copy++;
          }
        }else{
          if(nout+nout_check()> noutput_items){
            return nout;
          }else{
            bool oddOrEven = (d_copy%2)==0;// only used for 11M
            nout += (*this.*d_chip_mapper)(out+nout,in[nconsume++],oddOrEven);
            d_copy++;
          }        
        }
      }
      return nout;
    }
    int
    ppdu_chip_mapper_bc_impl::update_tag(int total_bytes) const
    {
      switch(d_rate){
        case LONG1M:
          return total_bytes*88;
        break;
        case LONG2M:
          return 24*88+(total_bytes-24)*44;
        break;
        case LONG5_5M:
          return 24*88+(total_bytes-24)*16;
        break;
        case LONG11M:
          return 24*88+(total_bytes-24)*8;
        break;
        case SHORT2M:
          return 15*88+(total_bytes-15)*44;
        break;
        case SHORT5_5M:
          return 15*88+(total_bytes-15)*16;
        break;
        case SHORT11M:
          return 15*88+(total_bytes-15)*8;
        break;
        default:
          return 0;
        break;
      }
    }

    int
    ppdu_chip_mapper_bc_impl::general_work (int noutput_items,
                       gr_vector_int &ninput_items,
                       gr_vector_const_void_star &input_items,
                       gr_vector_void_star &output_items)
    {
      const unsigned char *in = (const unsigned char *) input_items[0];
      gr_complex *out = (gr_complex *) output_items[0];
      int consume = 0;
      int nout = 0;
      std::vector<tag_t> tags;
      // add append
      if(d_count==0 && d_append==APPENDED_CHIPS){
        get_tags_in_window(tags,0,0,ninput_items[0],d_lentag);
        if(!tags.empty()){
          int offset = tags[0].offset - nitems_read(0);
          if(offset==0){
            d_count = pmt::to_long(tags[0].value);
            d_copy = 0;
            d_phase_acc = 0;
            int newLen = update_tag(d_count) + APPENDED_CHIPS;
            add_item_tag(0,nitems_written(0),d_lentag,pmt::from_long(newLen),d_name);
          }else{
            consume_each(offset);
            return 0;
          }
        }
      }
      /*
      if(d_copy<d_count){
        int nin = std::min(d_count-d_copy,ninput_items[0]);
        int ncon = 0;
        nout = chipGen(out,in,noutput_items,nin,ncon);
        if(d_count == d_copy){
          d_count = 0;
          d_copy  = 0;
        }
        consume_each(ncon);
        return nout;
      }else{
        consume_each(ninput_items[0]);
        return 0;
      }*/
      if(d_copy<d_count){
        int nin = std::min(d_count-d_copy,ninput_items[0]);
        int ncon = 0;
        nout = chipGen(out,in,noutput_items,nin,ncon);
        if(d_count==d_copy){
          d_append =0;
        }
        consume_each(ncon);
        return nout;
      }else if(d_append<APPENDED_CHIPS){
        if(noutput_items>=APPENDED_CHIPS){
          memcpy(out,d_append_symbols,sizeof(gr_complex)*APPENDED_CHIPS);
          d_append = APPENDED_CHIPS;
          d_count = 0;
          d_copy = 0;
          consume_each(0);
          return APPENDED_CHIPS;
        }else{
          consume_each(0);
          return 0;
        }
      }else{
        consume_each(ninput_items[0]);
        return 0;
      }

    }

  } /* namespace wifi_dsss */
} /* namespace gr */

