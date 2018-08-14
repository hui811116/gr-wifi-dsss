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
#include <wifi_dsss/ppdu_prefixer.h>
#include <gnuradio/block_detail.h>
#include <gnuradio/math.h>
#include <cstring>

namespace gr {
  namespace wifi_dsss {
    #define d_debug 0
    #define dout d_debug && std::cout
    #define LONG_PREAMBLE_LEN 18
    #define SHORT_PREAMBLE_LEN 9
    static const unsigned char d_long_preamble[18] = {
      0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,
      0xff,0xff,0xff,0xff,0xA0,0xF3
    };
    static const unsigned char d_short_preamble[9] = {
      0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xCF,0x05
    };
    static const unsigned char d_sig[4] = {
      0x0A,0x14,0x37,0x6e
    };
    static const uint16_t d_crc_mask[2] = {0x0000, 0x0810};
    static const uint8_t d_spread_mask = 0x91;
    // 1M, 2M, 5.5M, 11M
  	class ppdu_prefixer_impl : public ppdu_prefixer
  	{
  	public:
      enum DSSSRATE{
        LONG1M,
        LONG2M,
        LONG5_5M,
        LONG11M,
        SHORT2M,
        SHORT5_5M,
        SHORT11M
      };
  		ppdu_prefixer_impl(int rate): block("ppdu_prefixer",
  			gr::io_signature::make(0,0,0),
  			gr::io_signature::make(0,0,0)),
  			d_in_port(pmt::mp("psdu_in")),
  			d_out_port(pmt::mp("ppdu_out"))
  		{
        message_port_register_in(d_in_port);
  			set_msg_handler(d_in_port,boost::bind(&ppdu_prefixer_impl::psdu_in,this,_1));
  			message_port_register_out(d_out_port);
        
  		}
  		~ppdu_prefixer_impl(){}

  		void psdu_in(pmt::pmt_t msg)
  		{
  			pmt::pmt_t k = pmt::car(msg);
  			pmt::pmt_t v = pmt::cdr(msg);
        size_t io(0); // psdu length
        const uint8_t* uvec = pmt::u8vector_elements(v,io);
        pmt::pmt_t key, blob;
        switch(d_rate){
          case LONG1M:
            key = pmt::intern("LONG1M");
          break;
          case LONG2M:
            key = pmt::intern("LONG2M");
          break;
          case LONG5_5M:
            key = pmt::intern("LONG5_5M");
          break;
          case LONG11M:
            key = pmt::intern("LONG11M");
          break;
          case SHORT2M:
            key = pmt::intern("SHORT2M");
          break;
          case SHORT5_5M:
            key = pmt::intern("SHORT5_5M");
          break;
          case SHORT11M:
            key = pmt::intern("SHORT11M");
          break;
          default:
            throw std::runtime_error("Undefined rate type");
          break;
        }
        d_ppdu_index=1; // first byte reserved for ratetag
        placePreambleSfd();
        placeHeader(io);
        memcpy(&d_buf[d_ppdu_index],uvec,sizeof(char)*io);
        d_ppdu_index+=io;
        scrambler();
        // NOTE hide a rate tag in the first byte
        d_spread_buf[0] = (unsigned char) d_rate;
        blob = pmt::make_blob(d_spread_buf,d_ppdu_index+1);
        d_current_pkt = pmt::cons(pmt::PMT_NIL,blob);
        message_port_pub(d_out_port,d_current_pkt);
  		}
      void update_rate(int rate)
      {
        gr::thread::scoped_lock guard(d_mutex);
        switch(rate){
          case LONG1M:
            d_long_pre = true;
            d_rate_val = 1;
          break;
          case LONG2M:
            d_long_pre = true;
            d_rate_val = 2;
          break;
          case LONG5_5M:
            d_long_pre = true;
            d_rate_val = 5.5;
          break;
          case LONG11M:
            d_long_pre = true;
            d_rate_val = 11;
          break;
          case SHORT2M:
            d_long_pre = false;
            d_rate_val = 2;
          break;
          case SHORT5_5M:
            d_long_pre = false;
            d_rate_val = 5.5;
          break;
          case SHORT11M:
            d_long_pre = false;
            d_rate_val = 11;
          break;
          default:
            // if undefined, use 1MBPS rate tag
            d_long_pre = true;
            d_rate_val = 1;
          break;
        }
        d_rate = rate;
      }
      int get_rate() const
      {
        return d_rate;
      }
  	private:
      void placePreambleSfd()
      {
        if(d_long_pre){
          memcpy(d_buf+d_ppdu_index,d_long_preamble,sizeof(char)*LONG_PREAMBLE_LEN);
          d_ppdu_index += LONG_PREAMBLE_LEN;
        }else{
          memcpy(d_buf+d_ppdu_index,d_short_preamble,sizeof(char)*SHORT_PREAMBLE_LEN);
          d_ppdu_index += SHORT_PREAMBLE_LEN;
        }
      }
      void placeHeader(int psduLen)
      {
        //unsigned char service = 0x00;
        // sig
        // d_sig [0]: DSSS1M
        // ...         ...
        // d_sig [3]: DSSS11M
        float tmpLen;
        uint16_t u16Len;
        uint8_t* u8Len;
        switch(d_rate){
          case LONG1M:
            d_buf[d_ppdu_index++] = d_sig[0];
            d_buf[d_ppdu_index++] = 0x04;
            tmpLen = psduLen * 8;
          break;
          case LONG2M:
          case SHORT2M:
            d_buf[d_ppdu_index++] = d_sig[1];
            d_buf[d_ppdu_index++] = 0x04;
            tmpLen = psduLen*4;
          break;
          case LONG5_5M:
          case SHORT5_5M:
            d_buf[d_ppdu_index++] = d_sig[2];
            d_buf[d_ppdu_index++] = 0x04;
            tmpLen = psduLen*8/(float)5.5;
          break;
          case SHORT11M:
          case LONG11M:
            d_buf[d_ppdu_index++] = d_sig[3];
            tmpLen = 8*psduLen/(float)11.0;
            if(11*(ceil(tmpLen)-tmpLen) >= 8 ){
              // extend length field
              d_buf[d_ppdu_index++] = 0x84;
            }else{
              d_buf[d_ppdu_index++] = 0x04;
            }
          break;
          default:
            throw std::runtime_error("Undefined rate type");
          break;
        }
        // service
        // length
        u16Len = (uint16_t) ceil(tmpLen);
        u8Len = (uint8_t *) & u16Len;
        d_buf[d_ppdu_index++] = u8Len[0];
        d_buf[d_ppdu_index++] = u8Len[1];
        // crc
        uint16_t crc16_reg = 0xffff;
        uint32_t tmphdr = 0x00000000;
        for(int i=0;i<4;++i){
          tmphdr |= (d_buf[d_ppdu_index-1-i]<<(8*(3-i)) );
        }
        for(int i=0;i<32;++i){
          uint16_t newBit = (tmphdr >> i) & 0x0001;
          uint16_t nlsb = (crc16_reg >> 15) ^ newBit;
          crc16_reg ^= d_crc_mask[nlsb];
          crc16_reg = (crc16_reg<<1) | nlsb;
        }
        
        uint16_t crc16_inv = ~crc16_reg;
        u8Len = (uint8_t*) &crc16_inv;
        d_buf[d_ppdu_index++] = u8Len[0];
        d_buf[d_ppdu_index++] = u8Len[1];
      }
      void scrambler()
      {
        if(d_long_pre){
          d_init_state = 0x1B;
        }else{
          d_init_state = 0x6C;
        }
        int bcnt = 0;
        uint8_t tmp_byte, tmp_bit, tmp_spd;
        for(int i=0;i<d_ppdu_index;++i){
          tmp_byte = 0x00;
          for(int j=0;j<8;++j){
            tmp_bit = (d_buf[i] >> j)&0x01;
            tmp_spd = tmp_bit ^((d_init_state >>3)&0x01 )^((d_init_state>>6)&0x01);
            tmp_byte |= (tmp_spd << j);
            d_init_state = (d_init_state<<1) | tmp_spd;
          }
          d_spread_buf[i] = tmp_byte;
        }
      }
  		const pmt::pmt_t d_in_port;
  		const pmt::pmt_t d_out_port;
      bool d_long_pre;
      int d_rate;
      float d_rate_val;
      int d_ppdu_index;
      unsigned char d_buf[2048];
      unsigned char d_spread_buf[2048];
      unsigned char d_init_state; // for spreading code
      pmt::pmt_t d_current_pkt;
      gr::thread::mutex d_mutex;
  	};
    ppdu_prefixer::sptr 
    ppdu_prefixer::make(int rate)
    {
      return gnuradio::get_initial_sptr(new ppdu_prefixer_impl(rate));
    }
  } /* namespace wifi_dsss */
} /* namespace gr */

