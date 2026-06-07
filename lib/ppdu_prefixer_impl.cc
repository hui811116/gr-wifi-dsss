/* -*- c++ -*- */
/*
 * Copyright 2026 Teng-Hui Huang.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include "ppdu_prefixer_impl.h"
#include <gnuradio/io_signature.h>

namespace gr {
namespace wifi_dsss {
#define d_debug 0
#define dout d_debug && std::cout

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
//static const uint8_t d_spread_mask = 0x91;
ppdu_prefixer::sptr ppdu_prefixer::make(int rate)
{
    return gnuradio::make_block_sptr<ppdu_prefixer_impl>(rate);
}


/*
 * The private constructor
 */
ppdu_prefixer_impl::ppdu_prefixer_impl(int rate)
    : gr::block("ppdu_prefixer",
                gr::io_signature::make(0, 0, 0),
                gr::io_signature::make(0, 0, 0)),
                d_in_port(pmt::mp("psdu_in")),
  			    d_out_port(pmt::mp("ppdu_out"))
{
    message_port_register_in(d_in_port);
    //set_msg_handler(d_in_port,boost::bind(&ppdu_prefixer_impl::psdu_in,this,_1));
    set_msg_handler(d_in_port,[this](pmt::pmt_t msg){this->psdu_in(msg);});
    message_port_register_out(d_out_port);
    update_rate(rate);
}

/*
 * Our virtual destructor.
 */
ppdu_prefixer_impl::~ppdu_prefixer_impl() {}


void ppdu_prefixer_impl::psdu_in(pmt::pmt_t msg)
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
    d_ppdu_index = 0;
    if(d_long_pre){
        memcpy(d_buf+d_ppdu_index,d_long_preamble,sizeof(char)*LONG_PREAMBLE_LEN);
        d_ppdu_index += LONG_PREAMBLE_LEN;
    }else{
        memcpy(d_buf+d_ppdu_index,d_short_preamble,sizeof(char)*SHORT_PREAMBLE_LEN);
        d_ppdu_index += SHORT_PREAMBLE_LEN;
    }
    place_header(io);
    memcpy(&d_buf[d_ppdu_index],uvec,sizeof(char)*io);
    d_ppdu_index+=io;
    scrambler();
    // NOTE hide a rate tag in the first byte
    d_spread_buf[0] = (unsigned char) d_rate;
    blob = pmt::make_blob(d_spread_buf,d_ppdu_index+SCRAMBLER_BYTE_RESERVED);
    d_current_pkt = pmt::cons(pmt::PMT_NIL,blob);
    message_port_pub(d_out_port,d_current_pkt);
}

void ppdu_prefixer_impl::update_rate(int rate)
{
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

int ppdu_prefixer_impl::get_rate() const
{
    return d_rate;
}

void ppdu_prefixer_impl::place_header(int psdu_len)
{
    gr::thread::scoped_lock guard(d_mutex);
    // write service and sig
    // d_sig [0]: DSSS1M
    // ...         ...
    // d_sig [3]: DSSS11M
    uint16_t u16_len;
    uint8_t* u8_len;
    switch(d_rate){
    case LONG1M:
        d_buf[d_ppdu_index++] = d_sig[0];
        d_buf[d_ppdu_index++] = 0x04;
        u16_len = psdu_len * 8;
        break;
    case LONG2M:
    case SHORT2M:
        d_buf[d_ppdu_index++] = d_sig[1];
        d_buf[d_ppdu_index++] = 0x04;
        u16_len = psdu_len * 4;
        break;
    case LONG5_5M:
    case SHORT5_5M:
        d_buf[d_ppdu_index++] = d_sig[2];
        d_buf[d_ppdu_index++] = 0x04;
        u16_len = (psdu_len * 16 + 10) / 11; // Equivalent to ceil(psdu_len*16/11.0)
        break;
    case SHORT11M:
    case LONG11M:
        d_buf[d_ppdu_index++] = d_sig[3];
        u16_len = (psdu_len * 8 + 10) / 11; // Equivalent to ceil(psdu_len*8/11.0)
        if( (11 * u16_len - 8 * psdu_len) < 8){
            d_buf[d_ppdu_index++] = 0x04;
        }else{
            d_buf[d_ppdu_index++] = 0x84;
        }
        break;
    default:
        throw std::runtime_error("Undefined rate type");
        break;
    }
    // write length
    u8_len = (uint8_t *) & u16_len;
    d_buf[d_ppdu_index++] = u8_len[0];
    d_buf[d_ppdu_index++] = u8_len[1];
    // crc
    uint16_t crc16_reg = 0xffff;
    uint32_t tmp_hdr = 0x00000000;
    for(int i=0;i<4;++i){
        tmp_hdr |= (d_buf[d_ppdu_index-1-i]<<(8*(3-i)) );
    }
    for(int i=0;i<32;++i){
        uint16_t new_bit = (tmp_hdr >> i) & 0x0001;
        uint16_t nlsb = (crc16_reg >> 15) ^ new_bit;
        crc16_reg ^= d_crc_mask[nlsb];
        crc16_reg = (crc16_reg<<1) | nlsb;
    }
    
    uint16_t crc16_inv = ~crc16_reg;
    // write crc
    u8_len = (uint8_t*) &crc16_inv;
    d_buf[d_ppdu_index++] = u8_len[0];
    d_buf[d_ppdu_index++] = u8_len[1];
}

void ppdu_prefixer_impl::scrambler()
{
    if(d_long_pre){
        d_init_state = 0x1B;
    }else{
        d_init_state = 0x6C;
    }
    uint8_t tmp_byte, tmp_bit, tmp_spd;
    for(int i=0;i<d_ppdu_index;++i){
        tmp_byte = 0x00;
        for(int j=0;j<8;++j){
            tmp_bit = (d_buf[i] >> j)&0x01;
            tmp_spd = tmp_bit ^((d_init_state >>3)&0x01 )^((d_init_state>>6)&0x01);
            tmp_byte |= (tmp_spd << j);
            d_init_state = (d_init_state<<1) | tmp_spd;
        }
        d_spread_buf[SCRAMBLER_BYTE_RESERVED+ i] = tmp_byte;
    }
}

} /* namespace wifi_dsss */
} /* namespace gr */
