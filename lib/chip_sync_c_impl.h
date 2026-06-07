/* -*- c++ -*- */
/*
 * Copyright 2026 Teng-Hui Huang.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_WIFI_DSSS_CHIP_SYNC_C_IMPL_H
#define INCLUDED_WIFI_DSSS_CHIP_SYNC_C_IMPL_H

#include <gnuradio/wifi_dsss/chip_sync_c.h>
#include <atomic>


namespace gr {
namespace wifi_dsss {
#define MAX_BUFFER_BYTES 8192
static const gr_complex d_cck5_5_chips[4][8] = {
    {gr_complex(0.0,1.0),gr_complex(1.0,0.0),gr_complex(0.0,1.0),gr_complex(-1.0,0.0),gr_complex(0.0,1.0),gr_complex(1.0,0.0),gr_complex(-0.0,-1.0),gr_complex(1.0,0.0)},
    {gr_complex(0.0,1.0),gr_complex(-1.0,0.0),gr_complex(0.0,1.0),gr_complex(1.0,0.0),gr_complex(-0.0,-1.0),gr_complex(1.0,0.0),gr_complex(0.0,1.0),gr_complex(1.0,0.0)},
    {gr_complex(-0.0,-1.0),gr_complex(-1.0,0.0),gr_complex(-0.0,-1.0),gr_complex(1.0,0.0),gr_complex(0.0,1.0),gr_complex(1.0,0.0),gr_complex(-0.0,-1.0),gr_complex(1.0,0.0)},
    {gr_complex(-0.0,-1.0),gr_complex(1.0,0.0),gr_complex(-0.0,-1.0),gr_complex(-1.0,0.0),gr_complex(-0.0,-1.0),gr_complex(1.0,0.0),gr_complex(0.0,1.0),gr_complex(1.0,0.0)}
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
    
private:
    uint8_t descrambler(const uint8_t& rawbit);
    void enter_search();
    void enter_sync();
    void enter_psdu();
    bool check_hdr();
    //
    uint16_t (chip_sync_c_impl::* d_hdr_get_bits)(const gr_complex* in, bool is_even);
    uint16_t (chip_sync_c_impl::* d_get_symbol_fptr)(const gr_complex* in,bool is_even);
    uint16_t get_symbol_dbpsk(const gr_complex* in,bool is_even);
    uint16_t get_symbol_dqpsk(const gr_complex* in,bool is_even);
    uint16_t get_symbol_cck5_5(const gr_complex* in,bool is_even);
    uint16_t get_symbol_cck11(const gr_complex* in,bool is_even);
    //
    void psdu_write_bits(const uint16_t& out_byte);

    gr_complex pll_bpsk(const gr_complex& in);
    gr_complex pll_qpsk(const gr_complex& in);
    void reset_pll();
    
    gr::thread::mutex d_mutex;
    
    //bool d_change_type;
    //atomic_bool d_change_type;
    bool d_chip_sync;
    bool d_current_preamble_is_long;
    //bool d_preamble_type;
    std::atomic<bool> d_preamble_is_long;
    
    //float d_threshold;
    std::atomic<float> d_threshold;
    float d_rate_val;
    float d_phase;
    float d_freq;
    float d_alpha;
    float d_beta;
    float d_crit;
    float d_loopbw;
    
    unsigned int d_hdr_reg;
    
    uint8_t d_sig_dec;
    uint8_t d_service_dec;
    //uint8_t d_byte_reg;
    uint8_t d_des_state;
    
    uint16_t d_hdr_crc;
    uint16_t d_length_dec;
    uint16_t d_sync_reg;
    uint16_t d_sync_word;
    
    int d_rx_state;
    int d_hdr_bps;
    int d_chip_wait;
    //int d_bit_len;
    int d_psdu_bytes_len;
    int d_psdu_bit_cnt;
    int d_bit_cnt;
    int d_chip_cnt;
    int d_psdu_chip_size;
    int d_psdu_type;
    int d_psdu_sym_cnt;
    
    gr_complex d_prev_sym;
    
    unsigned char d_buf[MAX_BUFFER_BYTES];
    const pmt::pmt_t d_psdu_out;
    gr_complex* d_chip_buf;
    
public:
    enum RXSTATE
    {
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
    chip_sync_c_impl(bool is_long_preamble, float threshold);
    ~chip_sync_c_impl();

    // Where all the action really happens
    void set_preamble_type(bool is_long_preamble);
    void set_threshold(float threshold);
    void forecast(int noutput_items, gr_vector_int& ninput_items_required);

    int general_work(int noutput_items,
                     gr_vector_int& ninput_items,
                     gr_vector_const_void_star& input_items,
                     gr_vector_void_star& output_items);
};

} // namespace wifi_dsss
} // namespace gr

#endif /* INCLUDED_WIFI_DSSS_CHIP_SYNC_C_IMPL_H */
