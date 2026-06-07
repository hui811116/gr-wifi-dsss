/* -*- c++ -*- */
/*
 * Copyright 2026 Teng-Hui Huang.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_WIFI_DSSS_PPDU_PREFIXER_IMPL_H
#define INCLUDED_WIFI_DSSS_PPDU_PREFIXER_IMPL_H

#include <gnuradio/wifi_dsss/ppdu_prefixer.h>

namespace gr {
namespace wifi_dsss {
#define LONG_PREAMBLE_LEN 18
#define SHORT_PREAMBLE_LEN 9
#define SCRAMBLER_BYTE_RESERVED 1
#define MEM_RESERVED 8192

class ppdu_prefixer_impl : public ppdu_prefixer
{
private:
    void place_header(int psdu_len);
    void scrambler();
    const pmt::pmt_t d_in_port;
    const pmt::pmt_t d_out_port;
    bool d_long_pre;
    int d_rate;
    float d_rate_val;
    int d_ppdu_index;
    unsigned char d_buf[MEM_RESERVED];
    unsigned char d_spread_buf[MEM_RESERVED];
    unsigned char d_init_state; // for spreading code
    pmt::pmt_t d_current_pkt;
    gr::thread::mutex d_mutex;

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
    ppdu_prefixer_impl(int rate);
    ~ppdu_prefixer_impl();

    void psdu_in(pmt::pmt_t msg);
    void update_rate(int rate);
    int get_rate() const;

};

} // namespace wifi_dsss
} // namespace gr

#endif /* INCLUDED_WIFI_DSSS_PPDU_PREFIXER_IMPL_H */
