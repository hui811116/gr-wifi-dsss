/* -*- c++ -*- */
/*
 * Copyright 2026 Teng-Hui Huang.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_WIFI_DSSS_PPDU_CHIP_MAPPER_BC_IMPL_H
#define INCLUDED_WIFI_DSSS_PPDU_CHIP_MAPPER_BC_IMPL_H

#include <gnuradio/wifi_dsss/ppdu_chip_mapper_bc.h>

namespace gr {
namespace wifi_dsss {

class ppdu_chip_mapper_bc_impl : public ppdu_chip_mapper_bc
{
private:
    int d_count;
    int d_copy;
    int d_psdu_symbol_count;
    int d_psdu_symbol_num;
    int d_rate;
    int d_append;
    pmt::pmt_t d_rate_tag;
    bool d_preamble_type;
    float d_phase_acc;
    float d_rateVal;
    const pmt::pmt_t d_lentag;
    const pmt::pmt_t d_name;
    int chip_gen(gr_complex* out,const unsigned char* in,int noutput_items,int nin,int& nconsume);
    int cck_5_5M_chips(gr_complex* out,unsigned char byte, bool even);
    int cck_11M_chips(gr_complex* out,unsigned char byte, bool even);
    int dqpsk_2M_chips(gr_complex* out,unsigned char byte, bool even);
    int dbpsk_1M_chips(gr_complex* out,unsigned char byte, bool even);
    int (ppdu_chip_mapper_bc_impl::*d_chip_mapper)(gr_complex* out,unsigned char byte, bool even);
    int nout_check() const;
    int update_tag(int total_bytes) const;
    bool update_rate(unsigned char raw);

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
    ppdu_chip_mapper_bc_impl(const std::string& lentag);
    ~ppdu_chip_mapper_bc_impl();

    // Where all the action really happens
    void forecast(int noutput_items, gr_vector_int& ninput_items_required);

    int general_work(int noutput_items,
                     gr_vector_int& ninput_items,
                     gr_vector_const_void_star& input_items,
                     gr_vector_void_star& output_items);
};

} // namespace wifi_dsss
} // namespace gr

#endif /* INCLUDED_WIFI_DSSS_PPDU_CHIP_MAPPER_BC_IMPL_H */
