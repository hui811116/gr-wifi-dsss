/* -*- c++ -*- */
/*
 * Copyright 2026 Teng-Hui Huang.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_WIFI_DSSS_CHIP_SYNC_C_H
#define INCLUDED_WIFI_DSSS_CHIP_SYNC_C_H

#include <gnuradio/block.h>
#include <gnuradio/wifi_dsss/api.h>

namespace gr {
namespace wifi_dsss {

/*!
 * \brief <+description of block+>
 * \ingroup wifi_dsss
 *
 */
class WIFI_DSSS_API chip_sync_c : virtual public gr::block
{
public:
    typedef std::shared_ptr<chip_sync_c> sptr;

    /*!
     * \brief Return a shared_ptr to a new instance of wifi_dsss::chip_sync_c.
     *
     * To avoid accidental use of raw pointers, wifi_dsss::chip_sync_c's
     * constructor is in a private implementation
     * class. wifi_dsss::chip_sync_c::make is the public interface for
     * creating new instances.
     */
    static sptr make(bool is_long_preamble, float threshold);
    virtual void set_preamble_type(bool is_long_preamble) = 0;
    virtual void set_threshold(float threshold) =0;
};

} // namespace wifi_dsss
} // namespace gr

#endif /* INCLUDED_WIFI_DSSS_CHIP_SYNC_C_H */
