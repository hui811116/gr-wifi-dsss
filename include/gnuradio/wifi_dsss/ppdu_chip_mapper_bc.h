/* -*- c++ -*- */
/*
 * Copyright 2026 Teng-Hui Huang.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_WIFI_DSSS_PPDU_CHIP_MAPPER_BC_H
#define INCLUDED_WIFI_DSSS_PPDU_CHIP_MAPPER_BC_H

#include <gnuradio/block.h>
#include <gnuradio/wifi_dsss/api.h>

namespace gr {
namespace wifi_dsss {

/*!
 * \brief <+description of block+>
 * \ingroup wifi_dsss
 *
 */
class WIFI_DSSS_API ppdu_chip_mapper_bc : virtual public gr::block
{
public:
    typedef std::shared_ptr<ppdu_chip_mapper_bc> sptr;

    /*!
     * \brief Return a shared_ptr to a new instance of wifi_dsss::ppdu_chip_mapper_bc.
     *
     * To avoid accidental use of raw pointers, wifi_dsss::ppdu_chip_mapper_bc's
     * constructor is in a private implementation
     * class. wifi_dsss::ppdu_chip_mapper_bc::make is the public interface for
     * creating new instances.
     */
    static sptr make(const std::string& lentag);
};

} // namespace wifi_dsss
} // namespace gr

#endif /* INCLUDED_WIFI_DSSS_PPDU_CHIP_MAPPER_BC_H */
