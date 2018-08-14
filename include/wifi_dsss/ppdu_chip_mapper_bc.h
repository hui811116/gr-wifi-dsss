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


#ifndef INCLUDED_WIFI_DSSS_PPDU_CHIP_MAPPER_BC_H
#define INCLUDED_WIFI_DSSS_PPDU_CHIP_MAPPER_BC_H

#include <wifi_dsss/api.h>
#include <gnuradio/block.h>

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
      typedef boost::shared_ptr<ppdu_chip_mapper_bc> sptr;

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

