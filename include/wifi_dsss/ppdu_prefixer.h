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


#ifndef INCLUDED_WIFI_DSSS_PPDU_PREFIXER_H
#define INCLUDED_WIFI_DSSS_PPDU_PREFIXER_H

#include <wifi_dsss/api.h>
#include <gnuradio/block.h>

namespace gr {
  namespace wifi_dsss {

    /*!
     * \brief <+description+>
     *
     */
    class WIFI_DSSS_API ppdu_prefixer : virtual public block
    {
    public:
      typedef boost::shared_ptr<ppdu_prefixer> sptr;
      static sptr make(int rate);
      virtual void update_rate(int rate)=0;
    };

  } // namespace wifi_dsss
} // namespace gr

#endif /* INCLUDED_WIFI_DSSS_PPDU_PREFIXER_H */

