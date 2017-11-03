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
#include <wifi_dsss/ppdu_prefixer.h>
#include <gnuradio/block_detail.h>

namespace gr {
  namespace wifi_dsss {

  	class ppdu_prefixer_impl : public ppdu_prefixer
  	{
  	public:
  		ppdu_prefixer_impl(): block("ppdu_prefixer",
  			gr::io_signature::make(0,0,0),
  			gr::io_signature::make(0,0,0)),
  			d_in_port(pmt::mp("psdu_in")),
  			d_out_port(pmt::mp("ppdu_out"))
  		{
  			set_msg_handler(d_in_port,boost::bind(&ppdu_prefixer_impl::psdu_in,this,_1));
  			message_port_register_in(d_in_port);
  			message_port_register_out(d_out_port);
  		}
  		~ppdu_prefixer_impl(){}

  		void psdu_in(pmt::pmt_t msg)
  		{
  			pmt::pmt_t k = pmt::car(msg);
  			pmt::pmt_t v = pmt::cdr(msg);
  			//assert(pmt::is_blob(v));
  		}
  	private:
  		const pmt::pmt_t d_in_port;
  		const pmt::pmt_t d_out_port;

  	};

  } /* namespace wifi_dsss */
} /* namespace gr */

