/* -*- c++ -*- */
/*
 * Author: Diego Badillo-San-Juan <diego.badillo-san-juan@inria.fr>
 * Copyright 2025 Inria.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */


#ifndef INCLUDED_SIC_TAG_IQ_STREAM_IMPL_H
#define INCLUDED_SIC_TAG_IQ_STREAM_IMPL_H

#include <gnuradio/sic/tag_iq_stream.h>

namespace gr {
namespace sic {

class tag_iq_stream_impl : public tag_iq_stream
{
public:
    tag_iq_stream_impl(uint sps);
    ~tag_iq_stream_impl();

    // Where all the action really happens
    void forecast(int noutput_items, gr_vector_int& ninput_items_required);

    int general_work(int noutput_items,
                     gr_vector_int& ninput_items,
                     gr_vector_const_void_star& input_items,
                     gr_vector_void_star& output_items);

private:
    uint d_sps; // Samples per symbol
};

} // namespace sic
} // namespace gr

#endif /* INCLUDED_SIC_TAG_IQ_STREAM_IMPL_H */
