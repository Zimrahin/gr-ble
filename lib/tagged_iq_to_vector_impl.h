/* -*- c++ -*- */
/*
 * Author: Diego Badillo-San-Juan <diego.badillo-san-juan@inria.fr>
 * Copyright 2025 Inria.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_BLE_TAGGED_IQ_TO_VECTOR_IMPL_H
#define INCLUDED_BLE_TAGGED_IQ_TO_VECTOR_IMPL_H

#include <gnuradio/ble/tagged_iq_to_vector.h>
#include <volk/volk.h>

namespace gr {
namespace ble {

class tagged_iq_to_vector_impl : public tagged_iq_to_vector
{
public:
    tagged_iq_to_vector_impl(uint64_t pre_samples,
                             uint64_t post_samples,
                             uint64_t buffer_size);
    ~tagged_iq_to_vector_impl();

    // Where all the action really happens
    void forecast(int noutput_items, gr_vector_int& ninput_items_required);

    int general_work(int noutput_items,
                     gr_vector_int& ninput_items,
                     gr_vector_const_void_star& input_items,
                     gr_vector_void_star& output_items);

private:
    // Helper functions
    void extract_iq_packet(uint64_t packet_id, uint64_t start, uint64_t end);

    // Constants
    uint64_t d_pre_samples;         // Offset samples before the start packet tag
    uint64_t d_post_samples;        // Offset samples after the end packet tag
    uint64_t d_buffer_size;         // Maximum size of the buffer for storing IQ samples
    uint64_t d_current_buffer_size; // Current size of the buffer

    // Variables
    volk::vector<gr_complex> d_buffer; // Circular buffer for storing IQ samples
    uint64_t d_buffer_start;           // Absolute offset of first sample in buffer
    uint64_t d_buffer_write_index;     // Current write position in buffer
    std::map<uint64_t, uint64_t>
        d_payload_starts; // Key, value pair to temporary store where the packet starts
    uint64_t d_packet_id; // Current packet being processed
};

} // namespace ble
} // namespace gr

#endif /* INCLUDED_BLE_TAGGED_IQ_TO_VECTOR_IMPL_H */
