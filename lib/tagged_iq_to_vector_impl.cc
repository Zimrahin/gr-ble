/* -*- c++ -*- */
/*
 * Author: Diego Badillo-San-Juan <diego.badillo-san-juan@inria.fr>
 * Copyright 2025 Inria.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include "tagged_iq_to_vector_impl.h"
#include <gnuradio/io_signature.h>
#include <pmt/pmt.h>
#include <volk/volk.h>
#include <volk/volk_alloc.hh>
#include <deque>
#include <map>

namespace gr {
namespace ble {

using input_type = gr_complex;
tagged_iq_to_vector::sptr tagged_iq_to_vector::make(uint64_t pre_samples,
                                                    uint64_t post_samples,
                                                    uint64_t buffer_size)
{
    return gnuradio::make_block_sptr<tagged_iq_to_vector_impl>(
        pre_samples, post_samples, buffer_size);
}


// Constructor
tagged_iq_to_vector_impl::tagged_iq_to_vector_impl(uint64_t pre_samples,
                                                   uint64_t post_samples,
                                                   uint64_t buffer_size)
    : gr::block("tagged_iq_to_vector",
                gr::io_signature::make(1, 1, sizeof(input_type)),
                gr::io_signature::make(0, 0, 0)),
      d_pre_samples(pre_samples),
      d_post_samples(post_samples),
      d_buffer_size(buffer_size)
{
    message_port_register_out(pmt::mp("pmt_out"));
    d_buffer = volk::vector<gr_complex>(d_buffer_size);
    d_buffer_start = 0;
    d_buffer_write_index = 0;
    d_packet_id = 0;
}

// Destructor
tagged_iq_to_vector_impl::~tagged_iq_to_vector_impl() {}

void tagged_iq_to_vector_impl::forecast(int noutput_items,
                                        gr_vector_int& ninput_items_required)
{
    ninput_items_required[0] = std::min(1024, static_cast<int>(d_buffer_size));
}

// Need to check this method still
void tagged_iq_to_vector_impl::extract_iq_packet(uint64_t packet_id,
                                                 uint64_t start,
                                                 uint64_t end)
{
    if (start < d_buffer_start || end > d_buffer_start + d_buffer_size) {
        return;
    }

    const size_t start_idx = (start - d_buffer_start) % d_buffer_size;
    const size_t end_idx = (end - d_buffer_start) % d_buffer_size;
    const size_t packet_len = end - start;

    // Extract packet samples
    volk::vector<gr_complex> packet(packet_len);

    if (start_idx <= end_idx) {
        // Contiguous segment
        std::copy(
            d_buffer.begin() + start_idx, d_buffer.begin() + end_idx, packet.begin());
    } else {
        // Wrapped segment
        const size_t first_part = d_buffer_size - start_idx;
        std::copy(d_buffer.begin() + start_idx, d_buffer.end(), packet.begin());
        std::copy(d_buffer.begin(),
                  d_buffer.begin() + (packet_len - first_part),
                  packet.begin() + first_part);
    }

    // Create comprehensive metadata dictionary
    pmt::pmt_t meta = pmt::make_dict();
    meta = pmt::dict_add(meta, pmt::mp("packet_id"), pmt::from_uint64(packet_id));
    meta = pmt::dict_add(meta, pmt::mp("start_offset"), pmt::from_uint64(start));
    meta = pmt::dict_add(meta, pmt::mp("end_offset"), pmt::from_uint64(end));
    meta = pmt::dict_add(meta, pmt::mp("pre_samples"), pmt::from_long(d_pre_samples));
    meta = pmt::dict_add(meta, pmt::mp("post_samples"), pmt::from_long(d_post_samples));

    // Create complex vector
    pmt::pmt_t vec = pmt::init_c32vector(packet.size(), packet.data());

    // Combine metadata and vector
    pmt::pmt_t pdu = pmt::cons(meta, vec);

    // Send single message with all information
    message_port_pub(pmt::mp("packets"), pdu);
}

int tagged_iq_to_vector_impl::general_work(int noutput_items,
                                           gr_vector_int& ninput_items,
                                           gr_vector_const_void_star& input_items,
                                           gr_vector_void_star& output_items)
{
    auto in = static_cast<const input_type*>(input_items[0]);
    int n_store = std::min(ninput_items[0], static_cast<int>(d_buffer_size));

    // Store samples in a ring buffer
    for (int i = 0; i < n_store; i++) {
        d_buffer[d_buffer_write_index] = in[i];
        d_buffer_write_index = (d_buffer_write_index + 1) % d_buffer_size;
    }

    // Update buffer start position if d_buffer_size >= ninput_items[0]
    if (n_store == ninput_items[0] && n_store > 0) {
        d_buffer_start = nitems_read(0) + n_store - d_buffer_size;
    }

    // Create a PMT vector to hold the IQ samples from tags
    std::vector<tag_t> tags_in;
    get_tags_in_range(tags_in, 0, nitems_read(0), nitems_read(0) + ninput_items[0]);

    for (const auto& tag : tags_in) {
        uint64_t tag_packet_id =
            pmt::to_uint64(pmt::tuple_ref(tag.value, 0)); // Get packet count
        if (pmt::eq(tag.key, pmt::mp("Payload start"))) {
            d_packet_id = tag_packet_id;
            d_payload_starts[d_packet_id] = tag.offset;

        } else if (pmt::eq(tag.key, pmt::mp("CRC check"))) {
            // Check if we have a start tag for this packet
            if (d_packet_id != tag_packet_id || !d_payload_starts.count(d_packet_id)) {
                continue;
            }
            const bool crc_ok = pmt::to_bool(pmt::tuple_ref(tag.value, 1));

            const uint64_t start_offset = d_payload_starts[d_packet_id];
            const uint64_t end_offset = tag.offset;

            const uint64_t extract_start = (start_offset > d_pre_samples)
                                               ? start_offset - d_pre_samples
                                               : 0; // In case the first payload arrives
                                                    // at the very beginning of the stream
            const uint64_t extract_end = end_offset + d_post_samples;

            if (crc_ok) {
                extract_iq_packet(d_packet_id, extract_start, extract_end);
            }

            d_payload_starts.erase(d_packet_id);
        }
    }


    consume(0, ninput_items[0]);

    return noutput_items; // Tell runtime system how many output items we produced.
}

} /* namespace ble */
} /* namespace gr */
