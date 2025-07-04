/* -*- c++ -*- */
/*
 * Author: Diego Badillo-San-Juan <diego.badillo-san-juan@inria.fr>
 * Copyright 2025 Inria.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

/*
 * References:
 * - bastibl/gr-ieee802-15-4 (packet_sink.cc)
 * - GNU Radio 802.15.4 En- and Decoding
 * - CMOS RFIC Architectures for IEEE 802.15.4 Networks
 */

#ifndef INCLUDED_SIC_IEEE802154_PACKET_SINK_IMPL_H
#define INCLUDED_SIC_IEEE802154_PACKET_SINK_IMPL_H

#include <gnuradio/sic/ieee802154_packet_sink.h>

namespace gr {
namespace sic {

class ieee802154_packet_sink_impl : public ieee802154_packet_sink
{
public:
    ieee802154_packet_sink_impl(uint preamble_threshold,
                                bool crc_included,
                                uint block_id);
    ~ieee802154_packet_sink_impl();

    // Where all the action really happens
    int work(int noutput_items,
             gr_vector_const_void_star& input_items,
             gr_vector_void_star& output_items);

private:
    // Chip mapping for differential MSK decoding
    static constexpr std::array<uint32_t, 16> d_chip_mapping_msk = { {
        0xE077AE6C, // 0
        0xCE077AE6, // 1
        0x6CE077AE, // 2
        0xE6CE077A, // 3
        0xAE6CE077, // 4
        0x7AE6CE07, // 5
        0x77AE6CE0, // 6
        0x877AE6CE, // 7
        0x1F885193, // 8
        0x31F88519, // 9
        0x931F8851, // A
        0x1931F885, // B
        0x51931F88, // C
        0x851931F8, // D
        0x8851931F, // E
        0x78851931  // F
    } };

    // 5-byte-long synchronisation header: four 0x00 octets + SFD (0xA7)
    static constexpr std::array<uint8_t, 10> d_preamble_sequence = {
        { 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x7, 0xA }
    };

    // Finite State Machine
    enum class state { SEARCH_PREAMBLE, DECODE_LENGTH, DECODE_PAYLOAD, CHECK_CRC };
    state d_state;
    void enter_search_preamble();
    void enter_decode_length();
    void enter_decode_payload();
    void enter_check_crc();
    void process_search_preamble(uint8_t chip, uint64_t sample_index);
    void process_decode_length(uint8_t chip, uint64_t sample_index);
    void process_decode_payload(uint8_t chip, uint64_t sample_index);
    void process_check_crc(uint8_t chip, uint64_t sample_index);

    // Helper functions
    template <typename T>
    T reverse_bits(T unsigned_integer); // Reverse bits in an unsigned integer
    uint8_t slice(float data_in);       // Slice float data into binary data
    bool nibble_match(uint32_t chip_sequence,
                      uint8_t nibble,
                      uint threshold); // Checks whether a given chip sequence matches the
                                       // predefined constant channel mapping at index
                                       // nibble, with a tolerance of threshold errors
    uint8_t pack_chips_to_nibble(uint32_t chip_sequence,
                                 uint threshold); // Pack 32 chips into a nibble
    void compute_crc(uint8_t data_bit,
                     uint32_t& crc,
                     uint32_t polynomial,
                     uint32_t mask); // Compute CRC from a bit
    void compute_crc_byte(uint8_t data_byte,
                          uint32_t& crc,
                          uint32_t polynomial,
                          uint32_t mask); // Compute CRC from a byte
    void output_pdu(uint64_t sample_index,
                    bool crc_ok); // Output PDU with metadata and payload

    // Constants
    static const uint8_t d_max_payload_len =
        127;                         // Maximum payload length in bytes (including CRC)
    static const uint d_crc_len = 2; // CRC length in bytes
    static const uint8_t d_chip_sequence_len = 32; // 32-chip sequences for IEEE 802.15.4
    uint d_threshold;          // Allowed chip errors in the preamble detection
    bool d_crc_included;       // Is CRC included in the payload?
    uint d_block_id;           // Block instance ID
    uint32_t d_chip_mask;      // 32-bit mask for chip-sequence comparison
    uint32_t d_crc_mask;       // Mask for CRC computation
    uint32_t d_crc_polynomial; // Polynomial for CRC computation
    uint32_t d_crc_init;       // Initial value for CRC computation

    // Variables
    bool d_output_connected;  // Indicates if the stream output port is connected
    uint32_t d_shift_reg;     // 32-bit shift register for storing chips
    uint d_fill_buffer_count; // Ensures the shift register is filled before comparing
    uint d_nibble_count;      // Nibbles (32-chip sequences) we have found
                              // in the preamble
    uint8_t d_chip_count;     // Chips we have collected within a 32-chip sequence
    uint64_t d_packet_count;  // Packet count for the current packet
    uint8_t d_payload_len;    // Length of the payload
    std::array<uint8_t, d_max_payload_len> d_payload; // Payload buffer
    uint8_t d_reg_byte;                               // Shift register for byte decoding
    bool d_entering_payload;         // Indicates if we are entering the payload state
    uint64_t d_sample_payload_index; // First sample index of the payload
    uint8_t d_bytes_count;           // Number of bytes collected for the current packet
    uint32_t d_crc_computed;         // Buffer to store computed CRC
    uint32_t d_crc_received;         // Buffer to store received CRC
};

} // namespace sic
} // namespace gr

#endif /* INCLUDED_SIC_IEEE802154_PACKET_SINK_IMPL_H */
