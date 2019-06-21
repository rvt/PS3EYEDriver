#pragma once

#include "queue.hpp"
#include "internal.hpp"

#include <mutex>
#include <condition_variable>

struct libusb_device_handle;
struct libusb_transfer;

namespace ps3eye::detail {

/* packet types when moving from iso buf to frame buf */
enum gspca_packet_type : uint8_t
{
    DISCARD_PACKET,
    FIRST_PACKET,
    INTER_PACKET,
    LAST_PACKET,
};

// urb_descriptor

struct urb_descriptor final
{
    urb_descriptor();
    ~urb_descriptor();

    bool start_transfers(libusb_device_handle* handle, uint32_t frame_size);
    void close_transfers();
    void transfer_cancelled();
    void frame_add(enum gspca_packet_type packet_type, const uint8_t* data, int len);
    void pkt_scan(uint8_t* data, int len);

    static constexpr inline unsigned num_transfers = 5;
    static constexpr inline unsigned transfer_size = 65536;

    std::mutex num_active_transfers_mutex;
    std::condition_variable num_active_transfers_condition;

    libusb_transfer* xfr[num_transfers] {};
    frame_queue queue;
    uint8_t* cur_frame_start = nullptr;

    uint32_t frame_data_len = 0;
    uint32_t frame_size = 0;
    uint32_t last_pts = 0;
    uint16_t last_fid = 0;
    gspca_packet_type last_packet_type = DISCARD_PACKET;
    uint8_t num_active_transfers = 0;
    std::array<uint8_t, transfer_size * num_transfers> transfer_buffer {};
};

} // ns ps3eye::detail
