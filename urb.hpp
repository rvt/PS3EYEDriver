#pragma once

#include "queue.hpp"
#include "internal.hpp"

#include <mutex>
#include <condition_variable>

#include <libusb.h>

namespace ps3eye {

/* packet types when moving from iso buf to frame buf */
enum gspca_packet_type
{
    DISCARD_PACKET,
    FIRST_PACKET,
    INTER_PACKET,
    LAST_PACKET
};

// urb_descriptor

struct urb_descriptor final
{
    urb_descriptor();

    ~urb_descriptor()
    {
        ps3eye_debug("urb_descriptor destructor\n");
        close_transfers();
    }

    bool start_transfers(libusb_device_handle* handle, uint32_t frame_size);
    void close_transfers();
    void transfer_canceled();
    void frame_add(enum gspca_packet_type packet_type, const uint8_t* data, int len);
    void pkt_scan(uint8_t* data, int len);

    std::mutex num_active_transfers_mutex;
    std::condition_variable num_active_transfers_condition;

    libusb_transfer* xfr[NUM_TRANSFERS] {};
    frame_queue queue;
    std::array<uint8_t, TRANSFER_SIZE * NUM_TRANSFERS> transfer_buffer {};
    uint8_t* cur_frame_start = nullptr;

    uint32_t frame_data_len = 0;
    uint32_t frame_size = 0;
    uint32_t last_pts = 0;
    gspca_packet_type last_packet_type = DISCARD_PACKET;
    uint16_t last_fid = 0;
    uint8_t num_active_transfers = 0;
};

} // ns ps3eye
