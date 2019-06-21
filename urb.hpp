#pragma once

#include "queue.hpp"
#include "internal.hpp"

#include <memory>
#include <mutex>
#include <condition_variable>

#include "libusb.h"

namespace ps3eye {

/* packet types when moving from iso buf to frame buf */
enum gspca_packet_type
{
    DISCARD_PACKET,
    FIRST_PACKET,
    INTER_PACKET,
    LAST_PACKET
};

// URBDesc

struct URBDesc
{
    URBDesc();

    ~URBDesc()
    {
        ps3eye_debug("URBDesc destructor\n");
        close_transfers();
    }

    bool start_transfers(libusb_device_handle* handle, uint32_t curr_frame_size);
    void close_transfers();
    void transfer_canceled();
    void frame_add(enum gspca_packet_type packet_type, const uint8_t* data, int len);
    void pkt_scan(uint8_t* data, int len);

    uint8_t num_active_transfers = 0;
    std::mutex num_active_transfers_mutex;
    std::condition_variable num_active_transfers_condition;

    enum gspca_packet_type last_packet_type = DISCARD_PACKET;
    uint32_t last_pts = 0;
    uint16_t last_fid = 0;
    libusb_transfer* xfr[NUM_TRANSFERS];

    uint8_t* transfer_buffer = nullptr;
    uint8_t* cur_frame_start = nullptr;
    uint32_t cur_frame_data_len = 0;
    uint32_t frame_size = 0;
    FrameQueue* frame_queue = nullptr;
};

} // ns ps3eye
