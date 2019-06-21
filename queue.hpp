#pragma once

#include "internal.hpp"

#include <cstdint>
#include <mutex>
#include <condition_variable>
#include <vector>
#include <cstring>

namespace ps3eye {

class FrameQueue
{
public:
    FrameQueue(uint32_t frame_size)
        : frame_size(frame_size),
          frame_buffer(frame_size * BUF_FRAME_CNT)
    {
    }

    uint8_t* GetFrameBufferStart() { return frame_buffer.data(); }
    uint8_t* Enqueue();

    [[nodiscard]]
    bool Dequeue(uint8_t* new_frame, int frame_width, int frame_height, EOutputFormat outputFormat);

    static void DebayerGray(int frame_width, int frame_height, const uint8_t* inBayer, uint8_t* outBuffer);
    static void DebayerRGB(int frame_width, int frame_height, const uint8_t* inBayer, uint8_t* outBuffer, bool inBGR);

private:
    uint32_t frame_size;

    std::vector<uint8_t> frame_buffer;
    uint32_t head = 0;
    uint32_t tail = 0;
    uint32_t available = 0;

    std::mutex mutex;
    std::condition_variable empty_condition;
};

} // ns ps3eye
