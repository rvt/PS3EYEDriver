#pragma once

#ifdef PS3EYE_DEBUG
#   include <cstdio>
#endif

namespace ps3eye {

#ifdef PS3EYE_DEBUG
template<typename... xs> inline void ps3eye_debug(const xs&... args) { fprintf(stdout, args...); }
#else
template<typename... xs> inline void ps3eye_debug(const xs&...) {}
#endif

enum class EOutputFormat
{
    Bayer, // Output in Bayer. Destination buffer must be width * height bytes
    BGR, // Output in BGR. Destination buffer must be width * height * 3 bytes
    RGB, // Output in RGB. Destination buffer must be width * height * 3 bytes
    Gray // Output in Grayscale. Destination buffer must be width * height bytes
};

enum {
    TRANSFER_SIZE = 65536,
    NUM_TRANSFERS = 5,
    BUF_FRAME_CNT = 4,
    VENDOR_ID = 0x1415,
    PRODUCT_ID = 0x2000,
};

/* Values for bmHeaderInfo = (Video and Still Image Payload Headers, 2.4.3.3) */
enum {
    UVC_STREAM_EOH = (1 << 7),
    UVC_STREAM_ERR = (1 << 6),
    UVC_STREAM_STI = (1 << 5),
    UVC_STREAM_RES = (1 << 4),
    UVC_STREAM_SCR = (1 << 3),
    UVC_STREAM_PTS = (1 << 2),
    UVC_STREAM_EOF = (1 << 1),
    UVC_STREAM_FID = (1 << 0),
};

} // ns ps3eye
