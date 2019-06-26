#pragma once

#ifdef PS3EYE_DEBUG
#   include <cstdio>
#endif

namespace ps3eye::detail {

extern volatile bool _ps3eye_debug_status;

#ifdef PS3EYE_DEBUG
template<unsigned N, typename... xs>
inline void ps3eye_debug(const char(&fmt)[N], const xs&... args)
{
    if (_ps3eye_debug_status)
        fprintf(stderr, fmt, args...);
}
#else
template<unsigned N, typename... xs>
inline void ps3eye_debug(const char(&)[N], const xs&... args)
{
}
#endif

} // ns ps3eye::detail

namespace ps3eye {
enum class format
{
    Bayer, // Output in Bayer. Destination buffer must be width * height bytes
    BGR, // Output in BGR. Destination buffer must be width * height * 3 bytes
    RGB, // Output in RGB. Destination buffer must be width * height * 3 bytes
    Gray // Output in Grayscale. Destination buffer must be width * height bytes
};
} // ns ps3eye
