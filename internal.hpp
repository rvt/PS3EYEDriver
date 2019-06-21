#pragma once

#ifdef PS3EYE_DEBUG
#   include <cstdio>
#endif

namespace ps3eye::detail {

#ifdef PS3EYE_DEBUG
template<typename... xs> inline void ps3eye_debug(const xs&... args) { fprintf(stdout, args...); }
#else
template<typename... xs> inline void ps3eye_debug(const xs&...) {}
#endif

enum class format
{
    Bayer, // Output in Bayer. Destination buffer must be width * height bytes
    BGR, // Output in BGR. Destination buffer must be width * height * 3 bytes
    RGB, // Output in RGB. Destination buffer must be width * height * 3 bytes
    Gray // Output in Grayscale. Destination buffer must be width * height bytes
};

} // ns ps3eye::detail
