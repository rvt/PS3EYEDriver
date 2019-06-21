#pragma once

#include <algorithm>
#include <limits>

namespace ps3eye::detail {

template<typename t, t min_ = std::numeric_limits<t>::min(), t max_ = std::numeric_limits<t>::max()>
struct val_
{
    static constexpr int min = min_;
    static constexpr int max = max_;

    constexpr val_& operator=(long x)
    {
        value_ = (t)std::clamp(x, (long)min, (long)max);
        return *this;
    }

    constexpr val_(t x) : value_(x) {}

    val_(const val_&) = delete;
    val_& operator=(const val_&) = delete;

    constexpr operator t() const { return value_; }
    constexpr const t& operator*() const { return value_; }

private:
    t value_;
};

} // ns ps3eye::detail
