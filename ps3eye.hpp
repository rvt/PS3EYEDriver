// source code from https://github.com/inspirit/PS3EYEDriver
#pragma once

#include "urb.hpp"
#include "internal.hpp"
#include "setter.hpp"

#include <vector>
#include <array>
#include <cstdint>
#include <utility>

struct libusb_device;
struct libusb_device_handle;

namespace ps3eye::detail {

enum class resolution : uint8_t {
    QVGA,
    VGA,
};

struct rate_s
{
    int fps;
    uint8_t r11;
    uint8_t r0d;
    uint8_t re5;
};

struct camera
{
    explicit camera(libusb_device* device);
    ~camera();

    [[nodiscard]] bool init(resolution res, int framerate = 60, format fmt = format::BGR);
    [[nodiscard]] bool start();
    void stop();

    // Controls
    constexpr bool auto_gain() const { return auto_gain_; }
    void set_auto_gain(bool val);
    constexpr bool awb() const { return awb_; }
    void set_awb(bool val);
    constexpr uint8_t gain() const { return gain_; }
    void set_gain(int val);
    constexpr uint8_t exposure() const { return exposure_; }
    void set_exposure(int val);
    constexpr uint8_t sharpness() const { return sharpness_; }
    void set_sharpness(int val);
    constexpr uint8_t contrast() const { return contrast_; }
    void set_contrast(int val);
    constexpr uint8_t brightness() const { return brightness_; }
    void set_brightness(int val);
    constexpr uint8_t hue() const { return hue_; }
    void set_hue(int val);
    constexpr uint8_t red_balance() const { return redblc_; }
    void set_red_balance(int val);
    constexpr uint8_t blue_balance() const { return blueblc_; }
    void set_blue_balance(int val);
    constexpr uint8_t green_balance() const { return greenblc_; }
    void set_green_balance(int val);
    constexpr std::pair<bool, bool> flip_status() { return { flip_h_, flip_v_ }; }
    void set_flip_status(bool horizontal = false, bool vertical = false);
    constexpr bool test_pattern_status() const { return test_pattern_; }
    void set_test_pattern_status(bool enable);
    constexpr int framerate() const { return frame_rate_; }
    bool set_framerate(int val);

    constexpr bool is_open() const { return streaming_; }
    constexpr bool is_initialized() const { return device_ && handle_ && size() != std::pair<int,int>(); }

    constexpr libusb_device* device() const { return device_; }
    [[nodiscard]] bool usb_port(char* buf, unsigned sz) const;

    // Get a frame from the camera. Notes:
    // - If there is no frame available, this function will block until one is
    // - The output buffer must be sized correctly, depending out the output
    // format. See format.
    [[nodiscard]] bool get_frame(uint8_t* frame);

    inline int width() const { return size().first; }
    inline int height() const { return size().second; }
    std::pair<int, int> size() const;

    inline int stride() const { return width() * bytes_per_pixel(); }
    int bytes_per_pixel() const;

    camera(const camera&) = delete;
    void operator=(const camera&) = delete;

    static void set_debug(bool value);
    static bool is_debugging() { return _ps3eye_debug; }

    static int normalize_framerate(int fps, resolution res);
    int normalize_framerate(int fps);

private:
    static rate_s _normalize_framerate(int fps, resolution res);

    void release();
    [[nodiscard]] bool open_usb();
    void close_usb();

    // usb ops
    int ov534_set_frame_rate(int frame_rate, bool dry_run = false);
    void ov534_set_led(int status);
    void ov534_reg_write(uint16_t reg, uint8_t val);
    uint8_t ov534_reg_read(uint16_t reg);
    int sccb_check_status();
    void sccb_reg_write(uint8_t reg, uint8_t val);
    uint8_t sccb_reg_read(uint16_t reg);
    void reg_w_array(const uint8_t (*data)[2], int len);
    void sccb_w_array(const uint8_t (*data)[2], int len);

    template<uint8_t max = 255> using val = val_<uint8_t, 0, max>;

    // controls
    val<63> gain_ = 20; // 0 <-> 63
    val<63> sharpness_ = 0; // 0 <-> 63
    val<> exposure_ = 255;  // 0 <-> 255
    val<> hue_ = 143;       // 0 <-> 255
    val<> brightness_ = 20; // 0 <-> 255
    val<> contrast_ = 37;   // 0 <-> 255
    val<> blueblc_ = 128;    // 0 <-> 255
    val<> redblc_ = 128;     // 0 <-> 255
    val<> greenblc_ = 128;   // 0 <-> 255

    bool auto_gain_ = false;
    bool awb_ = false;
    bool flip_h_ = false;
    bool flip_v_ = false;
    bool test_pattern_ = false;
    bool streaming_ = false;

    //static bool enumerated;
    //static std::vector<std::shared_ptr<camera>> devices;

    resolution resolution_;
    int frame_rate_ = 0;
    format format_ = format::BGR;

    // usb stuff
    libusb_device* device_ = nullptr;
    libusb_device_handle* handle_ = nullptr;
    urb_descriptor urb;
    std::array<uint8_t, 64> usb_buf;
};

std::vector<std::shared_ptr<camera>> list_devices();

} // namespace ps3eye::detail

namespace ps3eye
{
    using ps3eye::detail::list_devices;

    using ps3eye::detail::camera;
    using ps3eye::detail::resolution;

    static constexpr inline auto res_VGA = resolution::VGA;
    static constexpr inline auto res_QVGA = resolution::QVGA;
} // ns ps3eye
