// source code from https://github.com/inspirit/PS3EYEDriver
#pragma once

#include "urb.hpp"
#include "internal.hpp"

#include <cstdint>
#include <vector>
#include <array>

struct libusb_device;
struct libusb_device_handle;

namespace ps3eye
{

struct urb_descriptor;
struct USBMgr;

struct camera
{
    explicit camera(libusb_device* device);
    ~camera();

    bool init(int width = 0, int height = 0, int framerate = 30, format fmt = format::BGR);
    void start();
    void stop();

    // Controls
    bool auto_gain() const { return auto_gain_; }
    void set_auto_gain(bool val);
    bool awb() const { return awb_; }
    void set_awb(bool val);
    uint8_t gain() const { return gain_; }
    void set_gain(int val);
    uint8_t exposure() const { return exposure_; }
    void set_exposure(int val);
    uint8_t sharpness() const { return sharpness_; }
    void set_sharpness(int val);
    uint8_t contrast() const { return contrast_; }
    void set_contrast(int val);
    uint8_t brightness() const { return brightness_; }
    void set_brightness(int val);
    uint8_t hue() const { return hue_; }
    void set_hue(int val);
    uint8_t red_balance() const { return redblc_; }
    void set_red_balance(int val);
    uint8_t blue_balance() const { return blueblc_; }
    void set_blue_balance(int val);
    uint8_t green_balance() const { return greenblc_; }
    void set_green_balance(int val);
    std::tuple<bool, bool> flip_status() { return { flip_h_, flip_v_ }; }
    void set_flip_status(bool horizontal = false, bool vertical = false);
    bool test_pattern_status() const { return test_pattern_; }
    void set_test_pattern_status(bool enable);
    int framerate() const { return frame_rate; }
    bool set_framerate(int val);

    bool is_open() const { return streaming_; }
    bool is_initialized() const { return device_ && handle_; }

    libusb_device* device() const { return device_; }
    [[nodiscard]] bool usb_port(char* buf, unsigned sz) const;

    // Get a frame from the camera. Notes:
    // - If there is no frame available, this function will block until one is
    // - The output buffer must be sized correctly, depending out the output
    // format. See format.
    [[nodiscard]] bool get_frame(uint8_t* frame);

    int width() const { return frame_width; }
    int height() const { return frame_height; }
    int stride() const { return frame_width * bytes_per_pixel(); }
    int bytes_per_pixel() const;

    static std::vector<std::shared_ptr<camera>> list_devices();

    camera(const camera&) = delete;
    void operator=(const camera&) = delete;

private:
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

    // controls
    uint8_t gain_ = 20; // 0 <-> 63
    uint8_t exposure_ = 120;  // 0 <-> 255
    uint8_t sharpness_ = 0; // 0 <-> 63
    uint8_t hue_ = 143;       // 0 <-> 255
    uint8_t brightness_ = 20; // 0 <-> 255
    uint8_t contrast_ = 37;   // 0 <-> 255
    uint8_t blueblc_ = 128;    // 0 <-> 255
    uint8_t redblc_ = 128;     // 0 <-> 255
    uint8_t greenblc_ = 128;   // 0 <-> 255

    bool auto_gain_ = false;
    bool awb_ = false;
    bool flip_h_ = false;
    bool flip_v_ = false;
    bool test_pattern_ = false;
    bool streaming_ = false;

    //static bool enumerated;
    //static std::vector<std::shared_ptr<camera>> devices;

    int frame_width = 0;
    int frame_height = 0;
    int frame_rate = 0;
    format frame_output_format = format::BGR;

    // usb stuff
    libusb_device* device_ = nullptr;
    libusb_device_handle* handle_ = nullptr;
    urb_descriptor urb;
    std::array<uint8_t, 64> usb_buf;
};

} // namespace ps3eye
