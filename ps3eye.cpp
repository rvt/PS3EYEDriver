// source code from https://github.com/inspirit/PS3EYEDriver
#include "ps3eye.hpp"
#include "urb.hpp"
#include "mgr.hpp"
#include "internal.hpp"

#include <atomic>
#include <condition_variable>
#include <cstring>
#include <mutex>
#include <thread>
#include <chrono>
#include <vector>
#include <algorithm>
#include <iterator>

#ifdef _MSC_VER
#   pragma warning(disable : 4200) // zero-length arrays
#   pragma warning(disable : 4996) // 'This function or variable may be unsafe': snprintf
#endif

#include <libusb.h>

namespace ps3eye
{

using namespace std::chrono_literals;

enum : uint8_t {
    OV534_REG_ADDRESS = 0xf1, /* sensor address */
    OV534_REG_SUBADDR = 0xf2,
    OV534_REG_WRITE = 0xf3,
    OV534_REG_READ = 0xf4,
    OV534_REG_OPERATION = 0xf5,
    OV534_REG_STATUS = 0xf6,
    OV534_OP_WRITE_3 = 0x37,
    OV534_OP_WRITE_2 = 0x33,
    OV534_OP_READ_2 = 0xf9,
};

static const uint8_t ov534_reg_initdata[][2] = {
    { 0xe7, 0x3a },

    { OV534_REG_ADDRESS, 0x42 }, /* select OV772x sensor */

    { 0x92, 0x01 },
    { 0x93, 0x18 },
    { 0x94, 0x10 },
    { 0x95, 0x10 },
    { 0xE2, 0x00 },
    { 0xE7, 0x3E },

    { 0x96, 0x00 },
    { 0x97, 0x20 },
    { 0x97, 0x20 },
    { 0x97, 0x20 },
    { 0x97, 0x0A },
    { 0x97, 0x3F },
    { 0x97, 0x4A },
    { 0x97, 0x20 },
    { 0x97, 0x15 },
    { 0x97, 0x0B },

    { 0x8E, 0x40 },
    { 0x1F, 0x81 },
    { 0xC0, 0x50 },
    { 0xC1, 0x3C },
    { 0xC2, 0x01 },
    { 0xC3, 0x01 },
    { 0x50, 0x89 },
    { 0x88, 0x08 },
    { 0x8D, 0x00 },
    { 0x8E, 0x00 },

    { 0x1C, 0x00 }, /* video data start (V_FMT) */

    { 0x1D, 0x00 }, /* RAW8 mode */
    { 0x1D, 0x02 }, /* payload size 0x0200 * 4 = 2048 bytes */
    { 0x1D, 0x00 }, /* payload size */

    { 0x1D, 0x01 }, /* frame size = 0x012C00 * 4 = 307200 bytes (640 * 480 @ 8bpp) */
    { 0x1D, 0x2C }, /* frame size */
    { 0x1D, 0x00 }, /* frame size */

    { 0x1C, 0x0A }, /* video data start (V_CNTL0) */
    { 0x1D, 0x08 }, /* turn on UVC header */
    { 0x1D, 0x0E },

    { 0x34, 0x05 },
    { 0xE3, 0x04 },
    { 0x89, 0x00 },
    { 0x76, 0x00 },
    { 0xE7, 0x2E },
    { 0x31, 0xF9 },
    { 0x25, 0x42 },
    { 0x21, 0xF0 },
    { 0xE5, 0x04 }
};

static const uint8_t ov772x_reg_initdata[][2] = {

    { 0x12, 0x80 }, /* reset */
    { 0x3D, 0x00 },

    { 0x12, 0x01 },
    /* Processed Bayer RAW (8bit) */
    { 0x11, 0x01 },
    { 0x14, 0x40 },
    { 0x15, 0x00 },
    { 0x63, 0xAA },
    // AWB
    { 0x64, 0x87 },
    { 0x66, 0x00 },
    { 0x67, 0x02 },
    { 0x17, 0x26 },
    { 0x18, 0xA0 },
    { 0x19, 0x07 },
    { 0x1A, 0xF0 },
    { 0x29, 0xA0 },
    { 0x2A, 0x00 },
    { 0x2C, 0xF0 },
    { 0x20, 0x10 },
    { 0x4E, 0x0F },
    { 0x3E, 0xF3 },
    { 0x0D, 0x41 },
    { 0x32, 0x00 },
    { 0x13, 0xF0 },
    // COM8  - jfrancois 0xf0	orig x0f7
    { 0x22, 0x7F },
    { 0x23, 0x03 },
    { 0x24, 0x40 },
    { 0x25, 0x30 },
    { 0x26, 0xA1 },
    { 0x2A, 0x00 },
    { 0x2B, 0x00 },
    { 0x13, 0xF7 },
    { 0x0C, 0xC0 },

    { 0x11, 0x00 },
    { 0x0D, 0x41 },

    { 0x8E, 0x00 },
    // De-noise threshold - jfrancois 0x00 - orig 0x04
};

static const uint8_t bridge_start_vga[][2] = {
    { 0x1c, 0x00 },
    { 0x1d, 0x00 },
    { 0x1d, 0x02 },
    { 0x1d, 0x00 },
    { 0x1d, 0x01 }, /* frame size = 0x012C00 * 4 = 307200 bytes (640 * 480 @ 8bpp) */
    { 0x1d, 0x2C },
    /* frame size */
    { 0x1d, 0x00 },
    /* frame size */
    { 0xc0, 0x50 },
    { 0xc1, 0x3c },
};
static const uint8_t sensor_start_vga[][2] = {
    { 0x12, 0x01 },
    { 0x17, 0x26 },
    { 0x18, 0xa0 },
    { 0x19, 0x07 },
    { 0x1a, 0xf0 },
    { 0x29, 0xa0 },
    { 0x2c, 0xf0 },
    { 0x65, 0x20 },
};
static const uint8_t bridge_start_qvga[][2] = {
    { 0x1c, 0x00 },
    { 0x1d, 0x00 },
    { 0x1d, 0x02 },
    { 0x1d, 0x00 },
    { 0x1d, 0x00 }, /* frame size = 0x004B00 * 4 = 76800 bytes (320 * 240 @ 8bpp) */
    { 0x1d, 0x4b },
    /* frame size */
    { 0x1d, 0x00 },
    /* frame size */
    { 0xc0, 0x28 },
    { 0xc1, 0x1e },
};
static const uint8_t sensor_start_qvga[][2] = {
    { 0x12, 0x41 },
    { 0x17, 0x3f },
    { 0x18, 0x50 },
    { 0x19, 0x03 },
    { 0x1a, 0x78 },
    { 0x29, 0x50 },
    { 0x2c, 0x78 },
    { 0x65, 0x2f },
};

camera::camera(libusb_device* device)
{
    device_ = device;
}

camera::~camera()
{
    stop();
    release();
}

void camera::release()
{
    if (handle_) close_usb();
}

bool camera::init(int width, int height, int framerate, format fmt)
{
    uint16_t sensor_id;

    // open usb device so we can setup and go
    if (!handle_)
    {
        if (!open_usb())
            return false;
    }

    // find best cam mode
    if ((width == 0 && height == 0) || width > 320 || height > 240)
    {
        frame_width = 640;
        frame_height = 480;
    }
    else
    {
        frame_width = 320;
        frame_height = 240;
    }
    frame_rate = ov534_set_frame_rate(framerate, true);
    frame_output_format = fmt;
    //

    /* reset bridge */
    ov534_reg_write(0xe7, 0x3a);
    ov534_reg_write(0xe0, 0x08);

    std::this_thread::sleep_for(100ms);

    /* initialize the sensor address */
    ov534_reg_write(OV534_REG_ADDRESS, 0x42);

    /* reset sensor */
    sccb_reg_write(0x12, 0x80);

    std::this_thread::sleep_for(100ms);

    /* probe the sensor */
    sccb_reg_read(0x0a);
    sensor_id = sccb_reg_read(0x0a) << 8;
    sccb_reg_read(0x0b);
    sensor_id |= sccb_reg_read(0x0b);
    ps3eye_debug("Sensor ID: %04x\n", sensor_id);

    /* initialize */
    reg_w_array(ov534_reg_initdata, std::size(ov534_reg_initdata));
    ov534_set_led(1);
    sccb_w_array(ov772x_reg_initdata, std::size(ov772x_reg_initdata));
    ov534_reg_write(0xe0, 0x09);
    ov534_set_led(0);

    return true;
}

void camera::start()
{
    if (streaming_)
        return;

    if (frame_width == 320)
    { /* 320x240 */
        reg_w_array(bridge_start_qvga, std::size(bridge_start_qvga));
        sccb_w_array(sensor_start_qvga, std::size(sensor_start_qvga));
    }
    else
    { /* 640x480 */
        reg_w_array(bridge_start_vga, std::size(bridge_start_vga));
        sccb_w_array(sensor_start_vga, std::size(sensor_start_vga));
    }

    ov534_set_frame_rate(frame_rate);

    set_auto_gain(auto_gain_);
    set_awb(awb_);
    set_gain(gain_);
    set_hue(hue_);
    set_exposure(exposure_);
    set_brightness(brightness_);
    set_contrast(contrast_);
    set_sharpness(sharpness_);
    set_red_balance(redblc_);
    set_blue_balance(blueblc_);
    set_green_balance(greenblc_);
    set_flip_status(flip_h_, flip_v_);

    ov534_set_led(1);
    ov534_reg_write(0xe0, 0x00); // start stream

    // init and start urb
    urb.start_transfers(handle_, frame_width * frame_height);
    streaming_ = true;
}

void camera::stop()
{
    if (!streaming_) return;

    /* stop streaming data */
    ov534_reg_write(0xe0, 0x09);
    ov534_set_led(0);

    // close urb
    urb.close_transfers();

    streaming_ = false;
}

#define MAX_USB_DEVICE_PORT_PATH 7

bool camera::usb_port(char* buf, unsigned sz) const
{
    bool success = false;

    if (is_initialized())
    {
        uint8_t port_numbers[MAX_USB_DEVICE_PORT_PATH];

        memset(buf, 0, sz);
        memset(port_numbers, 0, sizeof(port_numbers));

        int cnt = libusb_get_port_numbers(device_, port_numbers, MAX_USB_DEVICE_PORT_PATH);
        int bus_id = libusb_get_bus_number(device_);

        snprintf(buf, sz, "b%d", bus_id);
        if (cnt > 0)
        {
            success = true;

            for (int i = 0; i < cnt; i++)
            {
                uint8_t port_number = port_numbers[i];
                char port_string[8];

                snprintf(port_string, sizeof(port_string),
                         (i == 0) ? "_p%d" : ".%d", port_number);

                if (strlen(buf) + strlen(port_string) + 1 <= sz)
                {
                    std::strcat(buf, port_string);
                }
                else
                {
                    success = false;
                    break;
                }
            }
        }
    }

    return success;
}

int camera::bytes_per_pixel() const
{
    if (frame_output_format == format::Bayer)
        return 1;
    else if (frame_output_format == format::BGR)
        return 3;
    else if (frame_output_format == format::RGB)
        return 3;
    else if (frame_output_format == format::Gray)
        return 1;
    return 0;
}

bool camera::get_frame(uint8_t* frame)
{
    return urb.queue.dequeue(frame, frame_width, frame_height, frame_output_format);
}

bool camera::open_usb()
{
    // open, set first config and claim interface
    int res = libusb_open(device_, &handle_);
    if (res != 0)
    {
        ps3eye_debug("device open error: %d\n", res);
        return false;
    }

    // Linux has a kernel module for the PS3 eye camera (that's where most of
    // the code in here comes from..) so we must detach the driver before we can
    // hook up with the eye ourselves
    libusb_detach_kernel_driver(handle_, 0);

    // libusb_set_configuration(handle_, 0);

    res = libusb_claim_interface(handle_, 0);
    if (res != 0)
    {
        ps3eye_debug("device claim interface error: %d\n", res);
        return false;
    }

    return true;
}

void camera::close_usb()
{
    ps3eye_debug("closing device\n");
    libusb_release_interface(handle_, 0);
    libusb_attach_kernel_driver(handle_, 0);
    libusb_close(handle_);
    libusb_unref_device(device_);
    handle_ = nullptr;
    device_ = nullptr;
    ps3eye_debug("device closed\n");
}

/* Two bits control LED: 0x21 bit 7 and 0x23 bit 7.
 * (direction and output)? */
void camera::ov534_set_led(int status)
{
    uint8_t data;

    ps3eye_debug("led status: %d\n", status);

    data = ov534_reg_read(0x21);
    data |= 0x80;
    ov534_reg_write(0x21, data);

    data = ov534_reg_read(0x23);
    if (status)
        data |= 0x80;
    else
        data &= ~0x80;

    ov534_reg_write(0x23, data);

    if (!status)
    {
        data = ov534_reg_read(0x21);
        data &= ~0x80;
        ov534_reg_write(0x21, data);
    }
}

/* validate frame rate and (if not dry run) set it */
int camera::ov534_set_frame_rate(int frame_rate, bool dry_run)
{
    struct rate_s
    {
        int fps;
        uint8_t r11;
        uint8_t r0d;
        uint8_t re5;
    };

    const struct rate_s* r;

    static constexpr struct rate_s rate_0[] = {
        /* 640x480 */
        { 83, 0x01, 0xc1, 0x02 }, /* 83 FPS: video is partly corrupt */
        { 75, 0x01, 0x81, 0x02 }, /* 75 FPS or below: video is valid */
        { 60, 0x00, 0x41, 0x04 },
        { 50, 0x01, 0x41, 0x02 },
        { 40, 0x02, 0xc1, 0x04 },
        { 30, 0x04, 0x81, 0x02 },
        { 25, 0x00, 0x01, 0x02 },
        { 20, 0x04, 0x41, 0x02 },
        { 15, 0x09, 0x81, 0x02 },
        { 10, 0x09, 0x41, 0x02 },
        {  8, 0x02, 0x01, 0x02 },
        {  5, 0x04, 0x01, 0x02 },
        {  3, 0x06, 0x01, 0x02 },
        {  2, 0x09, 0x01, 0x02 },
    };

    static constexpr struct rate_s rate_1[] = {
        /* 320x240 */
        { 290, 0x00, 0xc1, 0x04 },
        { 205, 0x01, 0xc1, 0x02 }, /* 205 FPS or above: video is partly corrupt */
        { 187, 0x01, 0x81, 0x02 }, /* 187 FPS or below: video is valid */
        { 150, 0x00, 0x41, 0x04 },
        { 137, 0x02, 0xc1, 0x02 },
        { 125, 0x01, 0x41, 0x02 },
        { 100, 0x02, 0xc1, 0x04 },
        {  90, 0x03, 0x81, 0x02 },
        {  75, 0x04, 0x81, 0x02 },
        {  60, 0x04, 0xc1, 0x04 },
        {  50, 0x04, 0x41, 0x02 },
        {  40, 0x06, 0x81, 0x03 },
        {  37, 0x00, 0x01, 0x04 },
        {  30, 0x04, 0x41, 0x04 },
        {  17, 0x18, 0xc1, 0x02 },
        {  15, 0x18, 0x81, 0x02 },
        {  12, 0x02, 0x01, 0x04 },
        {  10, 0x18, 0x41, 0x02 },
        {   7, 0x04, 0x01, 0x04 },
        {   5, 0x06, 0x01, 0x04 },
        {   3, 0x09, 0x01, 0x04 },
        {   2, 0x18, 0x01, 0x02 },
    };

    int i;

    if (frame_width == 640)
    {
        r = rate_0;
        i = std::size(rate_0);
    }
    else
    {
        r = rate_1;
        i = std::size(rate_1);
    }
    while (--i > 0)
    {
        if (frame_rate >= r->fps) break;
        r++;
    }

    if (!dry_run)
    {
        sccb_reg_write(0x11, r->r11);
        sccb_reg_write(0x0d, r->r0d);
        ov534_reg_write(0xe5, r->re5);
    }

    ps3eye_debug("frame_rate: %d\n", r->fps);
    return r->fps;
}

void camera::ov534_reg_write(uint16_t reg, uint8_t val)
{
    int ret;

    // debug("reg=0x%04x, val=0%02x", reg, val);
    usb_buf[0] = val;

    ret = libusb_control_transfer(handle_, LIBUSB_ENDPOINT_OUT | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE,
                                  0x01, 0x00, reg, usb_buf.data(), 1, 500);
    if (ret < 0)
    {
        ps3eye_debug("write failed\n");
    }
}

uint8_t camera::ov534_reg_read(uint16_t reg)
{
    int ret;

    ret = libusb_control_transfer(handle_, LIBUSB_ENDPOINT_IN | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE,
                                  0x01, 0x00, reg, usb_buf.data(), 1, 500);

    // debug("reg=0x%04x, data=0x%02x", reg, usb_buf[0]);
    if (ret < 0)
    {
        ps3eye_debug("read failed\n");
    }
    return usb_buf[0];
}

int camera::sccb_check_status()
{
    uint8_t data;
    int i;

    for (i = 0; i < 5; i++)
    {
        data = ov534_reg_read(OV534_REG_STATUS);

        switch (data)
        {
        case 0x00:
            return 1;
        case 0x04:
            return 0;
        case 0x03:
            break;
        default:
            ps3eye_debug("sccb status 0x%02x, attempt %d/5\n", data, i + 1);
        }
    }
    return 0;
}

void camera::sccb_reg_write(uint8_t reg, uint8_t val)
{
    // debug("reg: 0x%02x, val: 0x%02x", reg, val);
    ov534_reg_write(OV534_REG_SUBADDR, reg);
    ov534_reg_write(OV534_REG_WRITE, val);
    ov534_reg_write(OV534_REG_OPERATION, OV534_OP_WRITE_3);

    if (!sccb_check_status())
    {
        ps3eye_debug("sccb_reg_write failed\n");
    }
}

uint8_t camera::sccb_reg_read(uint16_t reg)
{
    ov534_reg_write(OV534_REG_SUBADDR, (uint8_t)reg);
    ov534_reg_write(OV534_REG_OPERATION, OV534_OP_WRITE_2);
    if (!sccb_check_status())
    {
        ps3eye_debug("sccb_reg_read failed 1\n");
    }

    ov534_reg_write(OV534_REG_OPERATION, OV534_OP_READ_2);
    if (!sccb_check_status())
    {
        ps3eye_debug("sccb_reg_read failed 2\n");
    }

    return ov534_reg_read(OV534_REG_READ);
}
/* output a bridge sequence (reg - val) */
void camera::reg_w_array(const uint8_t (*data)[2], int len)
{
    while (--len >= 0)
    {
        ov534_reg_write((*data)[0], (*data)[1]);
        data++;
    }
}

/* output a sensor sequence (reg - val) */
void camera::sccb_w_array(const uint8_t (*data)[2], int len)
{
    while (--len >= 0)
    {
        if ((*data)[0] != 0xff)
        {
            sccb_reg_write((*data)[0], (*data)[1]);
        }
        else
        {
            sccb_reg_read((*data)[1]);
            sccb_reg_write(0xff, 0x00);
        }
        data++;
    }
}

//bool camera::enumerated = false;
//std::vector<std::shared_ptr<camera>> camera::devices;

std::vector<std::shared_ptr<camera>> camera::list_devices()
{
#if 0
    if (enumerated && (!forceRefresh)) return devices;

    devices.clear();

    //USBMgr::instance()->sTotalDevices =
    devices = USBMgr::instance()->list_devices();

    enumerated = true;
    return devices;
#else
    return USBMgr::instance().list_devices();
#endif
}

void camera::set_auto_gain(bool val)
{
    auto_gain_ = val;
    if (val)
    {
        sccb_reg_write(0x13, 0xf7); // AGC,AEC,AWB ON
        sccb_reg_write(0x64, sccb_reg_read(0x64) | 0x03);
    }
    else
    {
        sccb_reg_write(0x13, 0xf0); // AGC,AEC,AWB OFF
        sccb_reg_write(0x64, sccb_reg_read(0x64) & 0xFC);

        set_gain(gain_);
        set_exposure(exposure_);
    }
}

void camera::set_awb(bool val)
{
    awb_ = val;
    if (val)
    {
        sccb_reg_write(0x63, 0xe0); // AWB ON
    }
    else
    {
        sccb_reg_write(0x63, 0xAA); // AWB OFF
    }
}

bool camera::set_framerate(int val)
{
    if (streaming_) return false;
    frame_rate = ov534_set_frame_rate((uint8_t)std::clamp(val, 0, 512), true);
    return true;
}

void camera::set_test_pattern_status(bool enable)
{
    test_pattern_ = enable;
    uint8_t val = sccb_reg_read(0x0C);
    val &= ~0b00000001;
    if (test_pattern_) val |= 0b00000001; // 0x80;
    sccb_reg_write(0x0C, val);
}

void camera::set_exposure(int val)
{
    exposure_ = (uint8_t)std::clamp(val, 0, 255);
    sccb_reg_write(0x08, exposure_ >> 7);
    sccb_reg_write(0x10, uint8_t(exposure_ << 1));
}

void camera::set_sharpness(int val)
{
    sharpness_ = (uint8_t)std::clamp(val, 0, 63);
    sccb_reg_write(0x91, sharpness_); // vga noise
    sccb_reg_write(0x8E, sharpness_); // qvga noise
}

void camera::set_contrast(int val)
{
    contrast_ = (uint8_t)std::clamp(val, 0, 255);
    sccb_reg_write(0x9C, contrast_);
}

void camera::set_brightness(int val)
{
    brightness_ = (uint8_t)std::clamp(val, 0, 255);
    sccb_reg_write(0x9B, brightness_);
}

void camera::set_hue(int val)
{
    hue_ = (uint8_t)std::clamp(val, 0, 255);
    sccb_reg_write(0x01, hue_);
}

void camera::set_red_balance(int val)
{
    redblc_ = (uint8_t)std::clamp(val, 0, 255);
    sccb_reg_write(0x43, redblc_);
}

void camera::set_blue_balance(int val)
{
    blueblc_ = (uint8_t)std::clamp(val, 0, 255);
    sccb_reg_write(0x42, blueblc_);
}

void camera::set_green_balance(int val)
{
    greenblc_ = (uint8_t)std::clamp(val, 0, 255);
    sccb_reg_write(0x44, greenblc_);
}

void camera::set_flip_status(bool horizontal, bool vertical)
{
    flip_h_ = horizontal;
    flip_v_ = vertical;
    uint8_t val = sccb_reg_read(0x0c);
    val &= ~0xc0;
    if (!horizontal) val |= 0x40;
    if (!vertical) val |= 0x80;
    sccb_reg_write(0x0c, val);
}

void camera::set_gain(int val)
{
    gain_ = (uint8_t)std::clamp(val, 0, 63);
    val = gain_;
    switch (val & 0x30)
    {
    case 0x00:
        val &= 0x0F;
        break;
    case 0x10:
        val &= 0x0F;
        val |= 0x30;
        break;
    case 0x20:
        val &= 0x0F;
        val |= 0x70;
        break;
    case 0x30:
        val &= 0x0F;
        val |= 0xF0;
        break;
    }
    sccb_reg_write(0x00, (uint8_t)val);
}

} // namespace ps3eye
