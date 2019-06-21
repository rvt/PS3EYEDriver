// source code from https://github.com/inspirit/PS3EYEDriver
#pragma once

#include "internal.hpp"

#include <cstdint>
#include <memory>
#include <vector>
#include <array>

struct libusb_device;
struct libusb_device_handle;

namespace ps3eye
{

struct URBDesc;
struct USBMgr;
struct PS3EYECam;

using PS3EYERef = std::shared_ptr<PS3EYECam>;

struct PS3EYECam
{
    explicit PS3EYECam(libusb_device* device);
    ~PS3EYECam();

    bool init(uint32_t width = 0,
              uint32_t height = 0,
              uint16_t desiredFrameRate = 30,
              EOutputFormat outputFormat = EOutputFormat::BGR);
    void start();
    void stop();

    // Controls

    bool getAutogain() const { return autogain; }
    void setAutogain(bool val);
    bool getAutoWhiteBalance() const { return awb; }
    void setAutoWhiteBalance(bool val);
    uint8_t getGain() const { return gain; }
    void setGain(int val);
    uint8_t getExposure() const { return exposure; }
    void setExposure(int val);
    uint8_t getSharpness() const { return sharpness; }
    void setSharpness(int val);
    uint8_t getContrast() const { return contrast; }
    void setContrast(int val);
    uint8_t getBrightness() const { return brightness; }
    void setBrightness(int val);
    uint8_t getHue() const { return hue; }
    void setHue(int val);
    uint8_t getRedBalance() const { return redblc; }
    void setRedBalance(int val);
    uint8_t getBlueBalance() const { return blueblc; }
    void setBlueBalance(int val);
    uint8_t getGreenBalance() const { return greenblc; }
    void setGreenBalance(int val);
    bool getFlipH() const { return flip_h; }
    bool getFlipV() const { return flip_v; }
    void setFlip(bool horizontal = false, bool vertical = false);

    bool getTestPattern() const { return testPattern; }
    void setTestPattern(bool enable);

    bool isStreaming() const { return is_streaming; }
    bool isInitialized() const;

    libusb_device* getDevice() const { return device_; }
    bool getUSBPortPath(char* out_identifier, size_t max_identifier_length) const;

    // Get a frame from the camera. Notes:
    // - If there is no frame available, this function will block until one is
    // - The output buffer must be sized correctly, depending out the output
    // format. See EOutputFormat.
    void getFrame(uint8_t* frame);

    uint32_t getWidth() const { return frame_width; }
    uint32_t getHeight() const { return frame_height; }
    uint16_t getFrameRate() const { return frame_rate; }
    [[nodiscard]] bool setFrameRate(int val);
    uint32_t getRowBytes() const { return frame_width * getOutputBytesPerPixel(); }
    uint32_t getOutputBytesPerPixel() const;

    static const std::vector<std::shared_ptr<PS3EYECam>>& getDevices(bool forceRefresh = false);

    PS3EYECam(const PS3EYECam&) = delete;
    void operator=(const PS3EYECam&) = delete;

private:
    void release();

    // usb ops
    uint16_t ov534_set_frame_rate(uint16_t frame_rate, bool dry_run = false);
    void ov534_set_led(int status);
    void ov534_reg_write(uint16_t reg, uint8_t val);
    uint8_t ov534_reg_read(uint16_t reg);
    int sccb_check_status();
    void sccb_reg_write(uint8_t reg, uint8_t val);
    uint8_t sccb_reg_read(uint16_t reg);
    void reg_w_array(const uint8_t (*data)[2], int len);
    void sccb_w_array(const uint8_t (*data)[2], int len);

    // controls
    bool autogain = false;
    uint8_t gain = 20; // 0 <-> 63
    uint8_t exposure = 120;  // 0 <-> 255
    uint8_t sharpness = 0; // 0 <-> 63
    uint8_t hue = 143;       // 0 <-> 255
    bool awb = false;
    uint8_t brightness = 20; // 0 <-> 255
    uint8_t contrast = 37;   // 0 <-> 255
    uint8_t blueblc = 128;    // 0 <-> 255
    uint8_t redblc = 128;     // 0 <-> 255
    uint8_t greenblc = 128;   // 0 <-> 255
    bool flip_h = false;
    bool flip_v = false;
    bool testPattern = false;
    bool is_streaming = false;

    std::shared_ptr<USBMgr> mgrPtr;

    static bool devicesEnumerated;
    static std::vector<std::shared_ptr<PS3EYECam>> devices;

    int frame_width = 0;
    int frame_height = 0;
    uint16_t frame_rate;
    EOutputFormat frame_output_format = EOutputFormat::BGR;

    // usb stuff
    libusb_device* device_ = nullptr;
    libusb_device_handle* handle_ = nullptr;
    std::array<uint8_t, 64> usb_buf;

    std::shared_ptr<URBDesc> urb;

    bool open_usb();
    void close_usb();
};

} // namespace ps3eye
