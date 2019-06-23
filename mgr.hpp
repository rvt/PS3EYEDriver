#pragma once

#include "ps3eye.hpp"

#include <vector>
#include <thread>
#include <atomic>

struct libusb_context;
struct libusb_device;
struct libusb_device_handle;

namespace ps3eye::detail {

struct usb_manager
{
    usb_manager();
    ~usb_manager();

    static usb_manager& instance();
    std::vector<std::shared_ptr<camera>> list_devices();
    void camera_started();
    void camera_stopped();

    void set_debug(bool value);

private:
    libusb_context* usb_context = nullptr;
    std::thread update_thread;
    std::atomic_int active_camera_count = 0;
    std::atomic_bool exit_signaled = false;

    usb_manager(const usb_manager&);
    void operator=(const usb_manager&);

    void start_xfer_thread();
    void stop_xfer_thread();
    void xfer_callback();

    //int sTotalDevices = 0;
};

} // ns ps3eye::detail
