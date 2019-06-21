#pragma once

#include <vector>
#include <thread>
#include <atomic>

struct libusb_context;

namespace ps3eye {

struct camera;

struct USBMgr
{
    USBMgr();
    ~USBMgr();

    static USBMgr& instance();
    std::vector<std::shared_ptr<camera>> list_devices();
    void cameraStarted();
    void cameraStopped();

private:
    libusb_context* usb_context;
    std::thread update_thread;
    std::atomic_int active_camera_count;
    std::atomic_bool exit_signaled;

    USBMgr(const USBMgr&);
    void operator=(const USBMgr&);

    void startTransferThread();
    void stopTransferThread();
    void transferThreadFunc();

    //int sTotalDevices = 0;
};

} // ns ps3eye
