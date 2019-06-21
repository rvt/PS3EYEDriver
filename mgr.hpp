#pragma once

#include <memory>
#include <vector>
#include <thread>
#include <atomic>

struct libusb_context;

namespace ps3eye {

struct PS3EYECam;

struct USBMgr
{
    USBMgr();
    ~USBMgr();

    static std::shared_ptr<USBMgr> instance();
    int listDevices(std::vector<std::shared_ptr<PS3EYECam>>& list);
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
