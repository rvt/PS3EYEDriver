#include "mgr.hpp"
#include "internal.hpp"
#include "ps3eye.hpp"

#include <functional>
#include <libusb.h>

namespace ps3eye {

USBMgr::USBMgr()
{
    exit_signaled = false;
    active_camera_count = 0;
    libusb_init(&usb_context);
    libusb_set_option(usb_context, LIBUSB_OPTION_LOG_LEVEL, LIBUSB_LOG_LEVEL_WARNING);
}

USBMgr::~USBMgr()
{
    ps3eye_debug("USBMgr destructor\n");
    libusb_exit(usb_context);
}

USBMgr& USBMgr::instance()
{
    static USBMgr ret;
    return std::ref(ret);
}

void USBMgr::cameraStarted()
{
    if (active_camera_count++ == 0) startTransferThread();
}

void USBMgr::cameraStopped()
{
    if (--active_camera_count == 0) stopTransferThread();
}

void USBMgr::startTransferThread()
{
    update_thread = std::thread(&USBMgr::transferThreadFunc, this);
}

void USBMgr::stopTransferThread()
{
    exit_signaled = true;
    update_thread.join();
    // Reset the exit signal flag.
    // If we don't and we call startTransferThread() again, transferThreadFunc
    // will exit immediately.
    exit_signaled = false;
}

void USBMgr::transferThreadFunc()
{
    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 50 * 1000; // ms

    while (!exit_signaled)
    {
        libusb_handle_events_timeout_completed(usb_context, &tv, nullptr);
    }
}

std::vector<std::shared_ptr<camera>> USBMgr::list_devices()
{
    std::vector<std::shared_ptr<camera>> list;

    libusb_device* dev;
    libusb_device** devs;
    libusb_device_handle* devhandle;
    int i = 0;
    int cnt;

    cnt = (int)libusb_get_device_list(usb_context, &devs);

    if (cnt < 0)
    {
        ps3eye_debug("Error Device scan\n");
    }

    cnt = 0;
    while ((dev = devs[i++]) != nullptr)
    {
        struct libusb_device_descriptor desc;
        libusb_get_device_descriptor(dev, &desc);
        if (desc.idVendor == VENDOR_ID && desc.idProduct == PRODUCT_ID)
        {
            int err = libusb_open(dev, &devhandle);
            if (err == 0)
            {
                libusb_close(devhandle);
                list.push_back(std::make_shared<camera>(dev));
                libusb_ref_device(dev);
                cnt++;
            }
        }
    }

    libusb_free_device_list(devs, 1);

    return list;
}

} // ns ps3eye
