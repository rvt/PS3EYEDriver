#include "ps3eye.hpp"
#include <thread>
#include <chrono>
#include <cstdlib>

static bool test(ps3eye::camera& cam, int fps, ps3eye::resolution res)
{
    std::vector<uint8_t> buf;

    if (!cam.init(res, fps)) { fprintf(stderr, "failed to init camera\n"); return false; }
    if (!cam.start()) { fprintf(stderr, "failed to start camera\n"); return false; }

    auto [ w, h ] = cam.size();
    auto size = unsigned(w*h*cam.bytes_per_pixel());
    buf.reserve(size);

    bool ret = false;

    using namespace std::chrono_literals;

    for (unsigned k = 0; k < 10; k++)
    {
        if (fps > 0) std::this_thread::sleep_for(1ms * 500/fps);
        ret |= cam.get_frame(buf.data());
        if (ret)
            break;
    }

    if (ret)
    {
        for (int k = 0; k < fps * 10; k++)
        {
            ret &= cam.get_frame(buf.data());
            if (!ret)
                break;
        }
    }

    printf("[%s] %dx%d@%dHz\n", ret ? "GOOD" : "FAIL", cam.width(), cam.height(), cam.framerate());

    return ret;
};

int main(void)
{
    static const int rates_qvga[] = { 2,  3,  5,  7,  10, 12,  15,  17,  30,  37,
                                      40, 50, 60, 75, 90, 100, 125, 137, 150, 187 };
    static const int rates_svga[] = { 2, 3, 5, 8, 10, 15, 20, 25, 30, 40, 50, 60, 75, 83 };

    ps3eye::camera::set_debug(false);
    auto devices = ps3eye::list_devices();
    constexpr int ex_NOINPUT = 66 /* sysexits(3) */;

    if (devices.empty())
    {
        fprintf(stderr, "no camera\n");
        return ex_NOINPUT;
    }

    auto cam = devices[0];
    bool status = true;

    for (unsigned i = 0, last_fps = 0; i < std::size(rates_qvga); ++i)
    {
        auto fps = (unsigned)ps3eye::camera::normalize_framerate(rates_qvga[i], ps3eye::res_QVGA);
        if (fps == last_fps)
            continue;
        last_fps = fps;
        status &= test(*cam, rates_qvga[i], ps3eye::res_QVGA);
    }

    for (unsigned i = 0, last_fps = 0; i < std::size(rates_svga); ++i)
    {
        auto fps = (unsigned)ps3eye::camera::normalize_framerate(rates_qvga[i], ps3eye::res_VGA);
        if (fps == last_fps)
            continue;
        last_fps = fps;
        status &= test(*cam, rates_qvga[i], ps3eye::res_VGA);
    }

    return status ? EXIT_SUCCESS : ex_NOINPUT;
}
