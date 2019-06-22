#include "ps3eye.hpp"
#include <cstdlib>

int main(void)
{
    static const int rates_qvga[] = { 2,  3,  5,  7,  10, 12,  15,  17,  30,  37,
                                      40, 50, 60, 75, 90, 100, 125, 137, 150, 187 };
    static const int rates_svga[] = { 2, 3, 5, 8, 10, 15, 20, 25, 30, 40, 50, 60, 75, 83 };

    ps3eye::camera::set_debug(false);
    auto devices = ps3eye::list_devices();

    if (devices.empty())
    {
        fprintf(stderr, "no camera\n");
        return EXIT_FAILURE;
    }

    auto cam = devices[0];
    int status = EXIT_SUCCESS;
    bool running = true;

    auto test = [&](int fps, ps3eye::resolution res) {
        running &= cam->init(res, fps);
        running &= cam->start();

        auto [ w, h ] = cam->size();
        auto size = unsigned(w*h*cam->bytes_per_pixel());

        std::vector<uint8_t> buf(size);

        bool got_frame = false;

        for (unsigned k = 0; k < 5; k++)
        {
            got_frame |= cam->get_frame(buf.data());
            if (got_frame)
                break;
        }

        running &= got_frame;

        printf("[%s] %dx%d@%dHz\n", running ? "GOOD" : "FAIL", w, h, cam->framerate());

        if (!running)
            status = EXIT_FAILURE;
    };

    for (unsigned i = 0, last_fps = 0; i < std::size(rates_qvga); ++i)
    {
        auto fps = (unsigned)ps3eye::camera::normalize_framerate(rates_qvga[i], ps3eye::res_QVGA);
        if (fps == last_fps)
            continue;
        last_fps = fps;
        test(rates_qvga[i], ps3eye::res_QVGA);
    }

    for (unsigned i = 0, last_fps = 0; i < std::size(rates_svga); ++i)
    {
        auto fps = (unsigned)ps3eye::camera::normalize_framerate(rates_qvga[i], ps3eye::res_SVGA);
        if (fps == last_fps)
            continue;
        last_fps = fps;
        test(rates_qvga[i], ps3eye::res_SVGA);
    }

    return status;
}

