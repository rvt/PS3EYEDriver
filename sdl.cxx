/**
 * PS3EYEDriver Simple SDL 2 example, using OpenGL where available.
 * Thomas Perl <m@thp.io>; 2014-01-10
 * Joseph Howse <josephhowse@nummist.com>; 2014-12-26
 **/
#include "ps3eye.hpp"

#include <iostream>
#include <sstream>
#include <iterator>
#include <thread>
#include <chrono>

#include <SDL.h>

static void print_renderer_info(SDL_Renderer* renderer)
{
    SDL_RendererInfo renderer_info;
    SDL_GetRendererInfo(renderer, &renderer_info);
    printf("Renderer: %s\n", renderer_info.name);
}

static void run_camera(ps3eye::resolution res, int fps)
{
    ps3eye::camera::set_debug(true);

    auto cameras = ps3eye::list_devices();

    if (cameras.empty())
    {
        fprintf(stderr, "no device\n");
        return;
    }

    auto camera = cameras[0];
    bool running = true;

    running &= camera->init(res, fps, ps3eye::fmt_BGR);
    running &= camera->start();

    if (!running)
    {
        fprintf(stderr, "device init failed\n");
        return;
    }

    //camera->set_flip_status(true); /* mirrored left-right */

    auto [ w, h ] = camera->size();

    char title[256];
    sprintf(title, "%dx%d@%dHz\n", w, h, camera->framerate());

    SDL_Window* window = SDL_CreateWindow(title,
                                          SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED,
                                          w, h, 0);
    if (!window)
    {
        fprintf(stderr, "Failed to create window: %s\n", SDL_GetError());
        return;
    }

    SDL_Renderer* renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);
    if (!renderer)
    {
        fprintf(stderr, "Failed to create renderer: %s\n", SDL_GetError());
        SDL_DestroyWindow(window);
        return;
    }
    SDL_RenderSetLogicalSize(renderer, camera->width(), camera->height());
    print_renderer_info(renderer);

    SDL_Texture* video_tex =
        SDL_CreateTexture(renderer, SDL_PIXELFORMAT_BGR24, SDL_TEXTUREACCESS_STREAMING,
                          camera->width(), camera->height());

    if (!video_tex)
    {
        fprintf(stderr, "Failed to create video texture: %s\n", SDL_GetError());
        SDL_DestroyRenderer(renderer);
        SDL_DestroyWindow(window);
        return;
    }

    fprintf(stderr, "camera mode: %dx%d@%dHz\n", w, h, camera->framerate());

    SDL_Event e;
    unsigned last_ticks = 0;
    unsigned last_frames = 0;

    while (running)
    {
        while (SDL_PollEvent(&e))
        {
            if (e.type == SDL_QUIT ||
                (e.type == SDL_KEYUP && e.key.keysym.scancode == SDL_SCANCODE_ESCAPE))
            {
                running = false;
            }
        }

        {
            unsigned now_ticks = SDL_GetTicks();

            last_frames++;

            if (ps3eye::camera::is_debugging() &&
                now_ticks - last_ticks > 1000 * 10)
            {
                fprintf(stderr, "FPS: %.2f\n", last_frames * 1000 / double(now_ticks - last_ticks));
                last_ticks = now_ticks;
                last_frames = 0;
            }
        }

        void* video_tex_pixels;
        int pitch;
        SDL_LockTexture(video_tex, nullptr, &video_tex_pixels, &pitch);

        using namespace std::chrono_literals;

        if (int fps = camera->framerate(); fps > 0 && fps < 60)
            std::this_thread::sleep_for(1ms * fps/2);
        bool status = camera->get_frame((uint8_t*)video_tex_pixels);

        SDL_UnlockTexture(video_tex);

        if (status)
            SDL_RenderCopy(renderer, video_tex, nullptr, nullptr);
        SDL_RenderPresent(renderer);
    }

    camera->stop();

    SDL_DestroyTexture(video_tex);
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
}

int main(int argc, char** argv)
{
    ps3eye::resolution res = ps3eye::res_VGA;
    int fps = 60;

    for (int i = 1; i < argc; i++)
    {
        bool good_arg = false;

        if (std::string(argv[i]) == "--qvga")
        {
            res = ps3eye::res_QVGA;
            good_arg = true;
        }
        if ((std::string(argv[i]) == "--fps") && argc > i)
        {
            std::istringstream new_fps_ss(argv[i + 1]);
            if (new_fps_ss >> fps)
            {
                good_arg = true;
            }
            i++;
        }

        if (!good_arg)
        {
            std::cerr << "Usage: " << argv[0]
                      << " [--fps num] [--qvga]" << std::endl;
            return 64 /* EX_USAGE */;
        }
    }

    if (SDL_Init(SDL_INIT_VIDEO) < 0)
    {
        printf("Failed to initialize SDL: %s\n", SDL_GetError());
        return EXIT_FAILURE;
    }

    run_camera(res, fps);

    return EXIT_SUCCESS;
}
