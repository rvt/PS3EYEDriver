/**
 * PS3EYEDriver Simple SDL 2 example, using OpenGL where available.
 * Thomas Perl <m@thp.io>; 2014-01-10
 * Joseph Howse <josephhowse@nummist.com>; 2014-12-26
 **/
#include "../ps3eye.hpp"
#include <SDL.h>
#include <iostream>
#include <sstream>

struct context
{
    context(int width, int height, int fps)
    {
        if (hasDevices())
        {
            eye = devices[0];
            eye->init(width, height, (uint16_t)fps);
        }
    }

    bool hasDevices() { return (devices.size() > 0); }

    std::shared_ptr<ps3eye::camera> eye;
    std::vector<std::shared_ptr<ps3eye::camera>> devices { ps3eye::camera::list_devices() };

    Uint32 last_ticks = 0;
    Uint32 last_frames = 0;
    bool running = true;
};

static void print_renderer_info(SDL_Renderer* renderer)
{
    SDL_RendererInfo renderer_info;
    SDL_GetRendererInfo(renderer, &renderer_info);
    printf("Renderer: %s\n", renderer_info.name);
}

static void run_camera(int width, int height, int fps, Uint32 duration)
{
    context ctx(width, height, fps);
    if (!ctx.hasDevices())
    {
        printf("No PS3 Eye camera connected\n");
        return;
    }
    ctx.eye->set_flip_status(true); /* mirrored left-right */

    char title[256];
    sprintf(title, "%dx%d@%d\n", ctx.eye->width(), ctx.eye->height(), ctx.eye->framerate());

    SDL_Window* window = SDL_CreateWindow(title, SDL_WINDOWPOS_UNDEFINED,
                                          SDL_WINDOWPOS_UNDEFINED, width, height, 0);
    if (window == nullptr)
    {
        printf("Failed to create window: %s\n", SDL_GetError());
        return;
    }

    SDL_Renderer* renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);
    if (renderer == nullptr)
    {
        printf("Failed to create renderer: %s\n", SDL_GetError());
        SDL_DestroyWindow(window);
        return;
    }
    SDL_RenderSetLogicalSize(renderer, ctx.eye->width(), ctx.eye->height());
    print_renderer_info(renderer);

    SDL_Texture* video_tex =
        SDL_CreateTexture(renderer, SDL_PIXELFORMAT_BGR24, SDL_TEXTUREACCESS_STREAMING,
                          ctx.eye->width(), ctx.eye->height());

    if (video_tex == nullptr)
    {
        printf("Failed to create video texture: %s\n", SDL_GetError());
        SDL_DestroyRenderer(renderer);
        SDL_DestroyWindow(window);
        return;
    }

    ctx.eye->start();

    printf("Camera mode: %dx%d@%d\n", ctx.eye->width(), ctx.eye->height(), ctx.eye->framerate());

    SDL_Event e;

    Uint32 start_ticks = SDL_GetTicks();
    while (ctx.running)
    {
        if (duration != 0 && (SDL_GetTicks() - start_ticks) / 1000 >= duration)
            break;

        while (SDL_PollEvent(&e))
        {
            if (e.type == SDL_QUIT ||
                (e.type == SDL_KEYUP && e.key.keysym.scancode == SDL_SCANCODE_ESCAPE))
            {
                ctx.running = false;
            }
        }

        {
            Uint32 now_ticks = SDL_GetTicks();

            ctx.last_frames++;

            if (now_ticks - ctx.last_ticks > 1000)
            {
                fprintf(stderr, "FPS: %.2f\n", 1000 * ctx.last_frames / double(now_ticks - ctx.last_ticks));
                ctx.last_ticks = now_ticks;
                ctx.last_frames = 0;
            }
        }

        void* video_tex_pixels;
        int pitch;
        SDL_LockTexture(video_tex, nullptr, &video_tex_pixels, &pitch);
        bool status = ctx.eye->get_frame((uint8_t*)video_tex_pixels);
        SDL_UnlockTexture(video_tex);

        if (!status)
            break;

        SDL_RenderCopy(renderer, video_tex, nullptr, nullptr);
        SDL_RenderPresent(renderer);
    }

    ctx.eye->stop();

    SDL_DestroyTexture(video_tex);
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
}

int main(int argc, char* argv[])
{
    bool mode_test = false;
    int width = 640;
    int height = 480;
    int fps = 60;
    if (argc > 1)
    {
        bool good_arg = false;
        for (int i = 1; i < argc; i++)
        {
            if (std::string(argv[i]) == "--qvga")
            {
                width = 320;
                height = 240;
                good_arg = true;
            }

            if ((std::string(argv[i]) == "--fps") && argc > i)
            {
                std::istringstream new_fps_ss(argv[i + 1]);
                if (new_fps_ss >> fps)
                {
                    good_arg = true;
                }
            }

            if (std::string(argv[i]) == "--mode_test")
            {
                mode_test = true;
                good_arg = true;
            }
        }
        if (!good_arg)
        {
            std::cerr << "Usage: " << argv[0]
                      << " [--fps XX] [--qvga] [--mode_test]" << std::endl;
        }
    }

    if (SDL_Init(SDL_INIT_VIDEO) < 0)
    {
        printf("Failed to initialize SDL: %s\n", SDL_GetError());
        return EXIT_FAILURE;
    }

    if (mode_test)
    {
        int rates_qvga[] = { 2,  3,  5,  7,  10, 12,  15,  17,  30,  37,
                             40, 50, 60, 75, 90, 100, 125, 137, 150, 187 };
        int num_rates_qvga = sizeof(rates_qvga) / sizeof(int);

        int rates_vga[] = { 2, 3, 5, 8, 10, 15, 20, 25, 30, 40, 50, 60, 75 };
        int num_rates_vga = sizeof(rates_vga) / sizeof(int);

        for (int index = 0; index < num_rates_qvga; ++index)
            run_camera(320, 240, rates_qvga[index], 5);

        for (int index = 0; index < num_rates_vga; ++index)
            run_camera(640, 480, rates_vga[index], 5);
    }
    else
    {
        run_camera(width, height, fps, 0);
    }

    return EXIT_SUCCESS;
}
