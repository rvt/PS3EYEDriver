/**
 * PS3EYEDriver Simple SDL 2 example, using OpenGL where available.
 * Thomas Perl <m@thp.io>; 2014-01-10
 * Joseph Howse <josephhowse@nummist.com>; 2014-12-26
 **/
#include "../ps3eye.hpp"

#include <iostream>
#include <sstream>
#include <iterator>

#include <SDL.h>

struct context
{
    context(ps3eye::resolution res, int fps)
    {
        if (hasDevices())
        {
            eye = devices[0];
            running = eye->init(res, fps);
        }
        else
            running = false;
    }

    bool hasDevices() { return !devices.empty(); }

    std::shared_ptr<ps3eye::camera> eye;
    std::vector<std::shared_ptr<ps3eye::camera>> devices { ps3eye::list_devices() };

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

static void run_camera(ps3eye::resolution res, int fps, Uint32 duration)
{
    context ctx(res, fps);
    ctx.running &= ctx.eye->start();

    if (!ctx.running)
    {
        fprintf(stderr, "No PS3 Eye camera connected\n");
        return;
    }

    //ctx.eye->set_flip_status(true); /* mirrored left-right */

    char title[256];
    sprintf(title, "%dx%d@%d\n", ctx.eye->width(), ctx.eye->height(), ctx.eye->framerate());

    SDL_Window* window = SDL_CreateWindow(title,
                                          SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED,
                                          ctx.eye->width(), ctx.eye->height(), 0);
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

    ctx.running &= ctx.eye->start();

    fprintf(stderr, "camera mode: %dx%d@%d\n", ctx.eye->width(), ctx.eye->height(), ctx.eye->framerate());

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
    ps3eye::resolution res = ps3eye::res_SVGA;
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
                      << " [--fps XX] [--qvga]" << std::endl;
            return 64 /* EX_USAGE */;
        }
    }

    if (SDL_Init(SDL_INIT_VIDEO) < 0)
    {
        printf("Failed to initialize SDL: %s\n", SDL_GetError());
        return EXIT_FAILURE;
    }

    run_camera(res, fps, 0);

    return EXIT_SUCCESS;
}

