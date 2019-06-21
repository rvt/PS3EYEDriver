add_definitions(-DNOMINMAX)
add_definitions(-DWIN32_LEAN_AND_MEAN)
add_definitions(-D_CRT_SECURE_NO_WARNINGS)

if(NOT WIN32)
    add_definitions(-pthread)
    link_libraries(-pthread)
endif()

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")

if(NOT MSVC)
    if(PKG_CONFIG_FOUND)
        pkg_check_modules(libusb "libusb-1.0" QUIET)
    endif()
    if(libusb_FOUND)
        include_directories(${libusb_INCLUDE_DIRS})
        link_libraries(${libusb_LIBRARIES})
    endif()
else()
    set(SDK_LIBUSB CACHE PATH "")
    if(SDK_LIBUSB)
        set(libusb_FOUND TRUE)
        link_directories("${SDK_LIBUSB}")
        include_directories("${SDK_LIBUSB}")
    endif()
endif()

if(libusb_FOUND)
    add_library(ps3eye-driver STATIC
        "ps3eye.cpp"
        "urb.cpp"
        "queue.cpp"
        "mgr.cpp"
        "accessors.cpp"
    )
endif()

add_subdirectory("sdl")
