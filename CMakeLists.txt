set(CMAKE_CXX_VISIBILITY_PRESET "hidden")
list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake/")

add_definitions(-DNOMINMAX)
add_definitions(-D_CRT_SECURE_NO_WARNINGS)
add_definitions(-DPS3EYE_DEBUG)

if(WIN32)
    add_definitions(-DWIN32_LEAN_AND_MEAN)
    set(SDK_LIBUSB CACHE PATH "")
    if(SDK_LIBUSB)
        set(libusb_FOUND TRUE)
        link_directories("${SDK_LIBUSB}")
        include_directories(SYSTEM "${SDK_LIBUSB}")
    endif()
else()
    add_definitions(-pthread)
    find_package(LibUSB REQUIRED)
    include_directories(SYSTEM ${LibUSB_INCLUDE_DIRS})
    # /usr/local/Cellar/libusb/1.0.23/include/libusb-1.0 : /usr/local/Cellar/libusb/1.0.23/lib/libusb-1.0.dylib
    link_libraries(${LibUSB_LIBRARIES} -pthread)
endif()

if(libusb_FOUND OR LibUSB_FOUND)
    add_library(ps3eye-driver STATIC
        "ps3eye.cpp"
        "urb.cpp"
        "queue.cpp"
        "mgr.cpp"
        "accessors.cpp"
    )
endif()

if(TARGET ps3eye-driver)
    list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")
    find_package(SDL2 QUIET)

    if(SDL2_FOUND)
        include_directories(SYSTEM ${SDL2_INCLUDE_DIRS})
        link_libraries(${SDL2_LIBRARIES})
        link_libraries(ps3eye-driver)
        add_executable(ps3eye-sdl "sdl.cxx")
    endif()

    add_executable(ps3eye-mode-test "mode-test.cxx")
    target_link_libraries(ps3eye-mode-test ps3eye-driver)

    add_executable(ps3eye-frame-test "frame-test.cxx")
    target_link_libraries(ps3eye-frame-test ps3eye-driver)
endif()
