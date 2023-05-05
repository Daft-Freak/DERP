# USBIP server library

add_library(usbip-server STATIC usbip.c)
target_include_directories(usbip-server PUBLIC ${CMAKE_CURRENT_LIST_DIR})

if(WIN32)
    target_link_libraries(usbip-server ws2_32)
endif()