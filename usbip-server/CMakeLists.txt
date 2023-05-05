# USBIP server library

add_library(usbip-server STATIC usbip.c)
target_include_directories(usbip-server PUBLIC ${CMAKE_CURRENT_LIST_DIR})
