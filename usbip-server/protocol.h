#pragma once
#include <assert.h>
#include <stdint.h>

struct devlist_reply_header
{
    uint16_t version;
    uint16_t code;
    uint32_t status;
    uint32_t num_devices;
};
static_assert(sizeof(struct devlist_reply_header) == 12, "incorrect struct size");

struct devlist_reply_device
{
    char path[256];
    char busid[32];
    uint32_t busnum;
    uint32_t devnum;
    uint32_t speed;

    uint16_t idVendor;
    uint16_t idProduct;
    uint16_t bcdDevice;
    uint8_t bDeviceClass;
    uint8_t bDeviceSubClass;
    uint8_t bDeviceProtocol;
    uint8_t bConfigurationValue;
    uint8_t bNumConfigurations;
    uint8_t bNumInterfaces;
};
static_assert(sizeof(struct devlist_reply_device) == 312, "incorrect struct size");

struct devlist_reply_interface
{
    uint8_t bInterfaceClass;
    uint8_t bInterfaceSubClass;
    uint8_t bInterfaceProtocol;
    uint8_t padding;
};
static_assert(sizeof(struct devlist_reply_interface) == 4, "incorrect struct size");

struct import_reply_header
{
    uint16_t version;
    uint16_t code;
    uint32_t status;
};
static_assert(sizeof(struct import_reply_header) == 8, "incorrect struct size");

struct usbip_header_basic
{
    uint32_t command;
    uint32_t seqnum;
    uint32_t devid;
    uint32_t direction;
    uint32_t ep;
};
static_assert(sizeof(struct usbip_header_basic) == 20, "incorrect struct size");

struct cmd_submit
{
    uint32_t transfer_flags;
    uint32_t transfer_buffer_length;
    uint32_t start_frame;
    uint32_t number_of_packets;
    uint32_t interval;
    uint8_t setup[8];
};
static_assert(sizeof(struct cmd_submit) == 28, "incorrect struct size");

struct ret_submit
{
    uint32_t status;
    uint32_t actual_length;
    uint32_t start_frame;
    uint32_t number_of_packets;
    uint32_t error_count;
    uint8_t padding[8];
};
static_assert(sizeof(struct ret_submit) == 28, "incorrect struct size");

struct cmd_unlink
{
    uint32_t unlink_seqnum;
    uint8_t padding[24];
};
static_assert(sizeof(struct cmd_unlink) == 28, "incorrect struct size");

struct ret_unlink
{
    uint32_t status;
    uint8_t padding[24];
};
static_assert(sizeof(struct ret_unlink) == 28, "incorrect struct size");

static const uint16_t usbip_version = 0x0111;