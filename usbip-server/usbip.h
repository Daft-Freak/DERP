#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

struct timeval;

enum usbip_result {
    usbip_success = 0,
    usbip_disconnected,
    usbip_error,
    usbip_error_socket,
    usbip_error_unknown_command,
};

enum usbip_device_speed {
    usbip_speed_low = 1,
    usbip_speed_full,
    usbip_speed_high,
    usbip_speed_wireless,
    usbip_speed_super,
    usbip_speed_super_plus,
};

struct usbip_client;
struct usbip_server;

struct usbip_device
{
    const uint8_t *device_descriptor;
    const uint8_t *config_descriptor;
    enum usbip_device_speed speed;
    int num;
    void *user_data;

    bool (*get_descriptor)(struct usbip_client *client, uint32_t seqnum, uint8_t desc_type, uint8_t desc_index, uint16_t setup_index, uint16_t setup_length, void *user_data);
    bool (*control_request)(struct usbip_client *client, uint32_t seqnum, uint8_t request_type, uint8_t request, uint16_t value, uint16_t index, uint16_t length, const uint8_t *out_data, void *user_data);
    bool (*in)(struct usbip_client *client, uint32_t seqnum, int ep, uint32_t length, void *user_data);
    bool (*out)(struct usbip_client *client, uint32_t seqnum, int ep, uint32_t length, const uint8_t *data, void *user_data);
};

void usbip_add_device(struct usbip_device *device);
void usbip_remove_device(struct usbip_device *device);

struct usbip_client *usbip_create_client(int sosket_fd);
void usbip_destroy_client(struct usbip_client *client);
enum usbip_result usbip_client_recv(struct usbip_client *client);
enum usbip_result usbip_client_reply(struct usbip_client *client, uint32_t seqnum, const uint8_t *data, size_t data_len);
enum usbip_result usbip_client_stall(struct usbip_client *client, uint32_t seqnum);

enum usbip_result usbip_create_server(struct usbip_server **server, const char *bind_addr, uint16_t port);
void usbip_destroy_server(struct usbip_server *server);
enum usbip_result usbip_server_update(struct usbip_server *server, struct timeval *timeout);

#ifdef __cplusplus
}
#endif