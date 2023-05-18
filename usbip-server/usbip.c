#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#if defined(_WIN32)
#include <Ws2tcpip.h>

typedef int ssize_t; // used for send result
#else
#include <arpa/inet.h>
#include <netdb.h>
#include <netinet/tcp.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <unistd.h>
#endif

#include "usbip.h"
#include "protocol.h"

// #define USBIP_DEBUG

#ifdef USBIP_DEBUG
#define debug_printf printf
#else
#define debug_printf(...)
#endif

// internal structs
struct usbip_client
{
    int fd;
    struct usbip_device *imported_device;
    struct usbip_client *next; // for server
};

struct usbip_server
{
    int fd;
    fd_set sock_set;
    int max_set_fd;

    struct usbip_client *clients;
};

struct device_list_entry
{
    struct usbip_device *entry;
    struct device_list_entry *next;
};

// device list
static struct device_list_entry *device_list = NULL;
static int num_devices = 0;
static int next_dev_num = 1;

// socket wrappers
static int close_socket(int fd)
{
#ifdef _WIN32
    return closesocket(fd);
#else
    return close(fd);
#endif
}

static bool send_all(int fd, const void *data, size_t *len, int flags)
{
    size_t total_sent = 0;
    size_t to_send = *len;
    ssize_t sent = 0;

    while(to_send)
    {
        sent = send(fd, (char *)data + total_sent, to_send, flags);
        if(sent == -1)
            break;
        total_sent += sent;
        to_send -= sent;
    }

    *len = total_sent;
    return sent != -1;
}

static ssize_t recv_all(int fd, void *data, size_t len, int flags)
{
    return recv(fd, data, len, flags | MSG_WAITALL);
}

// device
void usbip_add_device(struct usbip_device *device)
{
    device->num = next_dev_num++;

    struct device_list_entry *new_entry = malloc(sizeof(struct device_list_entry));
    new_entry->entry = device;
    new_entry->next = NULL;

    if(!device_list)
        device_list = new_entry;
    else
    {
        struct device_list_entry *tail = device_list;
        while(tail->next)
            tail = tail->next;

        tail->next = new_entry;
    }

    num_devices++;
}

void usbip_remove_device(struct usbip_device *device)
{
    struct device_list_entry *prev = NULL, *entry;

    // find it
    for(entry = device_list; entry; prev = entry, entry = entry->next)
    {
        if(entry->entry == device)
            break;
    }

    if(!entry)
        return;

    if(prev)
        prev->next = entry->next;
    else // is head
        device_list = entry->next;

    free(entry);

    num_devices--;
}

// client
struct usbip_client *usbip_create_client(int fd)
{
    struct usbip_client *ret = malloc(sizeof(struct usbip_client));

    ret->fd = fd;
    ret->imported_device = NULL;
    ret->next = NULL;

    return ret;
}

void usbip_destroy_client(struct usbip_client *client)
{
    free(client);
}

enum usbip_result usbip_client_reply(struct usbip_client *client, uint32_t seqnum, const uint8_t *data, size_t data_len)
{
    struct usbip_header_basic reply_head = {0};

    reply_head.command = htonl(3);
    reply_head.seqnum = htonl(seqnum);

    size_t len = sizeof(reply_head);
    if(!send_all(client->fd, &reply_head, &len, 0) || len < sizeof(reply_head))
        return usbip_error_socket;

    struct ret_submit reply = {0};

    reply.actual_length = htonl(data_len);

    len = sizeof(reply);
    if(!send_all(client->fd, &reply, &len, 0) || len < sizeof(reply))
        return usbip_error_socket;

    // send data
    if(data && data_len)
    {
        len = data_len;
        if(!send_all(client->fd, data, &len, 0) || len < data_len)
            return usbip_error_socket;
    }

    return usbip_success;
}

enum usbip_result usbip_client_stall(struct usbip_client *client, uint32_t seqnum)
{
    struct usbip_header_basic reply_head = {0};

    reply_head.command = htonl(3);
    reply_head.seqnum = htonl(seqnum);

    size_t len = sizeof(reply_head);
    if(!send_all(client->fd, &reply_head, &len, 0) || len < sizeof(reply_head))
        return usbip_error_socket;

    struct ret_submit reply = {0};

    reply.status = htonl(-32); // EPIPE

    len = sizeof(reply);
    if(!send_all(client->fd, &reply, &len, 0) || len < sizeof(reply))
        return usbip_error_socket;

    return usbip_success;
}

static void fill_dev_reply(const struct usbip_device *dev, struct devlist_reply_device *reply_dev)
{
    snprintf(reply_dev->path, 255, "/sys/devices/pci0000:00/0000:00:00.1/usb1/1-%i", dev->num);
    snprintf(reply_dev->busid, 31, "1-%i", dev->num);
    reply_dev->busnum = htonl(1);
    reply_dev->devnum = htonl(dev->num);
    reply_dev->speed = htonl(dev->speed);

    reply_dev->idVendor = htons(dev->device_descriptor[8] | dev->device_descriptor[9] << 8);
    reply_dev->idProduct = htons(dev->device_descriptor[10] | dev->device_descriptor[11] << 8);
    reply_dev->bcdDevice = htons(dev->device_descriptor[12] | dev->device_descriptor[13] << 8);
    reply_dev->bDeviceClass = dev->device_descriptor[4];
    reply_dev->bDeviceSubClass = dev->device_descriptor[5];
    reply_dev->bDeviceProtocol = dev->device_descriptor[6];
    reply_dev->bConfigurationValue = dev->config_descriptor[5];
    reply_dev->bNumConfigurations = dev->device_descriptor[17];
    reply_dev->bNumInterfaces = dev->config_descriptor[4];
}

static enum usbip_result send_devlist(int fd)
{
    uint8_t buf[4];
    ssize_t received = recv_all(fd, buf, 4, 0); // status
    if(received != 4)
        return usbip_error_socket;

    struct devlist_reply_header reply_head;

    reply_head.version = htons(usbip_version);
    reply_head.code = htons(0x0005);
    reply_head.status = 0;
    reply_head.num_devices = htonl(num_devices);

    size_t len = sizeof(reply_head);
    if(!send_all(fd, &reply_head, &len, 0) || len != sizeof(reply_head))
        return usbip_error_socket;


    struct device_list_entry *entry;
    for(entry = device_list; entry; entry = entry->next)
    {
        struct devlist_reply_device reply_dev = {0};

        struct usbip_device *dev = entry->entry;

        fill_dev_reply(dev, &reply_dev);

        len = sizeof(reply_dev);
        if(!send_all(fd, &reply_dev, &len, 0) || len != sizeof(reply_dev))
            return usbip_error_socket;

        uint16_t cfg_desc_len = dev->config_descriptor[2] | dev->config_descriptor[3] << 8;

        // find interfaces
        const uint8_t *cfg_desc = dev->config_descriptor;
        for(int off = 9; off < cfg_desc_len; off += cfg_desc[off])
        {
            uint8_t type = cfg_desc[off + 1];

            if(type == 4) // interface
            {
                struct devlist_reply_interface reply_intf = {0};
                reply_intf.bInterfaceClass = cfg_desc[off + 5];
                reply_intf.bInterfaceSubClass = cfg_desc[off + 6];
                reply_intf.bInterfaceProtocol = cfg_desc[off + 7];

                len = sizeof(reply_intf);
                if(!send_all(fd, &reply_intf, &len, 0) || len != sizeof(reply_intf))
                    return usbip_error_socket;
            }
        }
    }

    return usbip_success;
}

static enum usbip_result handle_import(struct usbip_client *client)
{
    uint8_t buf[36];
    ssize_t received = recv_all(client->fd, buf, 36, 0); // status + busid
    if(received != 36)
        return usbip_error_socket;

    // find device
    char *busid = (char *)(buf + 4);

    int bus, dev;
    if(sscanf(busid, "%i-%i", &bus, &dev) == 2 && bus == 1)
    {
        struct device_list_entry *entry;
        for(entry = device_list; entry; entry = entry->next)
        {
            if(entry->entry->num == dev)
            {
                client->imported_device = entry->entry;
                break;
            }
        }
    }

    // send reply
    struct import_reply_header reply_head = {0};
    reply_head.version = htons(usbip_version);
    reply_head.code = htons(0x0003);

    if(client->imported_device == NULL)
    {
        // device not found
        reply_head.status = htonl(1); // error
        size_t len = sizeof(reply_head);
        if(!send_all(client->fd, &reply_head, &len, 0) || len != sizeof(reply_head))
            return usbip_error_socket;

        return true;
    }

    size_t len = sizeof(reply_head);
    if(!send_all(client->fd, &reply_head, &len, 0) || len != sizeof(reply_head))
        return usbip_error_socket;

    // same as list
    struct devlist_reply_device reply_dev = {0};

    fill_dev_reply(client->imported_device, &reply_dev);

    len = sizeof(reply_dev);
    if(!send_all(client->fd, &reply_dev, &len, 0) || len != sizeof(reply_dev))
        return usbip_error_socket;

    return usbip_success;
}

static enum usbip_result handle_submit(struct usbip_client *client, struct usbip_header_basic *head)
{
    struct usbip_device *dev = client->imported_device; 

    struct cmd_submit cmd_data;

    // recv command
    ssize_t received = recv_all(client->fd, &cmd_data, sizeof(cmd_data), 0);
    if(received != sizeof(cmd_data))
        return usbip_error_socket;

    cmd_data.transfer_flags = ntohl(cmd_data.transfer_flags);
    cmd_data.transfer_buffer_length = ntohl(cmd_data.transfer_buffer_length);
    cmd_data.start_frame = ntohl(cmd_data.start_frame);
    cmd_data.number_of_packets = ntohl(cmd_data.number_of_packets);
    cmd_data.interval = ntohl(cmd_data.interval);

    bool in = head->direction == 1;

    debug_printf("submit seq %i devid %x, dir %i, ep %i, flags %x, len %i, sf %i np %i int %i\n",
                 head->seqnum, head->devid, head->direction, head->ep,
                 cmd_data.transfer_flags, cmd_data.transfer_buffer_length, cmd_data.start_frame, cmd_data.number_of_packets, cmd_data.interval);

    // recv OUT data
    uint8_t *out_data = NULL;
    if(!in && cmd_data.transfer_buffer_length)
    {
        out_data = malloc(cmd_data.transfer_buffer_length);

        received = recv_all(client->fd, out_data, cmd_data.transfer_buffer_length, 0);
        if(received != cmd_data.transfer_buffer_length)
            return usbip_error_socket;
    }

    bool handled = false;
    
    if(head->ep == 0)
    {
        // control request
        uint8_t setup_req_type = cmd_data.setup[0];
        uint8_t setup_req = cmd_data.setup[1];

        uint16_t setup_value = cmd_data.setup[2] | cmd_data.setup[3] << 8;
        uint16_t setup_index = cmd_data.setup[4] | cmd_data.setup[5] << 8;
        uint16_t setup_len = cmd_data.setup[6] | cmd_data.setup[7] << 8;

        if(setup_req_type == 0x80 && setup_req == 6)
        {
            // get descriptor
            int desc_type = cmd_data.setup[3];
            int desc_index = cmd_data.setup[2];

            if(desc_type == 1) // device
            {
                uint8_t desc_len = dev->device_descriptor[0];
                size_t len = cmd_data.transfer_buffer_length < desc_len ? cmd_data.transfer_buffer_length : desc_len;

                return usbip_client_reply(client, head->seqnum, dev->device_descriptor, len);
            }
            else if(desc_type == 2) // config
            {
                // TODO: multiple configs
                uint16_t desc_len = dev->config_descriptor[2] | dev->config_descriptor[3] << 8;
                size_t len = cmd_data.transfer_buffer_length < desc_len ? cmd_data.transfer_buffer_length : desc_len;

                return usbip_client_reply(client, head->seqnum, dev->config_descriptor, len);
            }
            else // try tu use device callback for other descriptors
                handled = dev->get_descriptor && dev->get_descriptor(client, head->seqnum, desc_type, desc_index, setup_index, setup_len, dev->user_data);
        }
        else  // try tu use device callback for other control requests
            handled = dev->control_request && dev->control_request(client, head->seqnum, setup_req_type, setup_req, setup_value, setup_index, setup_len, out_data, dev->user_data);
    }
    else if(in)
        handled = dev->in && dev->in(client, head->seqnum, head->ep, cmd_data.transfer_buffer_length, dev->user_data);
    else
        handled = dev->out && dev->out(client, head->seqnum, head->ep, cmd_data.transfer_buffer_length, out_data, dev->user_data);

    if(out_data)
        free(out_data);

    if(handled)
        return usbip_success;

    // stall unhandled
    return usbip_client_stall(client, head->seqnum);
}

enum usbip_result usbip_client_recv(struct usbip_client *client)
{
    uint8_t buf[4];
    ssize_t received = recv_all(client->fd, buf, 4, 0);

    if(received == 0)
        return usbip_disconnected;
    else if(received == -1)
        return usbip_error_socket;
    else
    {
        uint16_t ver = buf[0] << 8 | buf[1];

        if(client->imported_device)
        {
            // get the rest of the header
            struct usbip_header_basic head;
            head.command = buf[0] << 24 | buf[1] << 16 | buf[2] << 8 | buf[3];

            received = recv_all(client->fd, ((uint8_t *)&head) + 4, sizeof(head) - 4, 0);
            if(received != sizeof(head) - 4)
                return usbip_error_socket;

            // swap
            head.seqnum = ntohl(head.seqnum);
            head.devid = ntohl(head.devid);
            head.direction = ntohl(head.direction);
            head.ep = ntohl(head.ep);

            switch(head.command)
            {
                case 1: // USBIP_CMD_SUBMIT
                    return handle_submit(client, &head);

                case 2: // USBIP_CMD_UNLINK
                {
                    struct cmd_unlink cmd_data;

                    received = recv_all(client->fd, &cmd_data, sizeof(cmd_data), 0);
                    if(received != sizeof(cmd_data))
                        return usbip_error_socket;

                    cmd_data.unlink_seqnum = ntohl(cmd_data.unlink_seqnum);

                    debug_printf("unlink seq %i devid %x, dir %i, ep %i, unlink seq %i\n",
                                 head.seqnum, head.devid, head.direction, head.ep,
                                 cmd_data.unlink_seqnum);

                    struct usbip_header_basic reply_head = {0};

                    reply_head.command = htonl(4);
                    reply_head.seqnum = htonl(head.seqnum);

                    size_t len = sizeof(reply_head);
                    if(!send_all(client->fd, &reply_head, &len, 0) || len < sizeof(reply_head))
                        return usbip_error_socket;

                    struct ret_unlink reply = {0};

                    struct usbip_device *dev = client->imported_device;
                    bool unlinked = !dev->unlink || dev->unlink(client, cmd_data.unlink_seqnum, dev->user_data);

                    if(unlinked)
                        reply.status = htonl(-104/*ECONNRESET*/);

                    len = sizeof(reply);
                    if(!send_all(client->fd, &reply, &len, 0) || len < sizeof(reply))
                        return usbip_error_socket;

                    break;
                }

                default:
                    debug_printf("cmd %x, seq %i devid %x, dir %i, ep %i\n", head.command, head.seqnum, head.devid, head.direction, head.ep);
                    return usbip_error_unknown_command;
            }

        }
        else if(ver == usbip_version)
        {
            uint16_t cmd = buf[2] << 8 | buf[3];

            switch(cmd)
            {
                case 0x8003: // OP_REQ_IMPORT
                    return handle_import(client);
                case 0x8005: // OP_REQ_DEVLIST
                    return send_devlist(client->fd);

                default:
                    debug_printf("cmd %x\n", cmd);
                    return usbip_error_unknown_command;
            }
        }
        else
        {
            debug_printf("ver %x\n", ver);
            return usbip_error_unknown_command;
        }
    }

    return usbip_success;
}

enum usbip_result usbip_create_server(struct usbip_server **server, const char *bind_addr, uint16_t port)
{
    if(!bind_addr)
        bind_addr = "::";

    if(!port)
        port = 3240;

    int sock_fd;

    if((sock_fd = socket(AF_INET6, SOCK_STREAM, 0)) == -1)
        return usbip_error_socket;
    
    int yes = 1;

    if(setsockopt(sock_fd, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(int)) == -1)
    {
        close_socket(sock_fd);
        return usbip_error_socket;
    }

    // allow IPv4 connections
    yes = 0; // no
    if(setsockopt(sock_fd, IPPROTO_IPV6, IPV6_V6ONLY, &yes, sizeof(int)) == -1)
    {
        close_socket(sock_fd);
        return usbip_error_socket;
    }

    // bind
    struct sockaddr_in6 addr = {0};
    addr.sin6_family = AF_INET6;
    addr.sin6_port = htons(port);

    if(inet_pton(AF_INET6, bind_addr, &addr.sin6_addr) != 1)
        return usbip_error;

    if(bind(sock_fd, (struct sockaddr *)&addr, sizeof(addr)) == -1)
    {
        close_socket(sock_fd);
        return usbip_error_socket;
    }

    if(listen(sock_fd, 1) == -1)
    {
        close_socket(sock_fd);
        return usbip_error_socket;
    }
    
    // alloc server
    struct usbip_server *new_server = malloc(sizeof(struct usbip_server));

    new_server->fd = sock_fd;

    // setup socket set
    FD_ZERO(&new_server->sock_set);
    FD_SET(sock_fd, &new_server->sock_set);
    new_server->max_set_fd = sock_fd;

    new_server->clients = NULL;

    *server = new_server;

    return usbip_success;
}

void usbip_destroy_server(struct usbip_server *server)
{
    struct usbip_client *client, *next;
    for(client = server->clients; client; client = next)
    {
        next = client->next;
        close_socket(client->fd);
        free(client);
    }

    close_socket(server->fd);

    free(server);
}

enum usbip_result usbip_server_update(struct usbip_server *server, struct timeval *timeout)
{
    fd_set set = server->sock_set;

    int ready = select(server->max_set_fd + 1, &set, NULL, NULL, timeout);

    bool rebuild_set = false;

    if(ready > 0)
    {
        if(FD_ISSET(server->fd, &set))
        {
            // accept
            struct sockaddr_storage remote_addr;
            socklen_t addr_len = sizeof(remote_addr);
            int fd = accept(server->fd, (struct sockaddr *)&remote_addr, &addr_len);

            if(fd == -1)
                return usbip_error_socket;
            else
            {
                char hoststr[NI_MAXHOST];
                char portstr[NI_MAXSERV];

                int rc = getnameinfo((struct sockaddr *)&remote_addr, addr_len, hoststr, sizeof(hoststr), portstr, sizeof(portstr), NI_NUMERICHOST | NI_NUMERICSERV);

                if(rc == 0)
                {
                    debug_printf("new connection from %s port %s\n", hoststr, portstr);
                }

                int yes = 1;
                setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &yes, sizeof(int));

                FD_SET(fd, &server->sock_set);
                
                if(fd > server->max_set_fd)
                    server->max_set_fd = fd;

                // create and add client to list
                struct usbip_client *new_client = usbip_create_client(fd);

                if(!server->clients)
                    server->clients = new_client;
                else
                {
                    struct usbip_client *tail = server->clients;
                    while(tail->next)
                        tail = tail->next;

                    tail->next = new_client;
                }
            }
        }

        struct usbip_client *client, *prev = NULL, *next;
        for(client = server->clients; client; prev = client, client = next)
        {
            next = client->next; // meight free client

            if(FD_ISSET(client->fd, &set))
            {
                enum usbip_result res = usbip_client_recv(client);

                if(res == usbip_disconnected || res == usbip_error_socket)
                {
                    // disconnected or error
                    debug_printf("client %s\n", res == usbip_disconnected ? "disconnected" : "error");

                    // remove from list
                    if(prev)
                        prev->next = client->next;
                    else
                        server->clients = client->next;

                    // close and destroy
                    close_socket(client->fd);
                    usbip_destroy_client(client);
                    rebuild_set = true;

                    client = prev; // keep old prev
                }
            }
        }
    }
    else if(ready == -1)
        return usbip_error_socket;
    else
        return usbip_not_ready;

    // rebuilt sosket set if someone disconnected
    if(rebuild_set)
    {
        // setup socket set
        FD_ZERO(&server->sock_set);
        FD_SET(server->fd, &server->sock_set);
        server->max_set_fd = server->fd;

        struct usbip_client *client;
        for(client = server->clients; client; client = client->next)
        {
            if(client->fd > server->max_set_fd)
                server->max_set_fd = client->fd;
        }
    }

    return usbip_success;
}