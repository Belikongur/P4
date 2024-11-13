#include "crane.h"

#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <lownet.h>
#include <serial_io.h>
#include <string.h>
#include <utility.h>

#define CRANE_PROTO 0x05

#define TAG "crane"

void crane_connect(uint8_t id);
void crane_disconnect();
int crane_action(uint8_t action);  // returns zero if ACK is received
void crane_test(uint8_t id);
void crane_receive(const lownet_frame_t* frame);
void crane_send(uint8_t destination, const crane_packet_t* packet);
void mil1_test();

// state of a single flow
static struct
{
    uint16_t seq;
    uint8_t crane;
    QueueHandle_t acks;
    enum {
        ST_DISCONNECTED,
        ST_HANDSHAKE,
        ST_CONNECTED,
    } state;
} state;

int crane_init(void) {
    if (lownet_register_protocol(CRANE_PROTO, crane_receive) != 0) {
        ESP_LOGE(TAG, "Failed to register crane protocol");
        return 1;
    }

    state.seq = 0;
    state.crane = 0;
    state.state = ST_DISCONNECTED;
    state.acks = xQueueCreate(8, sizeof(uint16_t));
    return 0;
}

void crane_command(char* args) {
    if (!args) {
        serial_write_line("Missing argument COMMAND");
        return;
    }
    char* saveptr;

    char* command = strtok_r(args, " ", &saveptr);
    if (!command) {
        serial_write_line("Missing argument COMMAND");
        return;
    }

    if (strcmp(command, "open") == 0) {
        char* id = strtok_r(NULL, " ", &saveptr);
        if (!id) {
            serial_write_line("Missing argument ID");
            return;
        }
        uint8_t dest = hex_to_dec(id + 2);
        crane_connect(dest);
    } else if (strcmp(command, "close") == 0) {
        crane_disconnect();
    } else if (strcmp(command, "t") == 0) {
        mil1_test();
    } else if (strcmp(command, "test") == 0) {
        char* id = strtok_r(NULL, " ", &saveptr);
        if (!id) {
            serial_write_line("Missing argument ID");
            return;
        }
        uint8_t dest = hex_to_dec(id + 2);
        crane_test(dest);
    } else {
        uint8_t action = CRANE_NULL;
        switch (command[0]) {
                // ------------------------------------------------
                // Milestone II, Task 2: implement commands to CLI
            case 'f':  // FORWARD, call crane_action with appropriate action!
                // your code goes here!
                break;
            // repeat the above for all commands!
            // ------------------------------------------------
            default:
                ESP_LOGI(TAG, "Invalid crane command");
                return;
        }
        crane_action(action);
    }
}

uint32_t reverse_bits(uint32_t n) {
    uint32_t reversed = 0;
    for (int i = 0; i < 32; i++) {
        reversed <<= 1;
        reversed |= (n & 1);
        n >>= 1;
    }
    return reversed;
}

void crane_recv_connect(const crane_packet_t* packet) {
    ESP_LOGI(TAG, "Received CONNECT packet");
    if (state.state != ST_HANDSHAKE)
        return;

    ESP_LOGI(TAG, "packet flags: %02x", packet->flags);

    // ------------------------------------------------
    // Milestone I: check that both SYN and ACK flags are set?
    // - if not, return
    if (packet->flags != 3) {
        serial_write_line("flags SYN and ACK are not set\n");
        return;
    }

    // Milestone I: respond with appropriate (ack) packet + challenge! (cf. Task 2)
    uint32_t challenge = reverse_bits(packet->d.conn.challenge);
    crane_packet_t outpkt = {
        // your code goes here!
        .type = CRANE_CONNECT,
        .flags = CRANE_ACK,
        .seq = 0,
        .d.conn.challenge = challenge};
    state.state = ST_CONNECTED;
    crane_send(state.crane, &outpkt);
    // ------------------------------------------------
}

void crane_recv_close(const crane_packet_t* packet) {
    ESP_LOGI(TAG, "Closing connection");
    state.seq = 0;
    state.state = ST_DISCONNECTED;
    state.crane = 0;
}

void crane_recv_status(const crane_packet_t* packet) {
    char buffer[200];

    if (packet->flags & CRANE_NAK)  // crane missed some packet
    {
        // Not in use yet -- anywhere, so you can ignore
        ESP_LOGI(TAG, "Received status packet with NAK -- not in use yet");
        return;
    }  // else  // push the cumulative ack to ACK-queue
       //  xQueueSend(ack_queue, packet->seq, 0);

    snprintf(buffer, sizeof buffer,
             "backlog: %d\n"
             "time: %d\n"
             "light: %s\n",
             packet->d.status.backlog,
             packet->d.status.time_left,
             packet->d.status.light ? "on" : "off");
    serial_write_line(buffer);
}

void crane_receive(const lownet_frame_t* frame) {
    crane_packet_t packet;
    memcpy(&packet, frame->payload, sizeof packet);
    ESP_LOGI(TAG, "Received packet frame from %02x, type: %d", frame->source, packet.type);
    switch (packet.type) {
        case CRANE_CONNECT:
            crane_recv_connect(&packet);
            break;
        case CRANE_STATUS:
            crane_recv_status(&packet);
            break;
        case CRANE_ACTION:
            break;
        case CRANE_CLOSE:
            crane_recv_close(&packet);
    }
}

/*
 * This function starts the connection establishment
 * procedure by sending a SYN packet to the given node.
 */
void crane_connect(uint8_t id) {
    if (state.state != ST_DISCONNECTED)
        return;

    crane_packet_t packet;
    packet.type = CRANE_CONNECT;
    packet.flags = CRANE_SYN;
    packet.seq = state.seq;

    state.crane = id;
    state.state = ST_HANDSHAKE;
    ++state.seq;

    crane_send(id, &packet);
}

void crane_disconnect(void) {
    crane_packet_t packet;

    // ------------------------------------------------
    // Milestone I, Task 3: construct a CLOSE packet and
    // send it to the (right) crane!
    packet.type = CRANE_CLOSE;
    packet.d.close = 0;
    // ------------------------------------------------
    // Then we update our state
    state.state = ST_DISCONNECTED;
    state.seq = 0;
    crane_send(state.crane, packet);
}

/*
 *	Subroutine for crane_action: read ACKs from crane, blocks for some time
 */
uint16_t read_acks(void) {
    uint16_t seq, x;

    // Wait for an ack up to 5 seconds
    if (xQueueReceive(state.acks, &seq, 5000 / portTICK_PERIOD_MS) != pdTRUE)
        seq = 0;
    // read any other acks if in the queue
    while (xQueueReceive(state.acks, &x, 0) == pdTRUE)
        seq = seq >= x ? seq : x;
    return seq;
}

/*
 *	This can block for a while if no immediate ACK
 */
int crane_action(uint8_t action) {
    crane_packet_t packet;

    packet.type = CRANE_ACTION;
    packet.seq = state.seq;
    packet.d.action.cmd = action;

    crane_send(state.crane, &packet);

    // ------------------------------------------------
    // Milestone II, Task 1: run the following up to five times
    // Your code goes here
    {
        uint16_t seq = read_acks();  // seq is the cumulative ack from crane, or zero if none

        // If seq > state.seq, report an error and close the connection, return -2
        // - Else if seq == state.seq, we are good, increment state.seq by one (why?!) and return 0
        // - Otherwise retransmit
    }
    // ------------------------------------------------

    // No ack received, disconnect
    ESP_LOGI(TAG, "Received no ack from node=0x%02x", state.crane);
    crane_disconnect();
    return -1;
}

// ------------------------------------------------
//
// Milestone III: run the test pattern
//
// 1. establish connection with TEST flag
// 2. run the test pattern according to the specs
// 3. close the connection
//
// Hint: you can work it all out here slowly, or be a wizard
//       and launch a separate task for this!
//
void crane_test(uint8_t id) {
}
// ------------------------------------------------

void crane_send(uint8_t id, const crane_packet_t* packet) {
    lownet_frame_t frame;
    frame.destination = id;
    frame.protocol = CRANE_PROTO;
    frame.length = sizeof *packet;
    memcpy(frame.payload, packet, sizeof *packet);

    lownet_send(&frame);
}

void mil1_test() {
    if (state.state != ST_DISCONNECTED)
        return;
    crane_packet_t packet = {0};
    packet.type = CRANE_CONNECT;
    packet.flags = (CRANE_SYN | CRANE_ACK);
    packet.seq = state.seq;
    packet.d.conn.challenge = 64;

    state.crane = 0xff;
    state.state = ST_HANDSHAKE;
    ++state.seq;
    crane_recv_connect(&packet);
}