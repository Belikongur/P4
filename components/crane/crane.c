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

void print_crane_packet(const crane_packet_t* packet) {
    if (!packet) {
        printf("Invalid packet pointer\n");
        return;
    }

    printf("Crane Packet:\n");
    printf("  Type: 0x%02X\n", packet->type);
    printf("  Flags: 0x%02X\n", packet->flags);
    printf("  Sequence: %u\n", packet->seq);

    switch (packet->type) {
        case 0x01:  // Connection Packet
            printf("  Connection Data:\n");
            printf("    Challenge: 0x%08lX\n", packet->d.conn.challenge);
            break;

        case 0x02:  // Action Packet
            printf("  Action Data:\n");
            printf("    Command: 0x%02X\n", packet->d.action.cmd);
            printf("    Reserved: {0x%02X, 0x%02X, 0x%02X}\n",
                   packet->d.action.reserved[0],
                   packet->d.action.reserved[1],
                   packet->d.action.reserved[2]);
            break;

        case 0x03:  // Status Packet
            printf("  Status Data:\n");
            printf("    Backlog: %u\n", packet->d.status.backlog);
            printf("    Time Left: %u seconds\n", packet->d.status.time_left);
            printf("    Light Status: %s\n", packet->d.status.light ? "ON" : "OFF");
            printf("    Temperature: %d°C\n", packet->d.status.temp);
            break;

        case 0x04:  // Close Packet
            printf("  Close Data: 0x%08lX\n", packet->d.close);
            break;

        default:
            printf("  Unknown type\n");
    }
}

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
        uint8_t action;
        switch (command[0]) {
                // ------------------------------------------------
                // Milestone II, Task 2: implement commands to CLI
                // FORWARD, call crane_action with appropriate action!
                // your code goes here!
            case 'f':
                action = CRANE_FWD;
                break;
            case 'r':
                action = CRANE_REV;
                break;
            case 'u':
                action = CRANE_UP;
                break;
            case 'd':
                action = CRANE_DOWN;
                break;
            case 'o':
                action = CRANE_LIGHT_ON;
                break;
            case 'l':
                action = CRANE_LIGHT_OFF;
                break;
            case 'n':
                action = CRANE_NULL;
                break;
            case 's':
                action = CRANE_STOP;
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

/*
 * Input uint32_t n and returns a uint32_t with
 * all bits of n reversed (XOR)
 */
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
    if ((packet->flags & (CRANE_SYN | CRANE_ACK)) != (CRANE_SYN | CRANE_ACK)) {
        ESP_LOGI(TAG, "Flags SYN and ACK are not both set");
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
    print_crane_packet(&outpkt);
    crane_send(state.crane, &outpkt);
    // ------------------------------------------------
}

void crane_recv_close(const crane_packet_t* packet) {
    ESP_LOGI(TAG, "Closing connection");
    state.seq = 0;
    state.state = ST_DISCONNECTED;
    state.crane = 0;
}

uint8_t commands_executed = 0;
void crane_recv_status(const crane_packet_t* packet) {
    char buffer[200];

    if (packet->flags & CRANE_NAK)  // crane missed some packet
    {
        // Not in use yet -- anywhere, so you can ignore
        ESP_LOGI(TAG, "Received status packet with NAK -- not in use yet");
        return;
    } else  // push the cumulative ack to ACK-queue
        xQueueSend(state.acks, &packet->seq, 0);

    snprintf(buffer, sizeof buffer,
             "backlog: %d\n"
             "time: %d\n"
             "light: %s\n",
             packet->d.status.backlog,
             packet->d.status.time_left,
             packet->d.status.light ? "on" : "off");
    serial_write_line(buffer);

    if (commands_executed < packet->d.status.backlog) commands_executed = packet->d.status.backlog;
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
    packet.flags = CRANE_SYN;
    packet.d.close = 0;
    print_crane_packet(&packet);
    // ------------------------------------------------
    // Then we update our state
    state.state = ST_DISCONNECTED;
    state.seq = 0;
    crane_send(state.crane, &packet);
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
    for (int i = 0; i < 5; i++) {
        uint16_t seq = read_acks();  // seq is the cumulative ack from crane, or zero if none

        // If seq > state.seq, report an error and close the connection, return -2
        // - Else if seq == state.seq, we are good, increment state.seq by one (why?!) and return 0
        // - Otherwise retransmit
        if (seq > state.seq) {
            ESP_LOGE(TAG, "Crane sequence number %d exceeds expected sequence %d", seq, state.seq);
            crane_disconnect();
            return -2;
        } else if (seq == state.seq) {
            state.seq++;
            return 0;
        } else {
            crane_send(state.crane, &packet);
        }
    }
    // ------------------------------------------------

    // No ack received, disconnect
    ESP_LOGI(TAG, "Received no ack from node=0x%02x", state.crane);
    crane_disconnect();
    return -1;
}

int await_execution() {
    int retries = 0;
    while (commands_executed < state.seq) {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        if (++retries > 5) {
            return -1;
        }
    }
    return 0;
}

void test_action(uint8_t action) {
    crane_packet_t packet;

    packet.type = CRANE_ACTION;
    packet.seq = state.seq;
    packet.d.action.cmd = action;
    // Test action crane packet
    print_crane_packet(&packet);
    crane_send(state.crane, &packet);
    for (int i = 0; i < 8; i++) {
        uint16_t seq, x;

        // Wait for an ack up to 2 seconds
        if (xQueueReceive(state.acks, &seq, 2000 / portTICK_PERIOD_MS) != pdTRUE)
            seq = 0;
        // read any other acks if in the queue
        while (xQueueReceive(state.acks, &x, 0) == pdTRUE)
            seq = seq >= x ? seq : x;

        if (seq > state.seq) {
            ESP_LOGE(TAG, "Crane sequence number %d exceeds expected sequence %d", seq, state.seq);
            crane_disconnect();
            return;
        } else if (seq == state.seq) {
            state.seq++;
            return;
        } else {
            crane_send(state.crane, &packet);
        }
    }
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
    // Establish connection
    if (state.state != ST_DISCONNECTED)
        return;

    crane_packet_t testpacket;
    testpacket.type = CRANE_CONNECT;
    testpacket.flags = CRANE_TEST;
    testpacket.seq = state.seq;

    state.crane = id;
    state.state = ST_HANDSHAKE;
    ++state.seq;

    crane_send(id, &testpacket);

    // Wait for handshake to finish
    int retries = 0;
    while (state.state != ST_HANDSHAKE) {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        if (++retries > 5) {
            ESP_LOGI(TAG, "Handshake timed out");
            crane_disconnect();
            return;
        }
    }

    // Run test pattern
    // first tests
    test_action(CRANE_LIGHT_ON);
    test_action(CRANE_FWD);
    test_action(CRANE_FWD);
    test_action(CRANE_REV);
    // Pause until all commands have executed
    if (await_execution()) {
        ESP_LOGE(TAG, "Command execution timed out");
        crane_disconnect();
        return;
    }
    // second batch of test
    test_action(CRANE_DOWN);
    test_action(CRANE_DOWN);
    // Pause again
    if (await_execution()) {
        ESP_LOGE(TAG, "Command execution timed out");
        crane_disconnect();
        return;
    }
    // final batch of tests
    test_action(CRANE_UP);
    test_action(CRANE_UP);
    test_action(CRANE_REV);
    test_action(CRANE_LIGHT_OFF);
    // Disconnect
    crane_disconnect();
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