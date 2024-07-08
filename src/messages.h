#pragma once

#include <stdint.h>

struct payloaded_message {
    char command[10];
    uint16_t seq_nr = 0;
    char payload[100];
    long last_sent = 0;
};