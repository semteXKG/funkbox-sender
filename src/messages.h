#pragma once

#include <stdint.h>

struct payloaded_message {
    char command[10];
    uint16_t seq_nr = 0;
    char payload[100];
    uint64_t last_sent;
};