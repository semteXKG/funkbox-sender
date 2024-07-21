#pragma once

#include <stdint.h>

struct payloaded_message {
    char* command;
    uint16_t seq_nr = 0;
    char* payload;
    long last_sent = 0;
};