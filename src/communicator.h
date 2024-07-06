#pragma once
#include "messages.h"
#include <heltec_unofficial.h>

#define MAX_ITEMS 32
#define MAX_READ_BYTES 100

void comm_setup();

void comm_enqueue_message(struct payloaded_message message);

void comm_loop();