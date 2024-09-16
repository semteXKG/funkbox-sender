#pragma once

#include <stdint.h>

enum severity {
    POSITIVE,
    NORMAL,
    WARN,
    CRIT
};

enum event_type {
    EVT_NONE,
    EVT_LAP,
    EVT_TIME_REMAIN,
    EVT_STATE_CHANGE
};

struct  event {
    int id;
    enum event_type type;
    enum severity severity;
    int64_t created_at;
    int64_t displayed_since;
    char text[100];
};

enum command_type {
    COM_NONE,
    COM_PIT,
    COM_STINT_OVER,
    COM_TBD
};

struct command {
    enum command_type type;
    int64_t created;
    int64_t handled;
};

struct car_sensor {
    int temp;
    double preassure;
};

struct stint_data {
    bool running;
    bool enabled;
    int64_t target;
    int64_t elapsed;
};

struct lap
{
    int lap_no;
    int32_t lap_time_ms;
};

struct lap_data {
    int lap_no;
    int32_t best_lap;
    int32_t current_lap;
    struct lap last_laps[5];
};

struct mcu_data {
    long network_time_adjustment;
    struct car_sensor water;
    struct car_sensor oil;
    struct car_sensor gas;
    struct stint_data stint;
    struct lap_data lap_data;  
    struct event events[5];
    int events_cnt;
    struct command outgoing_commands[5];
    int outgoing_commands_last_idx;
    struct command incoming_commands[5];
    int incoming_commands_last_idx;
};

struct time_str {
    int milliseconds;
    int seconds;
    int minutes;
};

struct ack_msg {
    int seq_nr;
};

enum lora_type {
    LORA_ACK, LORA_OIL, LORA_WATER, LORA_GAS, LORA_LAP, LORA_STINT, LORA_COMMAND
};

struct payloaded_message {
    enum lora_type type;
    uint16_t seq_nr;
    bool remove_older_versions;
    bool requires_ack;
    uint32_t timestamp;

    char payload[52];

    struct car_sensor* car_sensor_data;
    struct lap_data* lap_data;
    struct stint_data* stint_data;
    struct ack_msg* ack_msg;
};