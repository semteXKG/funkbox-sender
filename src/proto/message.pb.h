/* Automatically generated nanopb header */
/* Generated by nanopb-0.4.8 */

#ifndef PB_MESSAGE_PB_H_INCLUDED
#define PB_MESSAGE_PB_H_INCLUDED
#include <pb.h>

#if PB_PROTO_HEADER_VERSION != 40
#error Regenerate this file with the current version of nanopb generator.
#endif

/* Enum definitions */
/* https://github.com/protobuf-c/protobuf-c/wiki/Examples#strings */
typedef enum _Proto_Severity {
    Proto_Severity_POSITIVE = 1,
    Proto_Severity_NORMAL = 2,
    Proto_Severity_WARN = 3,
    Proto_Severity_CRIT = 4
} Proto_Severity;

typedef enum _Proto_Event_Type {
    Proto_Event_Type_EVT_NONE = 1,
    Proto_Event_Type_EVT_LAP = 2,
    Proto_Event_Type_EVT_TIME_REMAIN = 3,
    Proto_Event_Type_EVT_STATE_CHANGE = 4
} Proto_Event_Type;

typedef enum _Proto_Command_Type {
    Proto_Command_Type_COM_NONE = 1,
    Proto_Command_Type_COM_PIT = 2,
    Proto_Command_Type_COM_STINT_OVER = 3,
    Proto_Command_Type_COM_FCK = 4
} Proto_Command_Type;

typedef enum _Proto_Lora_Type {
    Proto_Lora_Type_LORA_ACK = 1,
    Proto_Lora_Type_LORA_OIL = 2,
    Proto_Lora_Type_LORA_WATER = 3,
    Proto_Lora_Type_LORA_GAS = 4,
    Proto_Lora_Type_LORA_LAP = 5,
    Proto_Lora_Type_LORA_STINT = 6,
    Proto_Lora_Type_LORA_COMMAND = 7
} Proto_Lora_Type;

/* Struct definitions */
typedef struct _Proto_Event {
    bool has_id;
    int32_t id;
    bool has_type;
    Proto_Event_Type type;
    bool has_severity;
    Proto_Severity severity;
    bool has_created_at;
    int64_t created_at;
    bool has_displayed_since;
    int64_t displayed_since;
    pb_callback_t text;
} Proto_Event;

typedef struct _Proto_Command {
    bool has_type;
    Proto_Command_Type type;
    bool has_id;
    int32_t id;
    bool has_created;
    int64_t created;
    bool has_handled;
    int64_t handled;
} Proto_Command;

typedef struct _Proto_Car_Sensor {
    bool has_temp;
    uint32_t temp;
    bool has_preassure;
    double preassure;
} Proto_Car_Sensor;

typedef struct _Proto_Stint_Data {
    bool has_running;
    bool running;
    bool has_enabled;
    bool enabled;
    bool has_target;
    uint32_t target;
    bool has_elapsed;
    uint32_t elapsed;
    bool has_elapsed_timestamp;
    uint32_t elapsed_timestamp;
} Proto_Stint_Data;

typedef struct _Proto_Lap {
    bool has_lap_no;
    int32_t lap_no;
    bool has_lap_time_ms;
    int64_t lap_time_ms;
} Proto_Lap;

typedef struct _Proto_Lap_Data {
    bool has_lap_no;
    int32_t lap_no;
    bool has_best_lap_ms;
    uint32_t best_lap_ms;
    bool has_current_lap_ms;
    uint32_t current_lap_ms;
    bool has_current_lap_snapshot_time;
    uint32_t current_lap_snapshot_time;
    pb_size_t laps_count;
    Proto_Lap laps[5];
} Proto_Lap_Data;

typedef struct _Proto_Gps_Data {
    bool has_spd;
    int32_t spd;
    bool has_lat;
    double lat;
    bool has_lon;
    double lon;
} Proto_Gps_Data;

typedef struct _Proto_Lora_Config {
    bool has_bandwidth;
    double bandwidth;
    bool has_spreading_factor;
    uint32_t spreading_factor;
    bool has_output_power;
    int32_t output_power;
} Proto_Lora_Config;

typedef struct _Proto_Mcu_Data {
    bool has_network_time_adjustment;
    uint32_t network_time_adjustment;
    bool has_send_timestamp;
    uint32_t send_timestamp;
    bool has_water;
    Proto_Car_Sensor water;
    bool has_oil;
    Proto_Car_Sensor oil;
    bool has_gas;
    Proto_Car_Sensor gas;
    bool has_stint;
    Proto_Stint_Data stint;
    bool has_lap_data;
    Proto_Lap_Data lap_data;
    pb_callback_t events;
    pb_callback_t outgoing_commands;
    pb_callback_t incoming_commands;
    bool has_gps;
    Proto_Gps_Data gps;
    bool has_gas_warn;
    Proto_Car_Sensor gas_warn;
    bool has_oil_warn;
    Proto_Car_Sensor oil_warn;
    bool has_lora_config;
    Proto_Lora_Config lora_config;
} Proto_Mcu_Data;

typedef struct _Proto_Update_Data {
    bool has_water_sensor;
    Proto_Car_Sensor water_sensor;
    bool has_oil_sensor;
    Proto_Car_Sensor oil_sensor;
    bool has_gas_sensor;
    Proto_Car_Sensor gas_sensor;
    bool has_lap_data;
    Proto_Lap_Data lap_data;
    bool has_stint_data;
    Proto_Stint_Data stint_data;
    bool has_gps_data;
    Proto_Gps_Data gps_data;
} Proto_Update_Data;

typedef struct _Proto_Ack_Data {
    bool has_seq_nr;
    uint32_t seq_nr;
} Proto_Ack_Data;

typedef struct _Proto_LoRa_Data {
    bool has_seq_nr;
    uint32_t seq_nr;
    bool has_requires_ack;
    bool requires_ack;
    bool has_send_timestamp;
    uint32_t send_timestamp;
    bool has_update_data;
    Proto_Update_Data update_data;
    bool has_command_data;
    Proto_Command command_data;
    bool has_ack_data;
    Proto_Ack_Data ack_data;
} Proto_LoRa_Data;

typedef struct _Proto_Message {
    bool has_mcu_data;
    Proto_Mcu_Data mcu_data;
    bool has_lora_data;
    Proto_LoRa_Data lora_data;
    bool has_command_data;
    Proto_Command command_data;
} Proto_Message;


#ifdef __cplusplus
extern "C" {
#endif

/* Helper constants for enums */
#define _Proto_Severity_MIN Proto_Severity_POSITIVE
#define _Proto_Severity_MAX Proto_Severity_CRIT
#define _Proto_Severity_ARRAYSIZE ((Proto_Severity)(Proto_Severity_CRIT+1))

#define _Proto_Event_Type_MIN Proto_Event_Type_EVT_NONE
#define _Proto_Event_Type_MAX Proto_Event_Type_EVT_STATE_CHANGE
#define _Proto_Event_Type_ARRAYSIZE ((Proto_Event_Type)(Proto_Event_Type_EVT_STATE_CHANGE+1))

#define _Proto_Command_Type_MIN Proto_Command_Type_COM_NONE
#define _Proto_Command_Type_MAX Proto_Command_Type_COM_FCK
#define _Proto_Command_Type_ARRAYSIZE ((Proto_Command_Type)(Proto_Command_Type_COM_FCK+1))

#define _Proto_Lora_Type_MIN Proto_Lora_Type_LORA_ACK
#define _Proto_Lora_Type_MAX Proto_Lora_Type_LORA_COMMAND
#define _Proto_Lora_Type_ARRAYSIZE ((Proto_Lora_Type)(Proto_Lora_Type_LORA_COMMAND+1))

#define Proto_Event_type_ENUMTYPE Proto_Event_Type
#define Proto_Event_severity_ENUMTYPE Proto_Severity

#define Proto_Command_type_ENUMTYPE Proto_Command_Type













/* Initializer values for message structs */
#define Proto_Event_init_default                 {false, 0, false, _Proto_Event_Type_MIN, false, _Proto_Severity_MIN, false, 0, false, 0, {{NULL}, NULL}}
#define Proto_Command_init_default               {false, _Proto_Command_Type_MIN, false, 0, false, 0, false, 0}
#define Proto_Car_Sensor_init_default            {false, 0u, false, 0}
#define Proto_Stint_Data_init_default            {false, 0, false, 0, false, 0, false, 0, false, 0}
#define Proto_Lap_init_default                   {false, 0, false, 0}
#define Proto_Lap_Data_init_default              {false, 0, false, 0, false, 0, false, 0, 0, {Proto_Lap_init_default, Proto_Lap_init_default, Proto_Lap_init_default, Proto_Lap_init_default, Proto_Lap_init_default}}
#define Proto_Gps_Data_init_default              {false, 0, false, 0, false, 0}
#define Proto_Lora_Config_init_default           {false, 0, false, 0, false, 0}
#define Proto_Mcu_Data_init_default              {false, 0, false, 0, false, Proto_Car_Sensor_init_default, false, Proto_Car_Sensor_init_default, false, Proto_Car_Sensor_init_default, false, Proto_Stint_Data_init_default, false, Proto_Lap_Data_init_default, {{NULL}, NULL}, {{NULL}, NULL}, {{NULL}, NULL}, false, Proto_Gps_Data_init_default, false, Proto_Car_Sensor_init_default, false, Proto_Car_Sensor_init_default, false, Proto_Lora_Config_init_default}
#define Proto_Update_Data_init_default           {false, Proto_Car_Sensor_init_default, false, Proto_Car_Sensor_init_default, false, Proto_Car_Sensor_init_default, false, Proto_Lap_Data_init_default, false, Proto_Stint_Data_init_default, false, Proto_Gps_Data_init_default}
#define Proto_Ack_Data_init_default              {false, 0}
#define Proto_LoRa_Data_init_default             {false, 0u, false, false, false, 0, false, Proto_Update_Data_init_default, false, Proto_Command_init_default, false, Proto_Ack_Data_init_default}
#define Proto_Message_init_default               {false, Proto_Mcu_Data_init_default, false, Proto_LoRa_Data_init_default, false, Proto_Command_init_default}
#define Proto_Event_init_zero                    {false, 0, false, _Proto_Event_Type_MIN, false, _Proto_Severity_MIN, false, 0, false, 0, {{NULL}, NULL}}
#define Proto_Command_init_zero                  {false, _Proto_Command_Type_MIN, false, 0, false, 0, false, 0}
#define Proto_Car_Sensor_init_zero               {false, 0, false, 0}
#define Proto_Stint_Data_init_zero               {false, 0, false, 0, false, 0, false, 0, false, 0}
#define Proto_Lap_init_zero                      {false, 0, false, 0}
#define Proto_Lap_Data_init_zero                 {false, 0, false, 0, false, 0, false, 0, 0, {Proto_Lap_init_zero, Proto_Lap_init_zero, Proto_Lap_init_zero, Proto_Lap_init_zero, Proto_Lap_init_zero}}
#define Proto_Gps_Data_init_zero                 {false, 0, false, 0, false, 0}
#define Proto_Lora_Config_init_zero              {false, 0, false, 0, false, 0}
#define Proto_Mcu_Data_init_zero                 {false, 0, false, 0, false, Proto_Car_Sensor_init_zero, false, Proto_Car_Sensor_init_zero, false, Proto_Car_Sensor_init_zero, false, Proto_Stint_Data_init_zero, false, Proto_Lap_Data_init_zero, {{NULL}, NULL}, {{NULL}, NULL}, {{NULL}, NULL}, false, Proto_Gps_Data_init_zero, false, Proto_Car_Sensor_init_zero, false, Proto_Car_Sensor_init_zero, false, Proto_Lora_Config_init_zero}
#define Proto_Update_Data_init_zero              {false, Proto_Car_Sensor_init_zero, false, Proto_Car_Sensor_init_zero, false, Proto_Car_Sensor_init_zero, false, Proto_Lap_Data_init_zero, false, Proto_Stint_Data_init_zero, false, Proto_Gps_Data_init_zero}
#define Proto_Ack_Data_init_zero                 {false, 0}
#define Proto_LoRa_Data_init_zero                {false, 0, false, 0, false, 0, false, Proto_Update_Data_init_zero, false, Proto_Command_init_zero, false, Proto_Ack_Data_init_zero}
#define Proto_Message_init_zero                  {false, Proto_Mcu_Data_init_zero, false, Proto_LoRa_Data_init_zero, false, Proto_Command_init_zero}

/* Field tags (for use in manual encoding/decoding) */
#define Proto_Event_id_tag                       1
#define Proto_Event_type_tag                     2
#define Proto_Event_severity_tag                 3
#define Proto_Event_created_at_tag               4
#define Proto_Event_displayed_since_tag          5
#define Proto_Event_text_tag                     6
#define Proto_Command_type_tag                   1
#define Proto_Command_id_tag                     2
#define Proto_Command_created_tag                3
#define Proto_Command_handled_tag                4
#define Proto_Car_Sensor_temp_tag                1
#define Proto_Car_Sensor_preassure_tag           2
#define Proto_Stint_Data_running_tag             1
#define Proto_Stint_Data_enabled_tag             2
#define Proto_Stint_Data_target_tag              3
#define Proto_Stint_Data_elapsed_tag             4
#define Proto_Stint_Data_elapsed_timestamp_tag   5
#define Proto_Lap_lap_no_tag                     1
#define Proto_Lap_lap_time_ms_tag                2
#define Proto_Lap_Data_lap_no_tag                1
#define Proto_Lap_Data_best_lap_ms_tag           2
#define Proto_Lap_Data_current_lap_ms_tag        3
#define Proto_Lap_Data_current_lap_snapshot_time_tag 4
#define Proto_Lap_Data_laps_tag                  5
#define Proto_Gps_Data_spd_tag                   1
#define Proto_Gps_Data_lat_tag                   2
#define Proto_Gps_Data_lon_tag                   3
#define Proto_Lora_Config_bandwidth_tag          1
#define Proto_Lora_Config_spreading_factor_tag   2
#define Proto_Lora_Config_output_power_tag       3
#define Proto_Mcu_Data_network_time_adjustment_tag 1
#define Proto_Mcu_Data_send_timestamp_tag        2
#define Proto_Mcu_Data_water_tag                 3
#define Proto_Mcu_Data_oil_tag                   4
#define Proto_Mcu_Data_gas_tag                   5
#define Proto_Mcu_Data_stint_tag                 6
#define Proto_Mcu_Data_lap_data_tag              7
#define Proto_Mcu_Data_events_tag                8
#define Proto_Mcu_Data_outgoing_commands_tag     9
#define Proto_Mcu_Data_incoming_commands_tag     10
#define Proto_Mcu_Data_gps_tag                   11
#define Proto_Mcu_Data_gas_warn_tag              12
#define Proto_Mcu_Data_oil_warn_tag              13
#define Proto_Mcu_Data_lora_config_tag           14
#define Proto_Update_Data_water_sensor_tag       1
#define Proto_Update_Data_oil_sensor_tag         2
#define Proto_Update_Data_gas_sensor_tag         3
#define Proto_Update_Data_lap_data_tag           4
#define Proto_Update_Data_stint_data_tag         5
#define Proto_Update_Data_gps_data_tag           6
#define Proto_Ack_Data_seq_nr_tag                1
#define Proto_LoRa_Data_seq_nr_tag               1
#define Proto_LoRa_Data_requires_ack_tag         2
#define Proto_LoRa_Data_send_timestamp_tag       3
#define Proto_LoRa_Data_update_data_tag          4
#define Proto_LoRa_Data_command_data_tag         5
#define Proto_LoRa_Data_ack_data_tag             6
#define Proto_Message_mcu_data_tag               1
#define Proto_Message_lora_data_tag              2
#define Proto_Message_command_data_tag           3

/* Struct field encoding specification for nanopb */
#define Proto_Event_FIELDLIST(X, a) \
X(a, STATIC,   OPTIONAL, INT32,    id,                1) \
X(a, STATIC,   OPTIONAL, UENUM,    type,              2) \
X(a, STATIC,   OPTIONAL, UENUM,    severity,          3) \
X(a, STATIC,   OPTIONAL, INT64,    created_at,        4) \
X(a, STATIC,   OPTIONAL, INT64,    displayed_since,   5) \
X(a, CALLBACK, OPTIONAL, STRING,   text,              6)
#define Proto_Event_CALLBACK pb_default_field_callback
#define Proto_Event_DEFAULT (const pb_byte_t*)"\x10\x01\x18\x01\x00"

#define Proto_Command_FIELDLIST(X, a) \
X(a, STATIC,   OPTIONAL, UENUM,    type,              1) \
X(a, STATIC,   OPTIONAL, INT32,    id,                2) \
X(a, STATIC,   OPTIONAL, INT64,    created,           3) \
X(a, STATIC,   OPTIONAL, INT64,    handled,           4)
#define Proto_Command_CALLBACK NULL
#define Proto_Command_DEFAULT (const pb_byte_t*)"\x08\x01\x00"

#define Proto_Car_Sensor_FIELDLIST(X, a) \
X(a, STATIC,   OPTIONAL, UINT32,   temp,              1) \
X(a, STATIC,   OPTIONAL, DOUBLE,   preassure,         2)
#define Proto_Car_Sensor_CALLBACK NULL
#define Proto_Car_Sensor_DEFAULT (const pb_byte_t*)"\x08\x00\x11\x00\x00\x00\x00\x00\x00\x00\x00\x00"

#define Proto_Stint_Data_FIELDLIST(X, a) \
X(a, STATIC,   OPTIONAL, BOOL,     running,           1) \
X(a, STATIC,   OPTIONAL, BOOL,     enabled,           2) \
X(a, STATIC,   OPTIONAL, UINT32,   target,            3) \
X(a, STATIC,   OPTIONAL, UINT32,   elapsed,           4) \
X(a, STATIC,   OPTIONAL, UINT32,   elapsed_timestamp,   5)
#define Proto_Stint_Data_CALLBACK NULL
#define Proto_Stint_Data_DEFAULT NULL

#define Proto_Lap_FIELDLIST(X, a) \
X(a, STATIC,   OPTIONAL, INT32,    lap_no,            1) \
X(a, STATIC,   OPTIONAL, INT64,    lap_time_ms,       2)
#define Proto_Lap_CALLBACK NULL
#define Proto_Lap_DEFAULT NULL

#define Proto_Lap_Data_FIELDLIST(X, a) \
X(a, STATIC,   OPTIONAL, INT32,    lap_no,            1) \
X(a, STATIC,   OPTIONAL, UINT32,   best_lap_ms,       2) \
X(a, STATIC,   OPTIONAL, UINT32,   current_lap_ms,    3) \
X(a, STATIC,   OPTIONAL, UINT32,   current_lap_snapshot_time,   4) \
X(a, STATIC,   REPEATED, MESSAGE,  laps,              5)
#define Proto_Lap_Data_CALLBACK NULL
#define Proto_Lap_Data_DEFAULT NULL
#define Proto_Lap_Data_laps_MSGTYPE Proto_Lap

#define Proto_Gps_Data_FIELDLIST(X, a) \
X(a, STATIC,   OPTIONAL, INT32,    spd,               1) \
X(a, STATIC,   OPTIONAL, DOUBLE,   lat,               2) \
X(a, STATIC,   OPTIONAL, DOUBLE,   lon,               3)
#define Proto_Gps_Data_CALLBACK NULL
#define Proto_Gps_Data_DEFAULT NULL

#define Proto_Lora_Config_FIELDLIST(X, a) \
X(a, STATIC,   OPTIONAL, DOUBLE,   bandwidth,         1) \
X(a, STATIC,   OPTIONAL, UINT32,   spreading_factor,   2) \
X(a, STATIC,   OPTIONAL, INT32,    output_power,      3)
#define Proto_Lora_Config_CALLBACK NULL
#define Proto_Lora_Config_DEFAULT NULL

#define Proto_Mcu_Data_FIELDLIST(X, a) \
X(a, STATIC,   OPTIONAL, UINT32,   network_time_adjustment,   1) \
X(a, STATIC,   OPTIONAL, UINT32,   send_timestamp,    2) \
X(a, STATIC,   OPTIONAL, MESSAGE,  water,             3) \
X(a, STATIC,   OPTIONAL, MESSAGE,  oil,               4) \
X(a, STATIC,   OPTIONAL, MESSAGE,  gas,               5) \
X(a, STATIC,   OPTIONAL, MESSAGE,  stint,             6) \
X(a, STATIC,   OPTIONAL, MESSAGE,  lap_data,          7) \
X(a, CALLBACK, REPEATED, MESSAGE,  events,            8) \
X(a, CALLBACK, REPEATED, MESSAGE,  outgoing_commands,   9) \
X(a, CALLBACK, REPEATED, MESSAGE,  incoming_commands,  10) \
X(a, STATIC,   OPTIONAL, MESSAGE,  gps,              11) \
X(a, STATIC,   OPTIONAL, MESSAGE,  gas_warn,         12) \
X(a, STATIC,   OPTIONAL, MESSAGE,  oil_warn,         13) \
X(a, STATIC,   OPTIONAL, MESSAGE,  lora_config,      14)
#define Proto_Mcu_Data_CALLBACK pb_default_field_callback
#define Proto_Mcu_Data_DEFAULT NULL
#define Proto_Mcu_Data_water_MSGTYPE Proto_Car_Sensor
#define Proto_Mcu_Data_oil_MSGTYPE Proto_Car_Sensor
#define Proto_Mcu_Data_gas_MSGTYPE Proto_Car_Sensor
#define Proto_Mcu_Data_stint_MSGTYPE Proto_Stint_Data
#define Proto_Mcu_Data_lap_data_MSGTYPE Proto_Lap_Data
#define Proto_Mcu_Data_events_MSGTYPE Proto_Event
#define Proto_Mcu_Data_outgoing_commands_MSGTYPE Proto_Command
#define Proto_Mcu_Data_incoming_commands_MSGTYPE Proto_Command
#define Proto_Mcu_Data_gps_MSGTYPE Proto_Gps_Data
#define Proto_Mcu_Data_gas_warn_MSGTYPE Proto_Car_Sensor
#define Proto_Mcu_Data_oil_warn_MSGTYPE Proto_Car_Sensor
#define Proto_Mcu_Data_lora_config_MSGTYPE Proto_Lora_Config

#define Proto_Update_Data_FIELDLIST(X, a) \
X(a, STATIC,   OPTIONAL, MESSAGE,  water_sensor,      1) \
X(a, STATIC,   OPTIONAL, MESSAGE,  oil_sensor,        2) \
X(a, STATIC,   OPTIONAL, MESSAGE,  gas_sensor,        3) \
X(a, STATIC,   OPTIONAL, MESSAGE,  lap_data,          4) \
X(a, STATIC,   OPTIONAL, MESSAGE,  stint_data,        5) \
X(a, STATIC,   OPTIONAL, MESSAGE,  gps_data,          6)
#define Proto_Update_Data_CALLBACK NULL
#define Proto_Update_Data_DEFAULT NULL
#define Proto_Update_Data_water_sensor_MSGTYPE Proto_Car_Sensor
#define Proto_Update_Data_oil_sensor_MSGTYPE Proto_Car_Sensor
#define Proto_Update_Data_gas_sensor_MSGTYPE Proto_Car_Sensor
#define Proto_Update_Data_lap_data_MSGTYPE Proto_Lap_Data
#define Proto_Update_Data_stint_data_MSGTYPE Proto_Stint_Data
#define Proto_Update_Data_gps_data_MSGTYPE Proto_Gps_Data

#define Proto_Ack_Data_FIELDLIST(X, a) \
X(a, STATIC,   OPTIONAL, UINT32,   seq_nr,            1)
#define Proto_Ack_Data_CALLBACK NULL
#define Proto_Ack_Data_DEFAULT NULL

#define Proto_LoRa_Data_FIELDLIST(X, a) \
X(a, STATIC,   OPTIONAL, UINT32,   seq_nr,            1) \
X(a, STATIC,   OPTIONAL, BOOL,     requires_ack,      2) \
X(a, STATIC,   OPTIONAL, UINT32,   send_timestamp,    3) \
X(a, STATIC,   OPTIONAL, MESSAGE,  update_data,       4) \
X(a, STATIC,   OPTIONAL, MESSAGE,  command_data,      5) \
X(a, STATIC,   OPTIONAL, MESSAGE,  ack_data,          6)
#define Proto_LoRa_Data_CALLBACK NULL
#define Proto_LoRa_Data_DEFAULT (const pb_byte_t*)"\x08\x00\x10\x00\x00"
#define Proto_LoRa_Data_update_data_MSGTYPE Proto_Update_Data
#define Proto_LoRa_Data_command_data_MSGTYPE Proto_Command
#define Proto_LoRa_Data_ack_data_MSGTYPE Proto_Ack_Data

#define Proto_Message_FIELDLIST(X, a) \
X(a, STATIC,   OPTIONAL, MESSAGE,  mcu_data,          1) \
X(a, STATIC,   OPTIONAL, MESSAGE,  lora_data,         2) \
X(a, STATIC,   OPTIONAL, MESSAGE,  command_data,      3)
#define Proto_Message_CALLBACK NULL
#define Proto_Message_DEFAULT NULL
#define Proto_Message_mcu_data_MSGTYPE Proto_Mcu_Data
#define Proto_Message_lora_data_MSGTYPE Proto_LoRa_Data
#define Proto_Message_command_data_MSGTYPE Proto_Command

extern const pb_msgdesc_t Proto_Event_msg;
extern const pb_msgdesc_t Proto_Command_msg;
extern const pb_msgdesc_t Proto_Car_Sensor_msg;
extern const pb_msgdesc_t Proto_Stint_Data_msg;
extern const pb_msgdesc_t Proto_Lap_msg;
extern const pb_msgdesc_t Proto_Lap_Data_msg;
extern const pb_msgdesc_t Proto_Gps_Data_msg;
extern const pb_msgdesc_t Proto_Lora_Config_msg;
extern const pb_msgdesc_t Proto_Mcu_Data_msg;
extern const pb_msgdesc_t Proto_Update_Data_msg;
extern const pb_msgdesc_t Proto_Ack_Data_msg;
extern const pb_msgdesc_t Proto_LoRa_Data_msg;
extern const pb_msgdesc_t Proto_Message_msg;

/* Defines for backwards compatibility with code written before nanopb-0.4.0 */
#define Proto_Event_fields &Proto_Event_msg
#define Proto_Command_fields &Proto_Command_msg
#define Proto_Car_Sensor_fields &Proto_Car_Sensor_msg
#define Proto_Stint_Data_fields &Proto_Stint_Data_msg
#define Proto_Lap_fields &Proto_Lap_msg
#define Proto_Lap_Data_fields &Proto_Lap_Data_msg
#define Proto_Gps_Data_fields &Proto_Gps_Data_msg
#define Proto_Lora_Config_fields &Proto_Lora_Config_msg
#define Proto_Mcu_Data_fields &Proto_Mcu_Data_msg
#define Proto_Update_Data_fields &Proto_Update_Data_msg
#define Proto_Ack_Data_fields &Proto_Ack_Data_msg
#define Proto_LoRa_Data_fields &Proto_LoRa_Data_msg
#define Proto_Message_fields &Proto_Message_msg

/* Maximum encoded size of messages (where known) */
/* Proto_Event_size depends on runtime parameters */
/* Proto_Mcu_Data_size depends on runtime parameters */
/* Proto_Message_size depends on runtime parameters */
#define MESSAGE_PB_H_MAX_SIZE                    Proto_LoRa_Data_size
#define Proto_Ack_Data_size                      6
#define Proto_Car_Sensor_size                    15
#define Proto_Command_size                       35
#define Proto_Gps_Data_size                      29
#define Proto_Lap_Data_size                      149
#define Proto_Lap_size                           22
#define Proto_LoRa_Data_size                     320
#define Proto_Lora_Config_size                   26
#define Proto_Stint_Data_size                    22
#define Proto_Update_Data_size                   258

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif
