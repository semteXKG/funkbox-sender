/* Automatically generated nanopb constant definitions */
/* Generated by nanopb-0.4.8 */

#include "message.pb.h"
#if PB_PROTO_HEADER_VERSION != 40
#error Regenerate this file with the current version of nanopb generator.
#endif

PB_BIND(Proto_Event, Proto_Event, AUTO)


PB_BIND(Proto_Command, Proto_Command, AUTO)


PB_BIND(Proto_Car_Sensor, Proto_Car_Sensor, AUTO)


PB_BIND(Proto_Stint_Data, Proto_Stint_Data, AUTO)


PB_BIND(Proto_Lap, Proto_Lap, AUTO)


PB_BIND(Proto_Lap_Data, Proto_Lap_Data, AUTO)


PB_BIND(Proto_Mcu_Data, Proto_Mcu_Data, 2)


PB_BIND(Proto_Update_Data, Proto_Update_Data, AUTO)


PB_BIND(Proto_Ack_Data, Proto_Ack_Data, AUTO)


PB_BIND(Proto_LoRa_Data, Proto_LoRa_Data, 2)


PB_BIND(Proto_Message, Proto_Message, 2)







#ifndef PB_CONVERT_DOUBLE_FLOAT
/* On some platforms (such as AVR), double is really float.
 * To be able to encode/decode double on these platforms, you need.
 * to define PB_CONVERT_DOUBLE_FLOAT in pb.h or compiler command line.
 */
PB_STATIC_ASSERT(sizeof(double) == 8, DOUBLE_MUST_BE_8_BYTES)
#endif

