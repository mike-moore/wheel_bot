/* Automatically generated nanopb constant definitions */
/* Generated by nanopb-0.3.6-dev at Sun Aug  6 08:51:18 2017. */

#include "comm_packet.pb.h"

/* @@protoc_insertion_point(includes) */
#if PB_PROTO_HEADER_VERSION != 30
#error Regenerate this file with the current version of nanopb generator.
#endif



const pb_field_t WayPoint_fields[4] = {
    PB_FIELD(  1, FLOAT   , REQUIRED, STATIC  , FIRST, WayPoint, Heading, Heading, 0),
    PB_FIELD(  2, FLOAT   , REQUIRED, STATIC  , OTHER, WayPoint, Distance, Heading, 0),
    PB_FIELD(  3, STRING  , REQUIRED, STATIC  , OTHER, WayPoint, Name, Distance, 0),
    PB_LAST_FIELD
};

const pb_field_t IdValuePairFloat_fields[3] = {
    PB_FIELD(  1, UINT32  , REQUIRED, STATIC  , FIRST, IdValuePairFloat, Id, Id, 0),
    PB_FIELD(  2, FLOAT   , OPTIONAL, STATIC  , OTHER, IdValuePairFloat, Value, Id, 0),
    PB_LAST_FIELD
};

const pb_field_t CommandPacket_fields[3] = {
    PB_FIELD(  1, MESSAGE , OPTIONAL, STATIC  , FIRST, CommandPacket, WayPointCmd, WayPointCmd, &WayPoint_fields),
    PB_FIELD(  2, MESSAGE , REPEATED, STATIC  , OTHER, CommandPacket, RoverCmds, WayPointCmd, &IdValuePairFloat_fields),
    PB_LAST_FIELD
};

const pb_field_t TelemetryPacket_fields[5] = {
    PB_FIELD(  1, FLOAT   , REQUIRED, STATIC  , FIRST, TelemetryPacket, MeasuredHeading, MeasuredHeading, 0),
    PB_FIELD(  2, FLOAT   , REQUIRED, STATIC  , OTHER, TelemetryPacket, MeasuredDistance, MeasuredHeading, 0),
    PB_FIELD(  3, MESSAGE , REPEATED, STATIC  , OTHER, TelemetryPacket, RoverStatus, MeasuredDistance, &IdValuePairFloat_fields),
    PB_FIELD(  4, STRING  , OPTIONAL, STATIC  , OTHER, TelemetryPacket, ActiveWayPoint, RoverStatus, 0),
    PB_LAST_FIELD
};


/* Check that field information fits in pb_field_t */
#if !defined(PB_FIELD_32BIT)
/* If you get an error here, it means that you need to define PB_FIELD_32BIT
 * compile-time option. You can do that in pb.h or on compiler command line.
 * 
 * The reason you need to do this is that some of your messages contain tag
 * numbers or field sizes that are larger than what can fit in 8 or 16 bit
 * field descriptors.
 */
PB_STATIC_ASSERT((pb_membersize(CommandPacket, WayPointCmd) < 65536 && pb_membersize(CommandPacket, RoverCmds[0]) < 65536 && pb_membersize(TelemetryPacket, RoverStatus[0]) < 65536), YOU_MUST_DEFINE_PB_FIELD_32BIT_FOR_MESSAGES_WayPoint_IdValuePairFloat_CommandPacket_TelemetryPacket)
#endif

#if !defined(PB_FIELD_16BIT) && !defined(PB_FIELD_32BIT)
/* If you get an error here, it means that you need to define PB_FIELD_16BIT
 * compile-time option. You can do that in pb.h or on compiler command line.
 * 
 * The reason you need to do this is that some of your messages contain tag
 * numbers or field sizes that are larger than what can fit in the default
 * 8 bit descriptors.
 */
PB_STATIC_ASSERT((pb_membersize(CommandPacket, WayPointCmd) < 256 && pb_membersize(CommandPacket, RoverCmds[0]) < 256 && pb_membersize(TelemetryPacket, RoverStatus[0]) < 256), YOU_MUST_DEFINE_PB_FIELD_16BIT_FOR_MESSAGES_WayPoint_IdValuePairFloat_CommandPacket_TelemetryPacket)
#endif


/* @@protoc_insertion_point(eof) */
