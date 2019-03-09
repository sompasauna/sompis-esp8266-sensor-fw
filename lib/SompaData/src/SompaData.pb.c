/* Automatically generated nanopb constant definitions */
/* Generated by nanopb-0.3.9 at Tue Nov 28 22:29:44 2017. */

#include "SompaData.pb.h"

/* @@protoc_insertion_point(includes) */
#if PB_PROTO_HEADER_VERSION != 30
#error Regenerate this file with the current version of nanopb generator.
#endif



const pb_field_t SensorReport_fields[12] = {
    PB_FIELD(  1, UINT32  , SINGULAR, STATIC  , FIRST, SensorReport, device_id, device_id, 0),
    PB_FIELD(  2, UINT32  , SINGULAR, STATIC  , OTHER, SensorReport, sensor_id, device_id, 0),
    PB_FIELD(  3, UINT32  , SINGULAR, STATIC  , OTHER, SensorReport, timestamp, sensor_id, 0),
    PB_ONEOF_FIELD(value,   4, FLOAT   , ONEOF, STATIC  , OTHER, SensorReport, f, timestamp, 0),
    PB_ONEOF_FIELD(value,   5, UINT32  , ONEOF, STATIC  , UNION, SensorReport, percentage, timestamp, 0),
    PB_ONEOF_FIELD(value,   6, INT32   , ONEOF, STATIC  , UNION, SensorReport, val, timestamp, 0),
    PB_ONEOF_FIELD(value,   7, BOOL    , ONEOF, STATIC  , UNION, SensorReport, b, timestamp, 0),
    PB_FIELD(  8, UINT32  , SINGULAR, STATIC  , OTHER, SensorReport, wifi_failcount, value.b, 0),
    PB_FIELD(  9, UINT32  , SINGULAR, STATIC  , OTHER, SensorReport, wifi_id, wifi_failcount, 0),
    PB_FIELD( 10, UINT32  , SINGULAR, STATIC  , OTHER, SensorReport, boot_count, wifi_id, 0),
    PB_FIELD( 11, UINT32  , SINGULAR, STATIC  , OTHER, SensorReport, millis, boot_count, 0),
    PB_LAST_FIELD
};

const pb_field_t ServerResponse_fields[6] = {
    PB_FIELD(  1, UINT32  , SINGULAR, STATIC  , FIRST, ServerResponse, goto_sleep_seconds, goto_sleep_seconds, 0),
    PB_FIELD(  2, BOOL    , SINGULAR, STATIC  , OTHER, ServerResponse, upgrade_available, goto_sleep_seconds, 0),
    PB_FIELD(  3, BOOL    , SINGULAR, STATIC  , OTHER, ServerResponse, start_ota_ap, upgrade_available, 0),
    PB_FIELD(  4, STRING  , SINGULAR, CALLBACK, OTHER, ServerResponse, firmware_url, start_ota_ap, 0),
    PB_FIELD(  5, STRING  , SINGULAR, CALLBACK, OTHER, ServerResponse, spiffs_url, firmware_url, 0),
    PB_LAST_FIELD
};

const pb_field_t Ping_fields[3] = {
    PB_FIELD(  1, UENUM   , SINGULAR, STATIC  , FIRST, Ping, operation, operation, 0),
    PB_FIELD(  2, UINT32  , SINGULAR, STATIC  , OTHER, Ping, number, operation, 0),
    PB_LAST_FIELD
};



/* @@protoc_insertion_point(eof) */
