#include "SompaData.h"
#include <pb_encode.h>
#include <pb_decode.h>
#include "SompaData.pb.h"

bool encode_sr_unionmessage(pb_ostream_t *stream, const pb_field_t messagetype[], const void *message)
{
    const pb_field_t *field;
    for (field = SensorReport_fields; field->tag != 0; field++)
    {
        if (field->ptr == messagetype)
        {
            /* This is our field, encode the message using it. */
            if (!pb_encode_tag_for_field(stream, field))
                return false;
            
            return pb_encode_submessage(stream, messagetype, message);
        }
    }
    
    /* Didn't find the field for messagetype */
    return false;
}