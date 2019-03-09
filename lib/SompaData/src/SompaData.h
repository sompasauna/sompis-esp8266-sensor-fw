#ifndef SOMPADATA_H
#define SOMPADATA_H

#include <pb_encode.h>
#include <pb_decode.h>
#include "SompaData.pb.h"

bool encode_sr_unionmessage(pb_ostream_t *stream, const pb_field_t messagetype[], const void *message);

#endif /* SOMPADATA_H */