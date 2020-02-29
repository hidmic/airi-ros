#ifndef AIRI_UCCN_H_
#define AIRI_UCCN_H_

#include <stdint.h>
#include "uccn/uccn.h"

typedef int32_t q16_16_t;

struct qencoder_state_s
{
  int32_t  ticks;  // in ticks
  q16_16_t position;  // in rads
  q16_16_t displacement;  // in rads
  q16_16_t velocity;  // in rads/sec
};

struct airi_uccn_drive_state_s
{
  struct qencoder_state_s left_encoder;
  struct qencoder_state_s right_encoder;
};

struct wheel_command_s
{
  q16_16_t velocity;  // in rads/sec
};

struct airi_uccn_drive_command_s
{
  struct wheel_command_s left_wheel;
  struct wheel_command_s right_wheel;
};

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

const struct uccn_record_typesupport_s * get_airi_uccn_drive_state_typesupport(void);

const struct uccn_record_typesupport_s * get_airi_uccn_drive_command_typesupport(void);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif  // AIRI_UCCN_H_
