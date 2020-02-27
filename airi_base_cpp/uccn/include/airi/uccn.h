#ifndef AIRI_UCCN_H_
#define AIRI_UCCN_H_

#include "uccn/uccn.h"

struct qencoder_state_s
{
  int32_t ticks;
};

struct airi_uccn_drive_state_s
{
  struct qencoder_state_s left_encoder;
  struct qencoder_state_s right_encoder;
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

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif  // AIRI_UCCN_H_
