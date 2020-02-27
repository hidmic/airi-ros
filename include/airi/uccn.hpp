#ifndef AIRI_UCCN_HPP_
#define AIRI_UCCN_HPP_

#include "uccn/uccn.hpp"

#include "airi/uccn.h"

namespace airi {

struct drive_state
{
  static const uccn_record_typesupport_s * get_typesupport() {
    return get_airi_drive_state_typesupport();
  }

  qencoder_state_s left_encoder;
  qencoder_state_s right_encoder;
};

}  // namespace airi

#endif  // AIRI_UCCN_HPP_
