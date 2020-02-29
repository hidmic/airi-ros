#ifndef AIRI_UCCN_HPP_
#define AIRI_UCCN_HPP_

#include "uccn/uccn.hpp"

#include "airi/uccn.h"

namespace airi {

namespace uccn {

struct drive_state : airi_uccn_drive_state_s
{
  static const uccn_record_typesupport_s * get_typesupport() {
    return get_airi_uccn_drive_state_typesupport();
  }
};

struct drive_command : airi_uccn_drive_command_s
{
  static const uccn_record_typesupport_s * get_typesupport() {
    return get_airi_uccn_drive_command_typesupport();
  }
};

}  // namespace uccn

}  // namespace airi

#endif  // AIRI_UCCN_HPP_
