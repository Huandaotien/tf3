// Compatibility types to avoid using ros:: or geometry_msgs:: namespaces inside tf3
#ifndef TF3_COMPAT_H
#define TF3_COMPAT_H

#include "tf3/time.h"
#include <string>

namespace tf3 {

// Minimal header/message equivalents owned by tf3 (no ros:: or geometry_msgs::)
struct HeaderMsg
{
  uint32_t seq;
  Time stamp;
  std::string frame_id;
};

struct Vector3Msg { double x, y, z; };
struct QuaternionMsg { double x, y, z, w; };
struct TransformMsg { Vector3Msg translation; QuaternionMsg rotation; };

struct TransformStampedMsg
{
  HeaderMsg header;
  std::string child_frame_id;
  TransformMsg transform;
};

} // namespace tf3

#endif // TF3_COMPAT_H
