// Minimal message_traits implementation for tf3 (no ROS dependency)
#ifndef TF3_MESSAGE_TRAITS_H
#define TF3_MESSAGE_TRAITS_H

namespace tf3 {
  namespace message_traits {
    template <typename T>
    struct IsMessage { static const bool value = false; };
  }
}

#endif // TF3_MESSAGE_TRAITS_H
