// Minimal compatibility macros used by tf3 to avoid depending on ROS
#ifndef TF3_MACROS_H_
#define TF3_MACROS_H_

#if defined(__clang__) || defined(__GNUC__)
# define ROS_DEPRECATED __attribute__((deprecated))
#elif defined(_MSC_VER)
# define ROS_DEPRECATED __declspec(deprecated)
#else
# define ROS_DEPRECATED
#endif

#endif // TF3_MACROS_H_
