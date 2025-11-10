// TF3 error codes in tf3 namespace (no tf3_msgs dependency)
#ifndef TF3_ERROR_TF3_H
#define TF3_ERROR_TF3_H

namespace tf3 {
struct TF3Error {
  enum { NO_ERROR = 0, LOOKUP_ERROR = 1, CONNECTIVITY_ERROR = 2, EXTRAPOLATION_ERROR = 3 };
};
}

#endif // TF3_ERROR_TF3_H
