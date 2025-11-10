// Minimal tf3::Time and tf3::Duration (non-ROS) for standalone tf3
#ifndef TF3_TIME_H
#define TF3_TIME_H

#include <cstdint>
#include <limits>
#include <cmath>
#include <chrono>
#include <thread>

namespace tf3 {

class Duration;

class Time {
public:
  uint32_t sec;
  uint32_t nsec;

  Time(): sec(0), nsec(0) {}
  Time(uint32_t s, uint32_t ns): sec(s), nsec(ns) {}

  static Time now()
  {
    using namespace std::chrono;
    auto now = system_clock::now().time_since_epoch();
    auto ns = duration_cast<nanoseconds>(now).count();
    Time t;
    t.sec = static_cast<uint32_t>(ns / 1000000000ULL);
    t.nsec = static_cast<uint32_t>(ns % 1000000000ULL);
    return t;
  }

  static Time fromSec(double s)
  {
    Time t;
    t.sec = static_cast<uint32_t>(std::floor(s));
    t.nsec = static_cast<uint32_t>((s - t.sec) * 1e9);
    return t;
  }

  double toSec() const { return static_cast<double>(sec) + static_cast<double>(nsec) * 1e-9; }
  bool isZero() const { return sec == 0 && nsec == 0; }

  bool operator==(const Time& o) const { return sec == o.sec && nsec == o.nsec; }
  bool operator!=(const Time& o) const { return !(*this == o); }
  bool operator<(const Time& o) const { return sec < o.sec || (sec == o.sec && nsec < o.nsec); }
  bool operator>(const Time& o) const { return o < *this; }
  bool operator<=(const Time& o) const { return !(*this > o); }
  bool operator>=(const Time& o) const { return !(*this < o); }

  friend Duration operator-(const Time& a, const Time& b);
  friend Time operator+(const Time& t, const Duration& d);
  friend Time operator-(const Time& t, const Duration& d);

  static const Time& TIME_MAX()
  {
    static Time tm{std::numeric_limits<uint32_t>::max(), std::numeric_limits<uint32_t>::max()};
    return tm;
  }
};

class Duration {
public:
  int32_t sec;
  int32_t nsec;

  Duration(): sec(0), nsec(0) {}
  Duration(int32_t s, int32_t ns): sec(s), nsec(ns) { normalize(); }
  explicit Duration(double seconds)
  {
    sec = static_cast<int32_t>(std::floor(seconds));
    nsec = static_cast<int32_t>((seconds - sec) * 1e9);
    normalize();
  }

  static Duration fromSec(double s) { return Duration(s); }
  static Duration fromNSec(int64_t ns_total)
  {
    int32_t s = static_cast<int32_t>(ns_total / 1000000000LL);
    int32_t ns = static_cast<int32_t>(ns_total % 1000000000LL);
    return Duration(s, ns);
  }

  double toSec() const { return static_cast<double>(sec) + static_cast<double>(nsec) * 1e-9; }

  void normalize()
  {
    if (nsec >= 1000000000L)
    {
      sec += nsec / 1000000000L;
      nsec = nsec % 1000000000L;
    }
    else if (nsec < 0)
    {
      int32_t borrow = (std::abs(nsec) / 1000000000L) + 1;
      sec -= borrow;
      nsec += borrow * 1000000000L;
    }
  }

  Duration operator+(const Duration& other) const { return Duration(sec + other.sec, nsec + other.nsec); }
  Duration operator-(const Duration& other) const { return Duration(sec - other.sec, nsec - other.nsec); }
  Duration& operator+=(const Duration& other) { sec += other.sec; nsec += other.nsec; normalize(); return *this; }
  Duration& operator-=(const Duration& other) { sec -= other.sec; nsec -= other.nsec; normalize(); return *this; }

  bool operator==(const Duration& o) const { return sec == o.sec && nsec == o.nsec; }
  bool operator!=(const Duration& o) const { return !(*this == o); }
  bool operator<(const Duration& o) const { return sec < o.sec || (sec == o.sec && nsec < o.nsec); }
  bool operator>(const Duration& o) const { return o < *this; }
  bool operator<=(const Duration& o) const { return !(*this > o); }
  bool operator>=(const Duration& o) const { return !(*this < o); }

  void sleep() const { if (sec < 0 || nsec < 0) return; auto ns_total = static_cast<int64_t>(sec) * 1000000000LL + nsec; std::this_thread::sleep_for(std::chrono::nanoseconds(ns_total)); }
};

inline Duration operator-(const Time& a, const Time& b)
{
  int32_t s = static_cast<int32_t>(a.sec) - static_cast<int32_t>(b.sec);
  int32_t ns = static_cast<int32_t>(a.nsec) - static_cast<int32_t>(b.nsec);
  Duration d(s, ns);
  d.normalize();
  return d;
}

inline Time operator+(const Time& t, const Duration& d)
{
  int64_t total_ns = static_cast<int64_t>(t.sec) * 1000000000LL + t.nsec + static_cast<int64_t>(d.sec) * 1000000000LL + d.nsec;
  Time out;
  out.sec = static_cast<uint32_t>(total_ns / 1000000000LL);
  out.nsec = static_cast<uint32_t>(total_ns % 1000000000LL);
  return out;
}

inline Time operator-(const Time& t, const Duration& d)
{
  int64_t total_ns = static_cast<int64_t>(t.sec) * 1000000000LL + t.nsec - (static_cast<int64_t>(d.sec) * 1000000000LL + d.nsec);
  if (total_ns < 0) total_ns = 0;
  Time out;
  out.sec = static_cast<uint32_t>(total_ns / 1000000000LL);
  out.nsec = static_cast<uint32_t>(total_ns % 1000000000LL);
  return out;
}

// Provide tf3::TIME_MAX symbol
namespace {
  static const Time TIME_MAX = Time::TIME_MAX();
}

} // namespace tf3

#endif // TF3_TIME_H
