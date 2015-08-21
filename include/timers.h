#ifndef TIMERS_H
#define TIMERS_H

#include <chrono>
#include <string>
#include <ros/ros.h>

// scoped timer
class ScopedTimer {
 public:
  ScopedTimer(const std::string &str) {
    msg_ = str;
    start_ = std::chrono::system_clock::now();
  }
  ~ScopedTimer() {
    std::chrono::time_point<std::chrono::system_clock> end;
    end = std::chrono::system_clock::now();
    std::chrono::duration<double> dt = end - start_;
    ROS_INFO_STREAM(msg_ << " took " << dt.count() << " seconds!");
  }

 private:
  std::string msg_;
  std::chrono::time_point<std::chrono::system_clock> start_;
};
// tic toc timer
class Timer {
 public:
  Timer() { tic(); }
  void tic() { start_ = std::chrono::system_clock::now(); }
  double toc() {
    std::chrono::time_point<std::chrono::system_clock> end;
    end = std::chrono::system_clock::now();
    std::chrono::duration<double> dt = end - start_;
    return dt.count();
  }

 private:
  std::chrono::time_point<std::chrono::system_clock> start_;
};

#endif  // TIMERS_H
