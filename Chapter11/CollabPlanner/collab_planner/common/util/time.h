//
// Created by Yakun Ouyang on 2021/9/15.
//
#pragma once
#include <ros/ros.h>

namespace common {
namespace util {

using namespace std::chrono;

inline double GetCurrentTimestamp() {
    return ((double)duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count() / 1000);
}

// inline double GetCurrentTimestamp() {
//   return ros::Time::now().toSec();
// }

}    // namespace util
}    // namespace common
