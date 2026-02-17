#pragma once

#include <functional>
#include <memory>

#include "rix/msg/sensor/LaserScan.hpp"

using rix::msg::sensor::LaserScan;

class LidarBase {
   public:
    LidarBase() = default;
    virtual ~LidarBase() = default;
    
    virtual void set_callback(std::function<void(const LaserScan &)> on_scan) = 0;
    virtual bool ok() const = 0;
    virtual void spin() = 0;
};