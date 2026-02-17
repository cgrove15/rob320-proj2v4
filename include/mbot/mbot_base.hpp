#pragma once

#include <functional>
#include <memory>

#include "rix/msg/geometry/Pose2DStamped.hpp"
#include "rix/msg/geometry/Twist2DStamped.hpp"

using rix::msg::geometry::Pose2DStamped;
using rix::msg::geometry::Twist2DStamped;

class MBotBase {
   public:
    MBotBase() = default;
    virtual ~MBotBase() = default;

    virtual void set_callback(std::function<void(const Pose2DStamped &)> on_pose) = 0;
    virtual bool ok() const = 0;
    virtual void spin() = 0;
    virtual void drive(const Twist2DStamped &cmd) const = 0;
    virtual void drive_to(const Pose2DStamped &cmd) const = 0;
};