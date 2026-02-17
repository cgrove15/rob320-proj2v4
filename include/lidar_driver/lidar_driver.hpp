#pragma once

#include <functional>
#include <iostream>
#include <mutex>
#include <string>

#include "lidar/lidar.hpp"
#include "rix/ipc/server_tcp.hpp"
#include "rix/ipc/signal.hpp"
#include "rix/msg/sensor/LaserScan.hpp"
#include "rix/msg/standard/UInt32.hpp"

using namespace rix::ipc;
using namespace rix::util;
using rix::msg::sensor::LaserScan;

class LidarDriver {
   public:
    LidarDriver(std::unique_ptr<interfaces::Server> server,
                std::unique_ptr<LidarBase> lidar);
    ~LidarDriver();

    void spin(std::unique_ptr<interfaces::Notification> notif);

   private:
    void on_scan_callback(const rix::msg::sensor::LaserScan &scan);

    std::unique_ptr<LidarBase> lidar;
    std::unique_ptr<interfaces::Server> server;
    std::weak_ptr<interfaces::Connection> connection;
    std::mutex connection_mtx;
};