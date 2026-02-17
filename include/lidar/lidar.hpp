#pragma once

#include <rplidar.h>
#include <signal.h>
#include <sys/time.h>
#include <unistd.h>

#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <functional>
#include <iostream>
#include <thread>

#include "lidar/lidar_base.hpp"
#include "rix/util/time.hpp"
#include "rix/msg/sensor/LaserScan.hpp"

using namespace rp::standalone::rplidar;

using rix::msg::sensor::LaserScan;

class Lidar : public LidarBase {
   public:
    Lidar(const std::string &port, uint32_t baudrate, uint16_t pwm);
    ~Lidar();

    void set_callback(std::function<void(const LaserScan &)> on_scan) override;
    bool ok() const override;
    void spin() override;

   private:
    bool connect();
    bool check_health();
    void run();
    void shutdown();

    std::thread run_thread;
    std::string port;
    RPlidarDriver *drv;
    LaserScan current_scan;
    std::function<void(const LaserScan &)> on_scan;
    uint32_t baudrate;
    uint16_t pwm;
    bool is_connected;
    bool shutdown_flag;

};
