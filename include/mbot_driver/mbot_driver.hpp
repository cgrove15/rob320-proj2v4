#pragma once

#include <functional>
#include <iostream>
#include <mutex>
#include <string>

#include "mbot/mbot.hpp"
#include "rix/ipc/client_tcp.hpp"
#include "rix/ipc/server_tcp.hpp"
#include "rix/ipc/signal.hpp"
#include "rix/msg/geometry/Pose2DStamped.hpp"
#include "rix/msg/standard/UInt32.hpp"

using namespace rix::ipc;
using namespace rix::util;
using rix::msg::geometry::Pose2DStamped;

class MBotDriver {
   public:
    MBotDriver(std::unique_ptr<interfaces::Server> server, std::unique_ptr<interfaces::Client> client,
               std::unique_ptr<MBotBase> mbot);
    void spin(std::unique_ptr<interfaces::Notification> notif);

   private:
    void on_pose_callback(const Pose2DStamped &pose);

    std::unique_ptr<MBotBase> mbot;
    std::unique_ptr<interfaces::Server> server;
    std::unique_ptr<interfaces::Client> client;
    std::weak_ptr<interfaces::Connection> connection;
    std::mutex connection_mtx;
};