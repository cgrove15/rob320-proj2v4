#pragma once

#include <errno.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

#include <cmath>
#include <cstdint>
#include <cstring>
#include <functional>
#include <iostream>
#include <mutex>
#include <semaphore>
#include <string>
#include <thread>

#include "mbot/mbot_base.hpp"
#include "mbot/messages.hpp"
#include "rix/msg/geometry/Pose2DStamped.hpp"
#include "rix/msg/geometry/Twist2DStamped.hpp"
#include "rix/util/time.hpp"

using rix::msg::geometry::Pose2DStamped;
using rix::msg::geometry::Twist2DStamped;

class MBot : public MBotBase {
   public:
    MBot(const std::string &port);
    ~MBot();

    void set_callback(std::function<void(const Pose2DStamped &)> on_pose) override;
    bool ok() const override;
    void spin() override;
    void drive(const Twist2DStamped &cmd) const override;
    void drive_to(const Pose2DStamped &cmd) const override;

   private:
    std::thread timesync_thread;
    std::thread listener_thread;
    std::thread controller_thread;

    std::string port;
    struct termios options;

    mutable std::mutex robot_fd_mutex;
    mutable std::mutex current_pose_mutex;
    mutable std::mutex goal_pose_mutex;
    mutable std::mutex controller_running_mutex;
    mutable std::binary_semaphore controller_start_semaphore;
    mutable std::binary_semaphore controller_abort_semaphore;

    int robot_fd;
    std::function<void(const Pose2DStamped &)> on_pose;

    std::unique_ptr<Pose2DStamped> current_pose;
    std::unique_ptr<Pose2DStamped> goal_pose;

    bool is_connected;
    bool shutdown_flag;
    bool controller_running;

    void shutdown();
    void timesync();
    void listener();
    void controller();
    void reset_pose();

    void handle_msg(uint16_t topic_id, void *msg, size_t msg_len) const;

    uint8_t checksum(uint8_t *addends, int len) const;
    int encode_msg(uint8_t *msg, int msg_len, uint16_t topic, uint8_t *rospkt,
                   int rospkt_len) const;

    bool read_header(uint8_t *header_data) const;
    bool validate_header(uint8_t *header_data) const;
    bool read_message(uint8_t *msg, uint16_t msg_len,
                      char *topic_msg_data_checksum) const;
    bool validate_message(uint8_t *header_data, uint8_t *msg, uint16_t msg_len,
                          char topic_msg_data_checksum) const;
};
