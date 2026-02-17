#include "mbot/mbot.hpp"

MBot::MBot(const std::string &port)
    : robot_fd(-1),
      port(port),
      on_pose(nullptr),
      is_connected(false),
      shutdown_flag(false),
      controller_running(false),
      controller_start_semaphore(0),
      controller_abort_semaphore(0),
      current_pose(std::make_unique<rix::msg::geometry::Pose2DStamped>()),
      goal_pose(std::make_unique<rix::msg::geometry::Pose2DStamped>()) {
    robot_fd = open(this->port.c_str(), O_RDWR);
    if (robot_fd < 0) {
        std::cerr << "Failed to open the serial port." << std::endl;
        return;
    }

    // Set up the serial port
    tcgetattr(robot_fd, &options);
    cfsetspeed(&options, B115200);
    options.c_cflag &= ~(CSIZE | PARENB | CSTOPB | CRTSCTS);
    options.c_cflag |= CS8 | CREAD | CLOCAL;
    options.c_oflag &= ~OPOST;
    options.c_lflag &= ~(ICANON | ISIG | ECHO | IEXTEN); /* Set non-canonical mode */
    options.c_cc[VTIME] = 1;
    options.c_cc[VMIN] = 0;
    cfmakeraw(&options);
    tcflush(robot_fd, TCIFLUSH);
    tcsetattr(robot_fd, TCSANOW, &options);
    if (tcgetattr(robot_fd, &options) != 0) {
        std::cerr << "Failed to set terminal attributes." << std::endl;
        close(robot_fd);
        return;
    }
    is_connected = true;
}

MBot::~MBot() { 
    shutdown(); 
    close(robot_fd);
}

void MBot::spin() {
    if (!is_connected) {
        return;
    }

    shutdown();  // Ensure thread has been joined
    shutdown_flag = false;

    // Start timesync thread
    timesync_thread = std::thread(&MBot::timesync, this);

    // Start listener thread
    listener_thread = std::thread(&MBot::listener, this);

    // Start controller thread
    controller_thread = std::thread(&MBot::controller, this);

    rix::msg::geometry::Twist2DStamped stop_cmd;
    drive(stop_cmd);
    reset_pose();
}

bool MBot::ok() const { return is_connected && !shutdown_flag; }

void MBot::set_callback(std::function<void(const Pose2DStamped &)> on_pose) {
    this->on_pose = on_pose;
}

void MBot::drive(const Twist2DStamped &cmd) const {
    // Create Twist2D message
    serial_twist2D_t mbot_cmd;
    mbot_cmd.utime = rix::util::Time(cmd.header.stamp).to_microseconds();
    mbot_cmd.vx = cmd.twist.vx;
    mbot_cmd.vy = cmd.twist.vy;
    mbot_cmd.wz = cmd.twist.wz;

    // Encode the command
    uint8_t msg[sizeof(serial_twist2D_t) + ROS_PKG_LENGTH];
    encode_msg((uint8_t *)&mbot_cmd, sizeof(serial_twist2D_t),
               MBOT_TOPIC_ID::MBOT_VEL_CMD, msg, sizeof(msg));

    // Send the drive command
    robot_fd_mutex.lock();
    int status = write(robot_fd, msg, sizeof(msg));
    robot_fd_mutex.unlock();
    if (status < 0) {
        std::cerr << "Failed to write message: " << strerror(errno) << std::endl;
        return;
    }
}

void MBot::drive_to(const Pose2DStamped &cmd) const {
    goal_pose_mutex.lock();
    *goal_pose = cmd;
    goal_pose_mutex.unlock();
    controller_start_semaphore.release();
}

void MBot::shutdown() {
    shutdown_flag = true;
    if (timesync_thread.joinable()) {
        timesync_thread.join();
    }
    if (listener_thread.joinable()) {
        listener_thread.join();
    }
    if (controller_thread.joinable()) {
        controller_thread.join();
    }
}

void MBot::reset_pose() {
    serial_pose2D_t msg;
    msg.utime = rix::util::Time::now().to_microseconds();
    msg.x = 0;
    msg.y = 0;
    msg.theta = 0;
    const size_t msg_size = sizeof(serial_pose2D_t) + ROS_PKG_LENGTH;
    uint8_t rospkt[msg_size];
    if (encode_msg((uint8_t *)&msg, sizeof(serial_pose2D_t), MBOT_TOPIC_ID::MBOT_ODOMETRY_RESET, rospkt, msg_size) < 0) {
        std::cerr << "Failed to encode message." << std::endl;
        return;
    }

    int status;
    robot_fd_mutex.lock();
    status = write(robot_fd, rospkt, msg_size);
    robot_fd_mutex.unlock();
    if (status < 0) {
        std::cerr << "Failed to write message: " << strerror(errno) << std::endl;
        return;
    }
}

void MBot::timesync() {
    while (!shutdown_flag) {
        // Encode the timesync message
        serial_timestamp_t msg;
        msg.utime = rix::util::Time::now().to_microseconds();
        const size_t msg_size = sizeof(serial_timestamp_t) + ROS_PKG_LENGTH;
        uint8_t rospkt[msg_size];
        if (encode_msg((uint8_t *)&msg, sizeof(serial_timestamp_t), MBOT_TOPIC_ID::MBOT_TIMESYNC, rospkt, msg_size) < 0) {
            std::cerr << "Failed to encode message." << std::endl;
            return;
        }

        // Send the timesync message
        robot_fd_mutex.lock();
        int status = write(robot_fd, rospkt, msg_size);
        robot_fd_mutex.unlock();
        if (status < 0) {
            std::cerr << "Failed to write message: " << strerror(errno) << std::endl;
            return;
        }

        // Run at 2 Hz
        usleep(500000);
    }
}

void MBot::listener() {
    uint8_t header_data[ROS_HEADER_LENGTH];
    header_data[0] = 0x00;

    while (!shutdown_flag) {
        // Read the header and check if we lost serial connection
        robot_fd_mutex.lock();
        int header_status = read_header(header_data);
        robot_fd_mutex.unlock();
        if (header_status < 0) {
            std::cerr << "Failed to read header: " << strerror(errno) << std::endl;
            return;
        }

        bool valid_header = (header_status == 1);
        if (valid_header) {
            valid_header = validate_header(header_data);
        }

        if (valid_header) {
            uint16_t msg_len = ((uint16_t)header_data[3] << 8) + (uint16_t)header_data[2];
            uint16_t topic_id = ((uint16_t)header_data[6] << 8) + (uint16_t)header_data[5];
            uint8_t *msg = new uint8_t[msg_len];

            int avail = 0;
            ioctl(robot_fd, FIONREAD, &avail);
            while (avail < (msg_len + 1) && !shutdown_flag) {
                usleep(1000);
                ioctl(robot_fd, FIONREAD, &avail);
            }
            if (shutdown_flag) {
                break;
            }

            // Read the message and check if we lost serial connection
            char topic_msg_data_checksum = 0;
            robot_fd_mutex.lock();
            int message_status = read_message(msg, msg_len, &topic_msg_data_checksum);
            robot_fd_mutex.unlock();
            if (message_status < 0) {
                std::cerr << "Failed to read message: " << strerror(errno) << std::endl;
                break;
            }

            bool valid_message = (message_status == 1);
            if (valid_message) {
                valid_message = validate_message(header_data, msg, msg_len, topic_msg_data_checksum);
                if (valid_message) {
                    handle_msg(topic_id, msg, msg_len);
                }
            }
            delete[] msg;
        }

        header_data[0] = 0x00;
    }
}

void MBot::controller() {
    const float kp_v = 0.6;
    const float kp_t = 1.2;
    rix::msg::geometry::Twist2DStamped stop_cmd = {};
    while (!shutdown_flag) {
        if (!controller_start_semaphore.try_acquire()) {
            usleep(1000);
            continue;
        }

        controller_running_mutex.lock();
        controller_running = true;
        controller_running_mutex.unlock();

        while (!shutdown_flag) {
            if (controller_abort_semaphore.try_acquire()) {
                break;
            }

            rix::msg::geometry::Pose2DStamped current;
            current_pose_mutex.lock();
            current = *current_pose;
            current_pose_mutex.unlock();

            rix::msg::geometry::Pose2DStamped goal;
            goal_pose_mutex.lock();
            goal = *goal_pose;
            goal_pose_mutex.unlock();

            float dx = goal.pose.x - current.pose.x;
            float dy = goal.pose.y - current.pose.y;
            float dt = goal.pose.theta - current.pose.theta;

            float mag = sqrt(dx * dx + dy * dy);
            bool reached_linear_goal = mag < 0.01;
            bool reached_angular_goal = fabs(dt) < M_PI / 360;

            rix::msg::geometry::Twist2DStamped cmd;
            cmd.twist.vx = kp_v *  (cos(current.pose.theta) * dx + sin(current.pose.theta) * dy);
            cmd.twist.vy = kp_v * (-sin(current.pose.theta) * dx + cos(current.pose.theta) * dy);
            cmd.twist.wz = kp_t * dt;

            if (reached_linear_goal && reached_angular_goal) {
                drive(stop_cmd);
                break;
            } else if (reached_linear_goal) {
                cmd.twist.vx = 0.0;
                cmd.twist.vy = 0.0;
            } else if (reached_angular_goal) {
                cmd.twist.wz = 0.0;
            }

            drive(cmd);
            usleep(10000);
        }

        controller_running_mutex.lock();
        controller_running = false;
        controller_running_mutex.unlock();
    }
}

void MBot::handle_msg(uint16_t topic_id, void *msg, size_t msg_len) const {
    switch (topic_id) {
        case MBOT_TOPIC_ID::MBOT_ODOMETRY:
            if (msg_len != sizeof(serial_pose2D_t)) {
                break;
            }
            current_pose_mutex.lock();
            serial_pose2D_t mbot_pose;
            std::memcpy(&mbot_pose, msg, sizeof(serial_pose2D_t));

            current_pose->header.frame_id = "mbot";
            current_pose->header.seq++;
            current_pose->header.stamp.sec = mbot_pose.utime / 1'000'000;
            current_pose->header.stamp.nsec = (mbot_pose.utime % 1'000'000) * 1'000;

            current_pose->pose.x = mbot_pose.x;
            current_pose->pose.y = mbot_pose.y;
            current_pose->pose.theta = mbot_pose.theta;

            current_pose_mutex.unlock();
            on_pose(*current_pose);
            break;
        default:
            break;
    }
}

uint8_t MBot::checksum(uint8_t *addends, int len) const {
    // takes in an array and sums the contents then checksums the array
    int sum = 0;
    for (int i = 0; i < len; i++) {
        sum += addends[i];
    }
    return 255 - ((sum) % 256);
}

int MBot::encode_msg(uint8_t *msg, int msg_len, uint16_t topic, uint8_t *rospkt,
                     int rospkt_len) const {
    // SANITY CHECKS
    if (msg_len + ROS_PKG_LENGTH != rospkt_len) {
        return -1;
    }

    // CREATE ROS PACKET
    // for ROS protocol and packet format see link:
    // http://wiki.ros.org/rosserial/Overview/Protocol
    rospkt[0] = SYNC_FLAG;
    rospkt[1] = VERSION_FLAG;
    rospkt[2] =
        (uint8_t)(msg_len & 0xFF);  // message length lower 8/16b via bitwise AND and cast
    rospkt[3] =
        (uint8_t)(msg_len >> 8);  // message length higher 8/16b via bitshift and cast

    uint8_t cs1_addends[2] = {rospkt[2], rospkt[3]};
    rospkt[4] = checksum(cs1_addends, 2);  // checksum over message length
    rospkt[5] =
        (uint8_t)(topic & 0xFF);  // message topic lower 8/16b via bitwise AND and cast
    rospkt[6] =
        (uint8_t)(topic >> 8);  // message length higher 8/16b via bitshift and cast

    memcpy(&rospkt[ROS_HEADER_LENGTH], msg, msg_len);  // copy message data to packet

    uint8_t cs2_addends[msg_len + 2];  // create array for the checksum over topic and
                                       // message content
    cs2_addends[0] = rospkt[5];
    cs2_addends[1] = rospkt[6];
    for (int i = 0; i < msg_len; i++) {
        cs2_addends[i + 2] = msg[i];
    }

    rospkt[rospkt_len - 1] =
        checksum(cs2_addends, msg_len + 2);  // checksum over message data and topic

    return 0;
}

bool MBot::read_header(uint8_t *header_data) const {
    unsigned char trigger_val = 0x00;
    int rc = 0x00;
    while (trigger_val != 0xff && !shutdown_flag && rc != 1) {
        rc = read(robot_fd, &trigger_val, 1);
        if (rc < 0) {
            return -1;
        }
    }
    header_data[0] = trigger_val;

    rc = read(robot_fd, &header_data[1], ROS_HEADER_LENGTH - 1);
    if (rc < 0) {
        return -1;
    }

    return (rc == ROS_HEADER_LENGTH - 1);
}

// Header validation function
bool MBot::validate_header(uint8_t *header_data) const {
    bool valid_header = (header_data[1] == 0xfe);
    uint8_t cs1_addends[2] = {header_data[2], header_data[3]};
    uint8_t cs_msg_len = checksum(cs1_addends, 2);
    valid_header = valid_header && (cs_msg_len == header_data[4]);

    return valid_header;
}

// Message read function
bool MBot::read_message(uint8_t *msg, uint16_t msg_len,
                        char *topic_msg_data_checksum) const {
    int rc = read(robot_fd, msg, msg_len);
    if (rc < 0) {
        return -1;
    }
    bool valid_message = (rc == msg_len);

    rc = read(robot_fd, topic_msg_data_checksum, 1);
    if (rc < 0) {
        return -1;
    }
    valid_message = valid_message && (rc == 1);

    return valid_message;
}

// Message validation function
bool MBot::validate_message(uint8_t *header_data, uint8_t *msg, uint16_t msg_len,
                            char topic_msg_data_checksum) const {
    uint8_t cs2_addends[msg_len + 2];
    cs2_addends[0] = header_data[5];
    cs2_addends[1] = header_data[6];
    for (int i = 0; i < msg_len; i++) {
        cs2_addends[i + 2] = msg[i];
    }

    uint8_t cs_topic_msg_data = checksum(cs2_addends, msg_len + 2);
    bool valid_message = (cs_topic_msg_data == topic_msg_data_checksum);

    return valid_message;
}
