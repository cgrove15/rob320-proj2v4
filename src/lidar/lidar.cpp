#include "lidar/lidar.hpp"

#define LIDAR_SCAN_SIZE 8192
#define PI 3.1415926535f

Lidar::Lidar(const std::string &port, uint32_t baudrate, uint16_t pwm)
    : is_connected(false), port(port), baudrate(baudrate), pwm(pwm), on_scan(nullptr) {
    drv = RPlidarDriver::CreateDriver();
    if (!drv) {
        std::cerr << "Error, cannot create RPlidarDriver." << std::endl;
        exit(1);
    }

    if (!connect()) {
        std::cerr << "Failed to connect to RPlidarDriver.";
        exit(1);
    }
    if (!check_health()) {
        std::cerr << "Invalid RPlidarDriver health status.";
        exit(1);
    }
}

Lidar::~Lidar() {
    // std::cout << "Lidar::~Lidar called." << std::endl;
    shutdown();
    if (drv) {
        // std::cout << "Disposing driver." << std::endl;
        RPlidarDriver::DisposeDriver(drv);
    }
    // std::cout << "Lidar::~Lidar returning." << std::endl;
}

void Lidar::set_callback(std::function<void(const LaserScan &)> on_scan) {
    this->on_scan = on_scan;
}

bool Lidar::ok() const { return is_connected && !shutdown_flag; }

void Lidar::spin() {
    if (!is_connected) {
        return;
    }

    shutdown();  // Ensure thread has been joined
    shutdown_flag = false;
    run_thread = std::thread(&Lidar::run, this);
}

bool Lidar::connect() {
    is_connected = false;
    if (IS_FAIL(drv->connect(port.c_str(), baudrate))) {
        std::cerr << "Error, cannot bind to the serial port /dev/rplidar." << std::endl;
        return false;
    }

    is_connected = true;
    return true;
}

bool Lidar::check_health() {
    rplidar_response_device_info_t devinfo;

    u_result op_result = drv->getDeviceInfo(devinfo);
    if (IS_FAIL(op_result)) {
        std::cerr << "Error, cannot get device info." << std::endl;
        return false;
    }

    rplidar_response_device_health_t healthinfo;
    op_result = drv->getHealth(healthinfo);
    if (!IS_OK(op_result)) {
        std::cerr << "Error, cannot retrieve the lidar health code: " << op_result
                  << std::endl;
        return false;
    }
    if (healthinfo.status == RPLIDAR_STATUS_ERROR) {
        std::cerr << "Error, rplidar internal error detected. Please reboot the device "
                     "to retry."
                  << std::endl;
        return false;
    }
    return true;
}

void Lidar::shutdown() {
    // std::cout << "Lidar::shutdown called." << std::endl;
    shutdown_flag = true;
    if (run_thread.joinable()) {
        // std::cout << "Joining thread." << std::endl;
        run_thread.join();
        // std::cout << "Thread joined." << std::endl;
    }
    // std::cout << "Lidar::shutdown returning." << std::endl;
}

void Lidar::run() {
    if (!is_connected) {
        return;
    }

    drv->startMotor();
    drv->setMotorPWM(pwm);
    drv->startScan(false, true);

    rix::util::Time now = rix::util::Time::now();
    rix::util::Time prev_time;

    rplidar_response_measurement_node_hq_t nodes[LIDAR_SCAN_SIZE];
    while (!shutdown_flag) {
        size_t recv_size = LIDAR_SCAN_SIZE;
        std::memset(&nodes, 0, sizeof(nodes));
        // std::cout << "Grabbing scan data." << std::endl;
        u_result op_result = drv->grabScanDataHq(nodes, recv_size);
        if (IS_OK(op_result)) {
            prev_time = now;
            now = rix::util::Time::now();
            rix::util::Duration delta = (now - prev_time) / LIDAR_SCAN_SIZE;
            drv->ascendScanData(nodes, LIDAR_SCAN_SIZE);

            current_scan.header.stamp = now.to_msg();
            current_scan.header.seq++;
            current_scan.ranges.resize(0);
            current_scan.intensities.resize(0);

            current_scan.angle_min = 0.0;
            current_scan.angle_max = 2 * PI;
            current_scan.angle_increment = (2 * PI) / LIDAR_SCAN_SIZE;
            current_scan.time_increment = (delta.to_nanoseconds() / LIDAR_SCAN_SIZE) / 1e9;
            current_scan.scan_time = delta.to_nanoseconds() / 1e9;
            current_scan.range_min = 0.0;
            current_scan.range_max = 12.0;

            for (int i = 0; i < LIDAR_SCAN_SIZE; i++) {
                int scan_idx = LIDAR_SCAN_SIZE - i - 1;
                float range = nodes[scan_idx].dist_mm_q2 / 4.0f / 1000.0f;
                // float theta = 2 * PI - nodes[scan_idx].angle_z_q14 * (PI / 32768.0);
                float intensity = nodes[scan_idx].quality >> RPLIDAR_RESP_MEASUREMENT_QUALITY_SHIFT;
                current_scan.ranges.push_back(range);
                current_scan.intensities.push_back(intensity);
            }
            if (on_scan) {
                // std::cout << "Calling on_scan." << std::endl;
                on_scan(current_scan);
            }
        } else {
            if (connect()) {
                drv->startMotor();
                drv->setMotorPWM(pwm);
                drv->startScan(false, true);
            }
        }
    }
    // std::cout << "Stopping driver." << std::endl;
    drv->stop();
    drv->stopMotor();
    // std::cout << "Lidar::run returning." << std::endl;
}