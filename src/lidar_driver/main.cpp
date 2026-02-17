#include "lidar_driver/lidar_driver.hpp"
#include "rix/util/log.hpp"

int main() {
    auto lidar = std::make_unique<Lidar>("/dev/rplidar", 115200, 700);
    if (!lidar->ok()) {
        rix::util::Log::error << "Failed to initialize lidar." << std::endl;
        return 1;
    }

    auto server = std::make_unique<ServerTCP>(Endpoint("0.0.0.0", 8100));
    if (!server->ok()) {
        rix::util::Log::error << "Failed to initialize server." << std::endl;
        return 1;
    }

    Signal ignore(SIGPIPE);  // Register a handler to SIGPIPE to ignore it
    auto notif = std::make_unique<Signal>(SIGINT);

    LidarDriver driver(std::move(server), std::move(lidar));
    driver.spin(std::move(notif));
    
    return 0;
}