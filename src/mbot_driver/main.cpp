#include "mbot_driver/mbot_driver.hpp"
#include "rix/util/log.hpp"

int main() {
    auto mbot = std::make_unique<MBot>("/dev/mbot_lcm");
    if (!mbot->ok()) {
        rix::util::Log::error << "Failed to initialize MBot." << std::endl;
        return 1;
    }

    auto server = std::make_unique<ServerTCP>(Endpoint("0.0.0.0", 8200));
    if (!server->ok()) {
        rix::util::Log::error << "Failed to initialize server." << std::endl;
        return 1;
    }
    
    auto client = std::make_unique<ClientTCP>();
    if (!client->ok()) {
        rix::util::Log::error << "Failed to initialize client." << std::endl;
        return 1;
    }

    Signal ignore(SIGPIPE);  // Register a handler to SIGPIPE to ignore it
    auto notif = std::make_unique<Signal>(SIGINT);

    MBotDriver driver(std::move(server), std::move(client), std::move(mbot));
    driver.spin(std::move(notif));

    return 0;
}