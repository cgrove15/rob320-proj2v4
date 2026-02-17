#include "lidar_driver/lidar_driver.hpp"

LidarDriver::LidarDriver(std::unique_ptr<interfaces::Server> server, std::unique_ptr<LidarBase> lidar)
    : server(std::move(server)), lidar(std::move(lidar)) {
    this->lidar->set_callback(std::bind(&LidarDriver::on_scan_callback, this, std::placeholders::_1));
}

LidarDriver::~LidarDriver() { lidar->set_callback(nullptr); }

/**< TODO */
void LidarDriver::spin(std::unique_ptr<interfaces::Notification> notif) {
    lidar->spin();
    
    while(true) {

        //see if the signal is caught
        if(notif->wait(rix::util::Duration(0))) {
            break;
        }

        //accept connection
        connection_mtx.lock();
        if(!server->accept(connection)) {
            connection_mtx.unlock();
            continue;
        }
        connection_mtx.unlock();

        //convert to connectiontcp
        connection_mtx.lock();
        auto shared_p = connection.lock();
        connection_mtx.unlock();

        if(!shared_p) {
            continue;
        }

        bool leave = false;
        while(true) {
            if(notif->wait(rix::util::Duration(0))) {
                leave = true;
                break;
            }

            if(!shared_p) {
                break;
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(15));
        }

        server->close(connection);
        connection.reset();

        if(leave) {
            break;
        }
    }

}

/**< TODO */
void LidarDriver::on_scan_callback(const rix::msg::sensor::LaserScan &scan) {
    connection_mtx.lock();
    auto shared_p = connection.lock();
    connection_mtx.unlock();

    if(!shared_p) {
        return;
    }

    rix::msg::standard::UInt32 size;
    size.data = static_cast<uint32_t>(scan.size());

    std::vector<uint8_t> size_buff(4); 
    size_t offset = 0;
    size.serialize(size_buff.data(), offset);

    //serialize data
    std::vector<uint8_t> msg_buff(size.data);
    offset = 0;
    scan.serialize(msg_buff.data(), offset);

    //  write the byte array prefixed with its size to stdout
    shared_p->write(size_buff.data(), 4);
    shared_p->write(msg_buff.data(), msg_buff.size());
}
