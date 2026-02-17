#include "mbot_driver/mbot_driver.hpp"

using rix::msg::standard::UInt32;

MBotDriver::MBotDriver(std::unique_ptr<interfaces::Server> server, std::unique_ptr<interfaces::Client> client,
                       std::unique_ptr<MBotBase> mbot)
    : server(std::move(server)), client(std::move(client)), mbot(std::move(mbot)) {
    this->mbot->set_callback(std::bind(&MBotDriver::on_pose_callback, this, std::placeholders::_1));
}

/**< TODO */
// accept a connection from a client using the server passed to the constructor
// get the accepted connection’s remote endpoint 
// use it to connect to a server at the same address with the port 8300 
//    using the client passed to the constructor
// read serialized Pose2DStamped messages (prefixed with the 4-byte size as in Project 1) from the client, 
// deserialize them
// use the MBot::drive_to function to pass goal poses to the position controller running in the background
// If at any point the client fails to read a message or reaches EOF, the client should be reset

void MBotDriver::spin(std::unique_ptr<interfaces::Notification> notif) {
    mbot->spin();
    
    while(!notif || !notif->is_ready()) {

        // //see if the signal is caught
        // if(notif->wait(rix::util::Duration(0.01))) {
        //     break;
        // }

        connection_mtx.lock();
        auto shared_p = connection.lock();
        connection_mtx.unlock();

        if(!shared_p) {
            //accept connection

            connection_mtx.lock();
            if(!server->accept(connection)) {
                connection_mtx.unlock();
                continue;
            }
            connection_mtx.unlock();


            //convert to connectiontcp
            connection_mtx.lock();
            shared_p = connection.lock();
            connection_mtx.unlock();

            if(!shared_p) {
                continue;
            }

            //get client's remote endpoint
            Endpoint client_ep = shared_p->remote_endpoint();
            

            //connect to the server but on port 8300
            Endpoint server_ep;
            server_ep.port = 8300;
            server_ep.address = client_ep.address;
            client->reset();

            if(!client->connect(server_ep)) {
                continue;
            }
        }
            
        //wait for data
        if(!client->wait_for_readable(rix::util::Duration(0.1))) {
            client->reset();
            continue;  // No data yet
        }
            
        //get input size
        uint8_t size_buf[4];
        ssize_t num_read = client->read(size_buf, 4);
        size_t offset = 0;

        //failed read data
        if(num_read <= 0) { 
            client->reset();
            continue;
        }

        //deserialize size
        rix::msg::standard::UInt32 size;
        if(!size.deserialize(size_buf, 4, offset)) {
            break;
        }

        //read input data
        std::vector<uint8_t> buff(size.data);
        ssize_t data_read = client->read(buff.data(), size.data);

        //failed read data
        if(data_read <= 0) {
            client->reset();
            continue;
        }

        //deseralize
        rix::msg::geometry::Pose2DStamped bot_command;
        offset = 0;
        if(!bot_command.deserialize(buff.data(), size.data, offset)) {
            break;
        }

        //send mbot commands
        mbot->drive_to(bot_command);
    }
}

/**< TODO */
// serialize (with the size prefixed to the message as in Project 1)
// send the PoseStamped2D message received from the MBot to the 
// connected client using the connection accepted by the server. 
void MBotDriver::on_pose_callback(const Pose2DStamped &pose) {

    connection_mtx.lock();
    auto shared_p = connection.lock();
    connection_mtx.unlock();

    if(!shared_p) {
        return;
    }

    rix::msg::standard::UInt32 size;
    size.data = static_cast<uint32_t>(pose.size());

    std::vector<uint8_t> size_buff(4); 
    size_t offset = 0;
    size.serialize(size_buff.data(), offset);

    //serialize data
    std::vector<uint8_t> msg_buff(size.data);
    offset = 0;
    pose.serialize(msg_buff.data(), offset);

    //  write the byte array prefixed with its size to stdout
    shared_p->write(size_buff.data(), 4);
    shared_p->write(msg_buff.data(), msg_buff.size());
}