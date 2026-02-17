#include "rix/ipc/server_tcp.hpp"

namespace rix {
namespace ipc {

/**< TODO */
ServerTCP::ServerTCP(const Endpoint &ep, size_t backlog) 
    : socket(AF_INET, SOCK_STREAM) {

    int value = 1;
    if(!socket.setsockopt(SOL_SOCKET, SO_REUSEADDR, value)) {
        return;
    }
    
    if(!socket.bind(ep)) {
        return;
    }

    if(!socket.listen(backlog)) {
        return;
    }
}

ServerTCP::ServerTCP() : socket(), connections() {}

ServerTCP::ServerTCP(const ServerTCP &other) : socket(other.socket), connections(other.connections) {}

ServerTCP &ServerTCP::operator=(const ServerTCP &other) {
    if (this != &other) {
        socket = other.socket;
        connections = other.connections;
    }
    return *this;
}

ServerTCP::~ServerTCP() {}

/**< TODO */
void ServerTCP::close(const std::weak_ptr<interfaces::Connection> &connection) {
    //get shared connection from connection
    auto shared_connection = connection.lock();

    //close connection
    if(!shared_connection) {
        return;
    }

    connections.erase(shared_connection);
}

/**< TODO */
bool ServerTCP::ok() const { 
    return (socket.fd() > -1 && socket.is_bound() && socket.is_listening());
}

/**< TODO */
Endpoint ServerTCP::local_endpoint() const { 
    Endpoint ep;
    socket.getsockname(ep);
    return ep;
}

/**< TODO */
bool ServerTCP::accept(std::weak_ptr<interfaces::Connection> &connection) { 
    if (!ok()) {
        return false;
    }    
    
    Socket new_socket;
    //if no pending connections...
    if(!socket.accept(new_socket)) {
        return false;
    }


    ConnectionTCP* conn = new ConnectionTCP(new_socket);
    std::shared_ptr<interfaces::Connection> new_connection(conn);

    connections.insert(new_connection);
    connection = new_connection;
    return true;
}

/**< TODO */
bool ServerTCP::wait_for_accept(rix::util::Duration duration) const { 
    return socket.wait_for_readable(duration);
}

/**< TODO */
void ServerTCP::set_nonblocking(bool status) {
    int flags = fcntl(socket.fd(), F_GETFL);
    
    if(flags == -1) {
        return;
    }
    
    if(status) {
        flags |= O_NONBLOCK;
    } else {
        flags &= ~O_NONBLOCK;
    }

    fcntl(socket.fd(), F_SETFL, flags);
}

/**< TODO */
bool ServerTCP::is_nonblocking() const {
    int flags = fcntl(socket.fd(), F_GETFL);
    if(flags == -1) {
        return false;
    }

    if((flags & O_NONBLOCK) != 0) {
        return true;
    }

    return false;
}

}  // namespace ipc
}  // namespace rix