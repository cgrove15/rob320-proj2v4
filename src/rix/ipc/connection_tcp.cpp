#include "rix/ipc/connection_tcp.hpp"

namespace rix {
namespace ipc {

ConnectionTCP::ConnectionTCP(const Socket &socket) : socket(socket) {}

ConnectionTCP::ConnectionTCP() : socket() {}

ConnectionTCP::ConnectionTCP(const ConnectionTCP &other) : socket(other.socket) {}

ConnectionTCP &ConnectionTCP::operator=(const ConnectionTCP &other) {
    if (this != &other) {
        socket = other.socket;
    }
    return *this;
}

ConnectionTCP::~ConnectionTCP() {}

/**< TODO */
ssize_t ConnectionTCP::write(const uint8_t *buffer, size_t len) const { 
    return ::write(socket.fd(), buffer, len);
}

/**< TODO */
ssize_t ConnectionTCP::read(uint8_t *buffer, size_t len) const {
    return ::read(socket.fd(), buffer, len);
}

/**< TODO */
Endpoint ConnectionTCP::remote_endpoint() const { 
    Endpoint ep;
    socket.getpeername(ep);
    return ep;
}

/**< TODO */
Endpoint ConnectionTCP::local_endpoint() const { 
    Endpoint ep;
    socket.getsockname(ep);
    return ep;
}

/**< TODO */
bool ConnectionTCP::ok() const { 
    return !(socket.fd() < 0);
}

/**< TODO */
bool ConnectionTCP::wait_for_writable(const rix::util::Duration &duration) const { 
    return socket.wait_for_writable(duration);
}

/**< TODO */
bool ConnectionTCP::wait_for_readable(const rix::util::Duration &duration) const {
        //make a pollfd of the file to input to poll function
    return socket.wait_for_readable(duration);
}

/**< TODO */
void ConnectionTCP::set_nonblocking(bool status) {
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
bool ConnectionTCP::is_nonblocking() const {
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