#include "rix/ipc/client_tcp.hpp"

namespace rix {
namespace ipc {

/**< TODO */
ClientTCP::ClientTCP() 
    : socket(AF_INET, SOCK_STREAM) {}

ClientTCP::ClientTCP(const ClientTCP &other) : socket(other.socket) {}

ClientTCP &ClientTCP::operator=(const ClientTCP &other) {
    if (this != &other) {
        socket = other.socket;
    }
    return *this;
}

ClientTCP::~ClientTCP() {}

/**< TODO */
bool ClientTCP::connect(const Endpoint &endpoint) { 
    if(!ok()) {
        return false;
    }

    if(is_nonblocking()) {
        socket.connect(endpoint);
        return false;
    }

    return socket.connect(endpoint);
}

/**< TODO */
ssize_t ClientTCP::write(const uint8_t *buffer, size_t len) const { 
    if(!ok()) {
        return false;
    }

    return ::write(socket.fd(), buffer, len);
}

/**< TODO */
ssize_t ClientTCP::read(uint8_t *buffer, size_t len) const { 
    if(!ok()) {
        return false;
    }

    return ::read(socket.fd(), buffer, len);
}

/**< TODO */
Endpoint ClientTCP::remote_endpoint() const { 
    Endpoint ep;
    socket.getpeername(ep);
    return ep;
}

/**< TODO */
Endpoint ClientTCP::local_endpoint() const {
    Endpoint ep;
    socket.getsockname(ep);
    return ep;
}

/**< TODO */
bool ClientTCP::ok() const {
    return (socket.fd() > -1);
}

/**< TODO */
bool ClientTCP::wait_for_connect(const rix::util::Duration &duration) const { 
    return socket.wait_for_writable(duration);
}

/**< TODO */
bool ClientTCP::wait_for_writable(const rix::util::Duration &duration) const { 
    return socket.wait_for_writable(duration);
}

/**< TODO */
bool ClientTCP::wait_for_readable(const rix::util::Duration &duration) const { 
    return socket.wait_for_readable(duration);
}

/**< TODO */
void ClientTCP::set_nonblocking(bool status) {
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
bool ClientTCP::is_nonblocking() const { 
    int flags = fcntl(socket.fd(), F_GETFL);
    if(flags == -1) {
        return false;
    }

    if((flags & O_NONBLOCK) != 0) {
        return true;
    }

    return false;
}

void ClientTCP::reset() { socket = Socket(AF_INET, SOCK_STREAM); }

}  // namespace ipc
}  // namespace rix