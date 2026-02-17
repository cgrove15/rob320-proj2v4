#include "rix/ipc/socket.hpp"

namespace rix {
namespace ipc {

Socket::Socket() : File() {}

Socket::Socket(int domain, int type)
    : File(socket(domain, type, 0)), domain(domain), type(type), _bound(false), _listening(false) {}

Socket::Socket(int fd, int domain, int type) : File(fd), domain(domain), type(type), _bound(false), _listening(false) {}

Socket::Socket(const Socket &src)
    : File(src), domain(src.domain), type(src.type), _bound(src._bound), _listening(src._listening) {}

Socket &Socket::operator=(const Socket &src) {
    if (this != &src) {
        (File &)*this = src;
        domain = src.domain;
        type = src.type;
        _bound = src._bound;
        _listening = src._listening;
    }
    return *this;
}

Socket::Socket(Socket &&src)
    : File(std::move(src)),
      domain(std::move(src.domain)),
      type(std::move(src.type)),
      _bound(std::move(src._bound)),
      _listening(std::move(src._listening)) {}

Socket &Socket::operator=(Socket &&src) {
    (File &)*this = std::move(src);
    domain = std::move(src.domain);
    type = std::move(src.type);
    _bound = std::move(src._bound);
    _listening = std::move(src._listening);
    return *this;
}

Socket::~Socket() {}

/**< TODO
 * Hint: You only need to consider the case when domain is AF_INET (IPv4)
 */
bool Socket::bind(const Endpoint &endpoint) { 
    if(_bound) {
        return false;
    }
    
    struct sockaddr_in addr;
    addr.sin_family = AF_INET;
    addr.sin_port = htons(endpoint.port);
    if(inet_pton(AF_INET, endpoint.address.c_str(), &addr.sin_addr) < 1) {
        return false;
    }

    if(::bind(fd_, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        return false;
    }

    _bound = true;
    return true;
}

/**< TODO */
bool Socket::listen(int backlog) { 
    if(_listening) {
        return false;
    }
    
    if(::listen(fd_, backlog) < 0) { 
        return false;
    }

    _listening = true;
    return true;
}

/**< TODO
 * Hint: You only need to consider the case when domain is AF_INET (IPv4)
 */
bool Socket::connect(const Endpoint &endpoint) { 
    struct sockaddr_in addr;
    addr.sin_family = AF_INET;
    addr.sin_port = htons(endpoint.port);
    if(inet_pton(AF_INET, endpoint.address.c_str(), &addr.sin_addr) < 1) {
        return false;
    }

    if(::connect(fd_, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        return false;
    }

    _bound = true;
    return true;
}

/**< TODO */
bool Socket::accept(Socket &sock) {
    if(!_listening) {
        return false;
    }

    struct sockaddr_in client_addr;
    socklen_t size = sizeof(client_addr);

    int fd__ = ::accept(fd_, (struct sockaddr*)&client_addr, &size);
    
    if(fd__ < 0) {
        return false;
    }

    sock = Socket(fd__, domain, type);
    sock._bound = true;
    return true;
}

/**< TODO
 * Hint: You only need to consider the case when domain is AF_INET (IPv4)
 */
bool Socket::getsockname(Endpoint &endpoint) const { 
    struct sockaddr_in addr;
    socklen_t size = sizeof(addr);
   
    if(::getsockname(fd_, (struct sockaddr*)&addr, &size) < 0) {
        return false;
    }

    char ip_addr[INET_ADDRSTRLEN];
    inet_ntop(AF_INET, &addr.sin_addr, ip_addr, sizeof(ip_addr));

    endpoint.port = ntohs(addr.sin_port);
    endpoint.address = ip_addr;

    return true;
}

/**< TODO
 * Hint: You only need to consider the case when domain is AF_INET (IPv4)
 */
bool Socket::getpeername(Endpoint &endpoint) const { 
    struct sockaddr_in addr;
    socklen_t size = sizeof(addr);

    if(::getpeername(fd_, (struct sockaddr*)&addr, &size) < 0) {
        return false;
    }

    char ip_addr[INET_ADDRSTRLEN];
    inet_ntop(AF_INET, &addr.sin_addr, ip_addr, sizeof(ip_addr));

    endpoint.port = ntohs(addr.sin_port);
    endpoint.address = ip_addr;

    return true;
}

/**< TODO */
bool Socket::getsockopt(int level, int optname, int &value) { 
    socklen_t size = sizeof(value);
    
    if(::getsockopt(fd_, level, optname, &value, &size) < 0) {
        return false;
    }

    return true;
}

/**< TODO */
bool Socket::setsockopt(int level, int optname, int value) { 
    if(::setsockopt(fd_, level, optname, &value, sizeof(value)) < 0) {
        return false;
    }

    return true;
}

bool Socket::is_bound() const { return _bound; }

bool Socket::is_listening() const { return _listening; }

}  // namespace ipc
}  // namespace rix