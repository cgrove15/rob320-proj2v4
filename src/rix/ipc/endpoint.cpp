#include "rix/ipc/endpoint.hpp"

namespace rix {
namespace ipc {

Endpoint::Endpoint() : address(""), port(0) {}

Endpoint::Endpoint(const std::string &address, int port) : address(address), port(port) {}

bool Endpoint::operator<(const Endpoint &other) const {
    return address < other.address || (address == other.address && port < other.port);
}

bool Endpoint::operator==(const Endpoint &other) const { return address == other.address && port == other.port; }

bool Endpoint::operator!=(const Endpoint &other) const { return !(*this == other); }

std::string Endpoint::to_string() const { return address + ":" + std::to_string(port); }

}  // namespace ipc
}  // namespace rix