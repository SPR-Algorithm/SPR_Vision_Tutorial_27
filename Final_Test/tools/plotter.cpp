#include "plotter.hpp"

#include <arpa/inet.h>   // htons, inet_pton
#include <sys/socket.h>  // socket, sendto
#include <unistd.h>      // close

#include <cerrno>
#include <stdexcept>
#include <system_error>

namespace tools
{
Plotter::Plotter(std::string host, int port)
{
  if (port < 1 || port > 65535) {
    throw std::invalid_argument("Plotter UDP port must be in [1, 65535]");
  }
  destination_.sin_family = AF_INET;
  destination_.sin_port = ::htons(static_cast<std::uint16_t>(port));
  if (host.find('\0') != std::string::npos ||
      ::inet_pton(AF_INET, host.c_str(), &destination_.sin_addr) != 1) {
    throw std::invalid_argument("Plotter requires a numeric IPv4 address: " + host);
  }
  socket_ = ::socket(AF_INET, SOCK_DGRAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0);
  if (socket_ < 0) {
    throw std::system_error(errno, std::generic_category(), "Plotter UDP socket");
  }
}

Plotter::~Plotter()
{
  if (socket_ >= 0) ::close(socket_);
}

void Plotter::plot(const nlohmann::json & json)
{
  std::unique_lock<std::mutex> lock(mutex_, std::try_to_lock);
  if (!lock.owns_lock()) return;
  const auto data = json.dump();
  // Maximum UDP payload with an IPv4 header, without fragmentation extensions.
  if (data.size() > 65507) return;
  // No retry: debug samples may be dropped when the socket cannot send now.
  (void)::sendto(
    socket_, data.data(), data.size(), MSG_DONTWAIT,
    reinterpret_cast<const sockaddr *>(&destination_), sizeof(destination_));
}

}  // namespace tools
