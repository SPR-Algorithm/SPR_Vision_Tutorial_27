#ifndef TOOLS__PLOTTER_HPP
#define TOOLS__PLOTTER_HPP

#include <netinet/in.h>  // sockaddr_in

#include <mutex>
#include <nlohmann/json.hpp>
#include <string>

namespace tools
{
class Plotter
{
public:
  // Numeric IPv4 address and UDP port in [1, 65535].
  Plotter(std::string host = "127.0.0.1", int port = 9870);
  ~Plotter();

  // Best-effort debug output: a busy sender, full socket or oversized datagram
  // drops the sample instead of delaying the control loop.
  void plot(const nlohmann::json & json);

private:
  int socket_ = -1;
  sockaddr_in destination_{};
  std::mutex mutex_;
};

}  // namespace tools

#endif  // TOOLS__PLOTTER_HPP
