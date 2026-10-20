#include "tools/plotter.hpp"

#include <arpa/inet.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <chrono>
#include <iostream>
#include <stdexcept>
#include <string>

namespace
{
void require(bool condition, const std::string & message)
{
  if (!condition) throw std::runtime_error(message);
}

struct UdpReceiver
{
  int fd = -1;
  int port = 0;

  UdpReceiver()
  {
    fd = ::socket(AF_INET, SOCK_DGRAM | SOCK_CLOEXEC, 0);
    require(fd >= 0, "create local UDP receiver");
    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = ::htonl(INADDR_LOOPBACK);
    address.sin_port = 0;
    if (::bind(fd, reinterpret_cast<const sockaddr *>(&address), sizeof(address)) != 0) {
      ::close(fd); fd = -1;
      throw std::runtime_error("bind local UDP receiver");
    }
    socklen_t size = sizeof(address);
    if (::getsockname(fd, reinterpret_cast<sockaddr *>(&address), &size) != 0) {
      ::close(fd); fd = -1;
      throw std::runtime_error("read local UDP port");
    }
    port = ::ntohs(address.sin_port);
  }
  ~UdpReceiver() { if (fd >= 0) ::close(fd); }

  nlohmann::json receive()
  {
    pollfd event{fd, POLLIN, 0};
    require(::poll(&event, 1, 1000) == 1 && (event.revents & POLLIN), "UDP sample not received");
    char buffer[65536];
    const auto count = ::recv(fd, buffer, sizeof(buffer), MSG_DONTWAIT);
    require(count > 0, "receive UDP sample");
    return nlohmann::json::parse(buffer, buffer + count);
  }
};

void test_nested_json_and_time()
{
  UdpReceiver receiver;
  tools::Plotter plotter("127.0.0.1", receiver.port);
  const nlohmann::json sample = {
    {"time", 12.345}, {"target", {{"yaw", 179.9}}},
    {"gimbal", {{"yaw", -179.8}}}, {"plan", {{"yaw", 180.1}}},
    {"tracking", true}, {"fire_reason", "cooldown"}};
  plotter.plot(sample);
  require(receiver.receive() == sample, "UDP preserves nested JSON curves and explicit time");
  auto next = sample;
  next["time"] = 12.355;
  next["target"]["yaw"] = 180.2;
  plotter.plot(next);
  const auto received = receiver.receive();
  require(received == next, "second sample preserves time and curve values");
  require(received["time"].get<double>() > sample["time"].get<double>(), "time advances");
}

void test_unavailable_receiver_and_oversized_sample()
{
  int unused_port;
  { UdpReceiver reservation; unused_port = reservation.port; }
  tools::Plotter absent("127.0.0.1", unused_port);
  const auto begin = std::chrono::steady_clock::now();
  for (int i = 0; i < 5000; ++i) absent.plot({{"time", i * 0.01}, {"yaw", 0.1}});
  require(std::chrono::steady_clock::now() - begin < std::chrono::seconds(2),
          "debug sends block when no UDP receiver is listening");

  UdpReceiver receiver;
  tools::Plotter plotter("127.0.0.1", receiver.port);
  plotter.plot({{"oversized", std::string(65508, 'x')}});
  const nlohmann::json valid = {{"time", 1.0}, {"target", {{"yaw", 0.5}}}};
  plotter.plot(valid);
  require(receiver.receive() == valid, "oversized sample is dropped and sender recovers");
  pollfd event{receiver.fd, POLLIN, 0};
  require(::poll(&event, 1, 0) == 0, "oversized sample unexpectedly emitted a datagram");
}

void test_invalid_destination()
{
  for (const auto & host : {"", "localhost", "256.0.0.1", "127.0.0.1 trailing", "::1"}) {
    bool rejected = false;
    try { tools::Plotter invalid(host, 9870); }
    catch (const std::invalid_argument &) { rejected = true; }
    require(rejected, std::string("invalid IPv4 accepted: ") + host);
  }
  for (int port : {-1, 0, 65536, 100000}) {
    bool rejected = false;
    try { tools::Plotter invalid("127.0.0.1", port); }
    catch (const std::invalid_argument &) { rejected = true; }
    require(rejected, "invalid UDP port accepted: " + std::to_string(port));
  }
}
}  // namespace

int main()
{
  try {
    test_nested_json_and_time();
    test_unavailable_receiver_and_oversized_sample();
    test_invalid_destination();
    std::cout << "plotter_tests: all checks passed\n";
    return 0;
  } catch (const std::exception & error) {
    std::cerr << "plotter_tests: " << error.what() << '\n';
    return 1;
  }
}
