#include "sim_bridge/foxglove_debug.hpp"

#include <boost/asio.hpp>
#include <boost/beast/core.hpp>
#include <boost/beast/websocket.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>

#include <array>
#include <chrono>
#include <cmath>
#include <future>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <thread>

namespace
{
namespace asio = boost::asio;
namespace beast = boost::beast;
namespace websocket = beast::websocket;
using tcp = asio::ip::tcp;
using Json = nlohmann::json;
using Clock = std::chrono::steady_clock;

void check(bool condition, const char * message)
{
  if (!condition) throw std::runtime_error(message);
}

void validate(const Json & value, const Json & schema)
{
  const auto type = schema.value("type", std::string{});
  if (type == "object") {
    check(value.is_object(), "payload does not match object schema");
    for (const auto & key : schema.value("required", Json::array())) {
      check(value.contains(key.get<std::string>()), "payload lacks schema-required field");
    }
    const auto properties = schema.value("properties", Json::object());
    for (const auto & property : properties.items()) {
      if (value.contains(property.key())) validate(value[property.key()], property.value());
    }
  } else if (type == "array") {
    check(value.is_array(), "payload does not match array schema");
    for (const auto & item : value) validate(item, schema.at("items"));
  } else if (type == "number") {
    check(value.is_number(), "payload does not match number schema");
  } else if (type == "integer") {
    check(value.is_number_integer(), "payload does not match integer schema");
  } else if (type == "string") {
    check(value.is_string(), "payload does not match string schema");
  } else if (type == "boolean") {
    check(value.is_boolean(), "payload does not match boolean schema");
  }
}

struct Message
{
  bool binary;
  std::string bytes;
};

struct Client
{
  asio::io_context io;
  websocket::stream<beast::tcp_stream> ws{io};
  Json channels;

  explicit Client(uint16_t port)
  {
    beast::get_lowest_layer(ws).connect({asio::ip::make_address("127.0.0.1"), port});
    ws.set_option(websocket::stream_base::decorator([](websocket::request_type & request) {
      request.set(beast::http::field::sec_websocket_protocol, "foxglove.websocket.v1");
    }));
    websocket::response_type response;
    ws.handshake(response, "127.0.0.1", "/");
    check(response[beast::http::field::sec_websocket_protocol] == "foxglove.websocket.v1", "missing negotiated subprotocol");
    const auto info_message = read();
    check(!info_message.binary, "serverInfo must be text");
    const auto info = Json::parse(info_message.bytes);
    check(info["op"] == "serverInfo" && info["capabilities"].is_array(), "invalid serverInfo");
    const auto advertised = read();
    check(!advertised.binary, "advertisement must be text");
    const auto advertisement = Json::parse(advertised.bytes);
    check(advertisement["op"] == "advertise", "missing advertise");
    channels = advertisement["channels"];
    check(channels.size() == 3, "expected metrics/image/scene channels");
    for (const auto & channel : channels) {
      check(channel["encoding"] == "json" && channel["schemaEncoding"] == "jsonschema", "wrong channel encodings");
      const auto schema = Json::parse(channel["schema"].get<std::string>());
      check(schema["title"] == channel["schemaName"] && schema["type"] == "object", "invalid advertised schema");
    }
  }

  ~Client()
  {
    beast::error_code ignored;
    beast::get_lowest_layer(ws).socket().close(ignored);
  }

  Message read()
  {
    beast::flat_buffer buffer;
    asio::steady_timer deadline(io);
    deadline.expires_after(std::chrono::seconds(3));
    bool completed = false;
    beast::error_code error;
    deadline.async_wait([&](beast::error_code ec) {
      if (!ec && !completed) {
        beast::error_code ignored;
        beast::get_lowest_layer(ws).socket().cancel(ignored);
      }
    });
    ws.async_read(buffer, [&](beast::error_code ec, std::size_t) {
      completed = true;
      error = ec;
      deadline.cancel();
    });
    io.restart();
    io.run();
    if (error) throw beast::system_error(error);
    return {ws.got_binary(), beast::buffers_to_string(buffer.data())};
  }

  void send(const Json & value)
  {
    const auto text = value.dump();
    ws.text(true);
    ws.write(asio::buffer(text));
  }

  void subscribe(uint32_t offset = 0)
  {
    send({{"op", "subscribe"}, {"subscriptions", {
      {{"id", offset + 41}, {"channelId", 1}}, {{"id", offset + 42}, {"channelId", 2}},
      {{"id", offset + 43}, {"channelId", 3}}}}});
  }
};

uint64_t little_endian(const std::string & bytes, std::size_t start, unsigned count)
{
  uint64_t value = 0;
  for (unsigned i = 0; i < count; ++i) value |= uint64_t(static_cast<unsigned char>(bytes[start + i])) << (8 * i);
  return value;
}

std::vector<uchar> unbase64(const std::string & text)
{
  const std::string alphabet = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  std::vector<uchar> decoded;
  uint32_t value = 0;
  unsigned bits = 0;
  for (const auto c : text) {
    if (c == '=') break;
    const auto digit = alphabet.find(c);
    check(digit != std::string::npos, "invalid base64 image");
    value = (value << 6) | static_cast<uint32_t>(digit);
    bits += 6;
    if (bits >= 8) {
      bits -= 8;
      decoded.push_back(static_cast<uchar>((value >> bits) & 255));
    }
  }
  return decoded;
}

sim_bridge::DebugFrame frame(uint64_t time)
{
  sim_bridge::DebugFrame output;
  output.timestamp_ns = time;
  output.image = cv::Mat(32, 64, CV_8UC3, cv::Scalar(30, 100, 200));
  output.telemetry = {{"frame_seq", 7}, {"yaw", 0.25}, {"state_dim", 11}, {"tracking", true}};
  output.center = {3, 1, 0.2};
  output.has_target = true;
  output.measured = {{2.7, 1.1, 0.2}};
  output.predicted = {{2.7, 1.0, 0.1}, {3.15, 0.74, 0.2}, {3.15, 1.26, 0.3}};
  return output;
}

void protocol_and_payloads()
{
  sim_bridge::FoxgloveDebug server("127.0.0.1", 0, 100);
  check(server.port() != 0, "ephemeral port was not assigned");
  Client client(server.port());
  const auto metrics_schema = Json::parse(client.channels[0]["schema"].get<std::string>());
  check(metrics_schema["properties"]["vyaw"]["type"] == "number", "Plot needs declared numeric metrics");
  const auto image_schema = Json::parse(client.channels[1]["schema"].get<std::string>());
  check(image_schema["properties"]["data"]["contentEncoding"] == "base64", "missing image byte schema");
  const auto scene_schema = Json::parse(client.channels[2]["schema"].get<std::string>());
  check(scene_schema["properties"]["entities"]["items"]["properties"]["cubes"]["items"]["properties"].contains("pose"), "missing scene primitive schema");
  client.subscribe();
  constexpr uint64_t stamp = 1723456789123456789ULL;
  server.publish(frame(stamp));
  std::array<bool, 3> received{};
  for (unsigned i = 0; i < 3; ++i) {
    const auto message = client.read();
    check(message.binary && message.bytes.size() > 13, "expected binary data message");
    check(message.bytes[0] == 1, "wrong binary opcode");
    const auto id = little_endian(message.bytes, 1, 4);
    check(id >= 41 && id <= 43, "binary data must use client subscription ID");
    check(little_endian(message.bytes, 5, 8) == stamp, "timestamp must be little-endian uint64");
    const auto payload = Json::parse(message.bytes.substr(13));
    validate(payload, Json::parse(client.channels[id - 41]["schema"].get<std::string>()));
    received[id - 41] = true;
    if (id == 41) {
      check(payload["frame_seq"] == 7 && payload["yaw"] == 0.25 && payload["state_dim"] == 11, "wrong metrics payload");
      check(payload["timestamp"]["sec"] == stamp / 1000000000 && payload["timestamp"]["nsec"] == stamp % 1000000000, "wrong JSON timestamp");
    } else if (id == 42) {
      check(payload["format"] == "jpeg" && payload["frame_id"] == "camera_optical_frame", "wrong image payload");
      const auto jpeg = unbase64(payload["data"].get<std::string>());
      const auto image = cv::imdecode(jpeg, cv::IMREAD_COLOR);
      check(image.cols == 64 && image.rows == 32, "image is not valid JPEG");
    } else {
      check(payload["entities"].size() == 3, "missing scene groups");
      check(payload["entities"][0]["frame_id"] == "world", "scene must render without external TF");
      check(payload["entities"][0]["spheres"].size() == 1, "missing measurement point");
      check(payload["entities"][1]["cubes"].size() == 3, "missing predicted armor plates");
      check(payload["entities"][2]["spheres"].size() == 1, "missing target center");
    }
  }
  check(received[0] && received[1] && received[2], "not all subscribed channels delivered");

  client.send({{"op", "unsubscribe"}, {"subscriptionIds", {41, 42, 43}}});
  // TCP ordering ensures unsubscribe precedes the new subscription.
  client.send({{"op", "subscribe"}, {"subscriptions", {{{"id", 99}, {"channelId", 3}}}}});
  std::this_thread::sleep_for(std::chrono::milliseconds(20));
  auto lost = frame(stamp + 1);
  lost.has_target = false;
  lost.measured.clear();
  server.publish(std::move(lost));
  const auto cleared = client.read();
  check(little_endian(cleared.bytes, 1, 4) == 99, "unsubscribe or subscription ID reuse failed");
  const auto scene = Json::parse(cleared.bytes.substr(13));
  check(scene["entities"][1]["cubes"].empty() && scene["entities"][2]["spheres"].empty(), "target loss leaves stale scene primitives");
  client.ws.close(websocket::close_code::normal);
}

void bounded_publication_and_shutdown()
{
  auto server = std::make_unique<sim_bridge::FoxgloveDebug>("127.0.0.1", 0, 30);
  // With no client, even unsupported images never reach expensive JPEG encoding.
  auto bad_image = frame(1);
  bad_image.image = cv::Mat(16, 16, CV_8UC2);
  server->publish(std::move(bad_image));
  Client first(server->port()), second(server->port());
  first.subscribe();
  second.subscribe(100);
  std::this_thread::sleep_for(std::chrono::milliseconds(20));
  for (uint64_t i = 2; i < 200; ++i) server->publish(frame(i));
  // Latest frame is retained; the two clients keep independent subscription IDs.
  server->publish(frame(200));
  bool newest_first = false, newest_second = false;
  for (unsigned i = 0; i < 9 && (!newest_first || !newest_second); ++i) {
    if (!newest_first) {
      const auto message = first.read();
      const auto id = little_endian(message.bytes, 1, 4);
      check(id >= 41 && id <= 43, "first client received another client's IDs");
      newest_first = little_endian(message.bytes, 5, 8) == 200;
    }
    if (!newest_second) {
      const auto message = second.read();
      const auto id = little_endian(message.bytes, 1, 4);
      check(id >= 141 && id <= 143, "second client received another client's IDs");
      newest_second = little_endian(message.bytes, 5, 8) == 200;
    }
  }
  check(newest_first && newest_second, "newest frame was not delivered to both clients");
  // Leave clients connected and unread while the service shuts down.
  const auto start = Clock::now();
  server.reset();
  check(Clock::now() - start < std::chrono::seconds(2), "shutdown waited on clients");
}

void slow_client_and_partial_handshake_shutdown()
{
  auto server = std::make_unique<sim_bridge::FoxgloveDebug>("127.0.0.1", 0, 100);
  Client slow(server->port()), fast(server->port());
  slow.send({{"op", "subscribe"}, {"subscriptions", {{{"id", 77}, {"channelId", 2}}}}});
  fast.send({{"op", "subscribe"}, {"subscriptions", {{{"id", 88}, {"channelId", 1}}}}});
  cv::Mat image(720, 1280, CV_8UC3);
  cv::randu(image, cv::Scalar::all(0), cv::Scalar::all(255));
  auto producer = std::async(std::launch::async, [&] {
    for (uint64_t i = 1; i <= 100; ++i) {
      auto next = frame(i);
      next.image = image;
      server->publish(std::move(next));
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    auto final = frame(101);
    final.image = image;
    server->publish(std::move(final));
  });
  bool newest = false;
  for (unsigned i = 0; i < 100 && !newest; ++i) {
    const auto message = fast.read();
    check(little_endian(message.bytes, 1, 4) == 88, "slow-client isolation changed subscription ID");
    newest = little_endian(message.bytes, 5, 8) == 101;
  }
  producer.get();
  check(newest, "a client that never reads blocked another client");
  asio::io_context io;
  tcp::socket half_open(io);
  half_open.connect({asio::ip::make_address("127.0.0.1"), server->port()});
  const auto start = Clock::now();
  server.reset();
  check(Clock::now() - start < std::chrono::seconds(2), "shutdown blocked on incomplete handshake or slow client");
}

void reject_missing_subprotocol()
{
  sim_bridge::FoxgloveDebug server("127.0.0.1", 0);
  asio::io_context io;
  websocket::stream<beast::tcp_stream> ws(io);
  beast::get_lowest_layer(ws).connect({asio::ip::make_address("127.0.0.1"), server.port()});
  beast::error_code error;
  ws.handshake("127.0.0.1", "/", error);
  check(bool(error), "server accepted a client without foxglove.websocket.v1");
}

void independent_metrics_and_visual_rates()
{
  sim_bridge::FoxgloveDebug server("127.0.0.1", 0, 80, 10);
  Client client(server.port());
  client.subscribe();
  auto producer = std::async(std::launch::async, [&] {
    for (uint64_t i = 1; i <= 100; ++i) {
      server.publish(frame(i));
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    server.publish(frame(1001));
  });
  std::array<unsigned, 3> count{};
  std::array<bool, 3> final{};
  while (!(final[0] && final[1] && final[2])) {
    const auto message = client.read();
    const auto id = little_endian(message.bytes, 1, 4);
    check(id >= 41 && id <= 43, "rate test received unknown channel");
    const auto index = id - 41;
    ++count[index];
    const auto stamp = little_endian(message.bytes, 5, 8);
    const auto payload = Json::parse(message.bytes.substr(13));
    const auto & json_stamp = index == 2 ? payload.at("entities").at(0).at("timestamp") : payload.at("timestamp");
    check(json_stamp.at("sec").get<uint64_t>() * 1000000000ULL +
      json_stamp.at("nsec").get<uint64_t>() == stamp,
      "independent encoded channels lost their original timestamps");
    if (stamp == 1001) final[index] = true;
  }
  producer.get();
  check(count[0] >= 20, "metrics did not publish at the faster rate");
  check(count[1] >= 3 && count[2] >= 3, "visual channels did not publish periodically");
  check(count[0] > count[1] * 3 && count[0] > count[2] * 3,
    "image or scene followed the faster metrics rate");
}

void visual_only_retains_last_frame_until_its_deadline()
{
  sim_bridge::FoxgloveDebug server("127.0.0.1", 0, 80, 5);
  Client client(server.port());
  client.send({{"op", "subscribe"}, {"subscriptions", {
    {{"id", 42}, {"channelId", 2}}, {{"id", 43}, {"channelId", 3}}}}});
  server.publish(frame(1));
  for (unsigned i = 0; i < 2; ++i) {
    const auto message = client.read();
    check(little_endian(message.bytes, 5, 8) == 1, "initial visual frame missing");
  }
  server.publish(frame(2));
  // No further publish() calls: a metrics tick must not consume and discard the
  // only image/scene while waiting for their lower-rate deadline.
  for (unsigned i = 0; i < 2; ++i) {
    const auto message = client.read();
    const auto id = little_endian(message.bytes, 1, 4);
    check(id == 42 || id == 43, "visual-only subscription received metrics");
    check(little_endian(message.bytes, 5, 8) == 2, "last visual frame was dropped before its deadline");
  }
}

void invalid_rates_are_rejected()
{
  const auto nan = std::numeric_limits<double>::quiet_NaN();
  const auto infinity = std::numeric_limits<double>::infinity();
  for (const auto rate : {0.0, -1.0, nan, infinity, 241.0}) {
    bool rejected = false;
    try { sim_bridge::FoxgloveDebug server("127.0.0.1", 0, rate); }
    catch (const std::invalid_argument &) { rejected = true; }
    check(rejected, "invalid metrics rate was accepted");
  }
  for (const auto rate : {-1.0, nan, infinity, 241.0}) {
    bool rejected = false;
    try { sim_bridge::FoxgloveDebug server("127.0.0.1", 0, 50, rate); }
    catch (const std::invalid_argument &) { rejected = true; }
    check(rejected, "invalid visual rate was accepted");
  }
}
}  // namespace

int main()
{
  try {
    protocol_and_payloads();
    bounded_publication_and_shutdown();
    slow_client_and_partial_handshake_shutdown();
    reject_missing_subprotocol();
    independent_metrics_and_visual_rates();
    visual_only_retains_last_frame_until_its_deadline();
    invalid_rates_are_rejected();
    std::cout << "Foxglove protocol, schemas, JPEG, scene, independent rates, multi-client, slow-client, and shutdown tests passed\n";
    return 0;
  } catch (const std::exception & error) {
    std::cerr << "Foxglove test failed: " << error.what() << '\n';
    return 1;
  }
}
