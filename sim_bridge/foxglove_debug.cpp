#include "foxglove_debug.hpp"

#include <Eigen/Geometry>
#include <boost/asio.hpp>
#include <boost/beast/core.hpp>
#include <boost/beast/http.hpp>
#include <boost/beast/websocket.hpp>
#include <opencv2/imgcodecs.hpp>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <deque>
#include <mutex>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <thread>
#include <unordered_map>

namespace sim_bridge
{
namespace
{
namespace asio = boost::asio;
namespace beast = boost::beast;
namespace websocket = beast::websocket;
namespace http = beast::http;
using tcp = asio::ip::tcp;
using Json = nlohmann::json;
using Clock = std::chrono::steady_clock;
constexpr std::size_t CHANNEL_COUNT = 3;
constexpr std::size_t MAX_CLIENTS = 16;
constexpr std::size_t MAX_MESSAGE_BYTES = 8 * 1024 * 1024;
constexpr char SUBPROTOCOL[] = "foxglove.websocket.v1";

Json object_schema(Json properties, Json required = Json::array())
{
  return {{"type", "object"}, {"properties", std::move(properties)}, {"required", std::move(required)}};
}

Json vector_schema(const std::string & title, std::initializer_list<const char *> axes)
{
  Json properties = Json::object(), required = Json::array();
  for (const auto * axis : axes) {
    properties[axis] = {{"type", "number"}};
    required.push_back(axis);
  }
  auto result = object_schema(std::move(properties), std::move(required));
  result["title"] = title;
  return result;
}

Json time_schema(const char * title)
{
  auto result = object_schema(
    {{"sec", {{"type", "integer"}}},
     {"nsec", {{"type", "integer"}, {"minimum", 0}, {"maximum", 999999999}}}},
    {"sec", "nsec"});
  result["title"] = title;
  return result;
}

// Foxglove JSON schemas use inline nested schemas, time/duration titles, and
// contentEncoding=base64 for byte arrays. These definitions cover emitted data.
// References: https://docs.foxglove.dev/docs/sdk/schemas/
// https://github.com/foxglove/schemas/tree/main/schemas/jsonschema
Json image_schema()
{
  auto schema = object_schema(
    {{"timestamp", time_schema("time")}, {"frame_id", {{"type", "string"}}},
     {"data", {{"type", "string"}, {"contentEncoding", "base64"}}},
     {"format", {{"type", "string"}}}},
    {"timestamp", "frame_id", "data", "format"});
  schema["title"] = "foxglove.CompressedImage";
  return schema;
}

Json scene_schema()
{
  const auto vec = vector_schema("foxglove.Vector3", {"x", "y", "z"});
  auto pose = object_schema(
    {{"position", vec}, {"orientation", vector_schema("foxglove.Quaternion", {"x", "y", "z", "w"})}},
    {"position", "orientation"});
  pose["title"] = "foxglove.Pose";
  auto primitive = object_schema(
    {{"pose", pose}, {"size", vec}, {"color", vector_schema("foxglove.Color", {"r", "g", "b", "a"})}},
    {"pose", "size", "color"});
  Json properties = {
    {"timestamp", time_schema("time")}, {"frame_id", {{"type", "string"}}},
    {"id", {{"type", "string"}}}, {"lifetime", time_schema("duration")},
    {"frame_locked", {{"type", "boolean"}}}};
  for (const auto * name : {"metadata", "arrows", "cubes", "spheres", "cylinders", "lines", "triangles", "texts", "models"}) {
    properties[name] = {{"type", "array"}, {"items", object_schema(Json::object())}};
  }
  primitive["title"] = "foxglove.CubePrimitive";
  properties["cubes"]["items"] = primitive;
  primitive["title"] = "foxglove.SpherePrimitive";
  properties["spheres"]["items"] = primitive;
  auto entity = object_schema(std::move(properties), {"timestamp", "frame_id", "id", "lifetime", "frame_locked", "metadata", "arrows", "cubes", "spheres", "cylinders", "lines", "triangles", "texts", "models"});
  entity["title"] = "foxglove.SceneEntity";
  auto deletion = object_schema(
    {{"timestamp", time_schema("time")}, {"type", {{"type", "integer"}, {"enum", {0, 1}}}},
     {"id", {{"type", "string"}}}}, {"timestamp", "type", "id"});
  auto result = object_schema(
    {{"deletions", {{"type", "array"}, {"items", deletion}}},
     {"entities", {{"type", "array"}, {"items", entity}}}}, {"deletions", "entities"});
  result["title"] = "foxglove.SceneUpdate";
  return result;
}

Json metrics_schema()
{
  Json properties = {{"timestamp", time_schema("time")}};
  for (const auto * name : {
    "timestamp_ns", "frame_seq", "generation", "frame_age_ms", "receive_to_process_ms",
    "processing_ms", "loop_ms", "fps", "received", "replaced", "mismatch", "reconnects",
    "clock_resets", "detected", "raw_detected", "accepted", "state_dim", "center_x", "center_y", "center_z",
    "yaw", "vyaw", "radius", "z0", "z1", "z2", "vz", "dh1", "dh2", "nis", "nis_failures",
    "command_yaw", "command_pitch", "prediction_horizon_ms", "debug_dropped", "update_count",
    "stale", "command_age_ms", "frame_age_p50_ms", "frame_age_p95_ms", "frame_age_p99_ms", "processing_p95_ms",
    "fire_yaw_error_deg", "fire_pitch_error_deg", "fire_cooldown_ms", "fire_stable_ms",
    "shots_requested", "launch_count", "hit_count", "gun_yaw", "gun_pitch", "aim_id",
    "target_yaw", "target_pitch", "yaw_vel", "pitch_vel", "yaw_acc", "pitch_acc",
    "aim_visibility_deg", "plan_tracking_error", "distance", "fly_time_ms", "max_frame_age_ms",
    "fire_interval_ms", "fire_yaw_tolerance_deg", "fire_pitch_tolerance_deg",
    "planning_ms", "planning_estimate_ms", "time"}) {
    properties[name] = {{"type", "number"}};
  }
  for (const auto * name : {"tracking", "converged", "synthetic", "connected", "command_sent",
    "planned_control", "solver_ok", "planner_fire", "fire", "following", "control_enabled",
    "fire_enabled", "measurement_valid"}) {
    properties[name] = {{"type", "boolean"}};
  }
  properties["tracker_state"] = {{"type", "string"}};
  properties["fire_reason"] = {{"type", "string"}};
  properties["aim_xyza"] = {{"type", "array"}, {"items", {{"type", "number"}}}};
  properties["raw_confidences"] = {{"type", "array"}, {"items", {{"type", "number"}}}};
  properties["raw_names"] = {{"type", "array"}, {"items", {{"type", "string"}}}};
  properties["raw_colors"] = {{"type", "array"}, {"items", {{"type", "string"}}}};
  Json angles = {{"valid", {{"type", "boolean"}}}};
  for (const auto *name : {"yaw", "pitch", "yaw_deg", "pitch_deg", "yaw_unwrapped", "yaw_unwrapped_deg"})
    angles[name] = {{"type", "number"}};
  properties["gimbal"] = object_schema(angles, {"valid"});
  Json reference = angles;
  reference["state_valid"] = {{"type", "boolean"}};
  for (const auto *name : {"phase", "phase_deg", "phase_unwrapped", "phase_unwrapped_deg", "phase_rate"})
    reference[name] = {{"type", "number"}};
  properties["target"] = object_schema(reference, {"valid", "state_valid"});
  Json planned = angles;
  for (const auto *name : {"yaw_vel", "pitch_vel", "yaw_acc", "pitch_acc"})
    planned[name] = {{"type", "number"}};
  properties["plan"] = object_schema(planned, {"valid"});
  properties["error"] = object_schema({{"valid", {{"type", "boolean"}}},
    {"yaw", {{"type", "number"}}}, {"pitch", {{"type", "number"}}},
    {"yaw_deg", {{"type", "number"}}}, {"pitch_deg", {{"type", "number"}}}}, {"valid"});
  properties["P_diag"] = {{"type", "array"}, {"items", {{"type", "number"}}}};
  properties["ekf_x"] = {{"type", "array"}, {"items", {{"type", "number"}}}};
  auto schema = object_schema(std::move(properties));
  schema["title"] = "auto_aim.DebugMetrics";
  schema["additionalProperties"] = true;
  return schema;
}

Json advertisement()
{
  Json channels = Json::array();
  const std::array<const char *, CHANNEL_COUNT> topics{
    "/auto_aim/metrics", "/auto_aim/image", "/auto_aim/scene"};
  const std::array<const char *, CHANNEL_COUNT> names{
    "auto_aim.DebugMetrics", "foxglove.CompressedImage", "foxglove.SceneUpdate"};
  const std::array<Json, CHANNEL_COUNT> schemas{metrics_schema(), image_schema(), scene_schema()};
  for (std::size_t i = 0; i < CHANNEL_COUNT; ++i) {
    channels.push_back({{"id", i + 1}, {"topic", topics[i]}, {"encoding", "json"},
      {"schemaName", names[i]}, {"schemaEncoding", "jsonschema"}, {"schema", schemas[i].dump()}});
  }
  return {{"op", "advertise"}, {"channels", std::move(channels)}};
}

Json timestamp(uint64_t ns) { return {{"sec", ns / 1000000000}, {"nsec", ns % 1000000000}}; }
Json xyz(const Eigen::Vector3d & p) { return {{"x", p.x()}, {"y", p.y()}, {"z", p.z()}}; }
Json rgba(double r, double g, double b) { return {{"r", r}, {"g", g}, {"b", b}, {"a", 1.0}}; }

Json make_entity(const char * id, uint64_t ns)
{
  Json entity = {{"timestamp", timestamp(ns)}, {"frame_id", "world"}, {"id", id},
    {"lifetime", {{"sec", 0}, {"nsec", 0}}}, {"frame_locked", false}};
  for (const auto * name : {"metadata", "arrows", "cubes", "spheres", "cylinders", "lines", "triangles", "texts", "models"}) {
    entity[name] = Json::array();
  }
  return entity;
}

Json make_primitive(const Eigen::Vector3d & p, const Eigen::Vector3d & size, Json color,
                    const Eigen::Quaterniond & q = Eigen::Quaterniond::Identity())
{
  return {{"pose", {{"position", xyz(p)}, {"orientation", {{"x", q.x()}, {"y", q.y()}, {"z", q.z()}, {"w", q.w()}}}}},
    {"size", xyz(size)}, {"color", std::move(color)}};
}

Json make_scene(const DebugFrame & frame)
{
  auto measured = make_entity("measured", frame.timestamp_ns);
  auto predicted = make_entity("predicted", frame.timestamp_ns);
  auto center = make_entity("center", frame.timestamp_ns);
  for (std::size_t i = 0; i < std::min<std::size_t>(frame.measured.size(), 64); ++i) {
    const auto & p = frame.measured[i];
    if (p.allFinite()) measured["spheres"].push_back(make_primitive(p, Eigen::Vector3d::Constant(0.04), rgba(1.0, 0.4, 0.1)));
  }
  if (frame.has_target && frame.center.allFinite()) {
    center["spheres"].push_back(make_primitive(frame.center, Eigen::Vector3d::Constant(0.05), rgba(0.1, 0.7, 1.0)));
    for (std::size_t i = 0; i < std::min<std::size_t>(frame.predicted.size(), 3); ++i) {
      const auto & p = frame.predicted[i];
      if (!p.allFinite()) continue;
      const auto direction = frame.center - p;
      const auto yaw = std::atan2(direction.y(), direction.x());
      const Eigen::Quaterniond q = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(-15.0 * 3.141592653589793 / 180.0, Eigen::Vector3d::UnitY());
      predicted["cubes"].push_back(make_primitive(p, {0.012, 0.135, 0.056}, rgba(0.1, 1.0, 0.3), q));
    }
  }
  // Stable IDs replace all three groups, including empty groups after target loss.
  return {{"deletions", Json::array()}, {"entities", {measured, predicted, center}}};
}

std::string base64(const std::vector<uchar> & bytes)
{
  constexpr char alphabet[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  std::string out;
  out.reserve((bytes.size() + 2) / 3 * 4);
  for (std::size_t i = 0; i < bytes.size(); i += 3) {
    const uint32_t bits = uint32_t(bytes[i]) << 16 |
      (i + 1 < bytes.size() ? uint32_t(bytes[i + 1]) << 8 : 0) |
      (i + 2 < bytes.size() ? uint32_t(bytes[i + 2]) : 0);
    out.push_back(alphabet[(bits >> 18) & 63]);
    out.push_back(alphabet[(bits >> 12) & 63]);
    out.push_back(i + 1 < bytes.size() ? alphabet[(bits >> 6) & 63] : '=');
    out.push_back(i + 2 < bytes.size() ? alphabet[bits & 63] : '=');
  }
  return out;
}

struct EncodedFrame
{
  uint64_t timestamp_ns = 0;
  std::array<std::shared_ptr<const std::string>, CHANNEL_COUNT> payloads;
};

std::shared_ptr<const std::string> binary_message(uint32_t id, uint64_t ns, const std::string & payload)
{
  auto message = std::make_shared<std::string>(13, '\0');
  (*message)[0] = 1;
  for (unsigned i = 0; i < 4; ++i) (*message)[1 + i] = static_cast<char>((id >> (8 * i)) & 255);
  for (unsigned i = 0; i < 8; ++i) (*message)[5 + i] = static_cast<char>((ns >> (8 * i)) & 255);
  message->append(payload);
  return message;
}

bool protocol_offered(beast::string_view offered)
{
  std::string token;
  std::istringstream tokens{std::string(offered)};
  while (std::getline(tokens, token, ',')) {
    const auto start = token.find_first_not_of(" \t");
    const auto end = token.find_last_not_of(" \t");
    if (start != std::string::npos && token.substr(start, end - start + 1) == SUBPROTOCOL) return true;
  }
  return false;
}

bool get_u32(const Json & value, uint32_t & result)
{
  if (!value.is_number_unsigned() && !value.is_number_integer()) return false;
  if (value.is_number_integer() && !value.is_number_unsigned() && value.get<int64_t>() < 0) return false;
  const auto number = value.get<uint64_t>();
  if (number > UINT32_MAX) return false;
  result = static_cast<uint32_t>(number);
  return true;
}
}  // namespace

struct FoxgloveDebug::Impl
{
  struct Session : std::enable_shared_from_this<Session>
  {
    Impl & owner;
    websocket::stream<beast::tcp_stream> ws;
    beast::flat_buffer buffer;
    http::request<http::string_body> request;
    std::array<std::optional<uint32_t>, CHANNEL_COUNT> subscriptions;
    std::array<std::shared_ptr<const std::string>, CHANNEL_COUNT> pending;
    std::deque<std::shared_ptr<const std::string>> controls;
    std::shared_ptr<const std::string> active;
    std::size_t next_channel = 0;
    bool closed = false;

    Session(Impl & impl, tcp::socket socket) : owner(impl), ws(std::move(socket)) {}

    void start()
    {
      beast::get_lowest_layer(ws).expires_after(std::chrono::seconds(3));
      http::async_read(ws.next_layer(), buffer, request,
        [self = shared_from_this()](beast::error_code ec, std::size_t) {
          if (ec || !websocket::is_upgrade(self->request) ||
              !protocol_offered(self->request[http::field::sec_websocket_protocol])) return self->shutdown();
          beast::get_lowest_layer(self->ws).expires_never();
          self->ws.set_option(websocket::stream_base::timeout::suggested(beast::role_type::server));
          self->ws.set_option(websocket::stream_base::decorator([](websocket::response_type & response) {
            response.set(http::field::sec_websocket_protocol, SUBPROTOCOL);
          }));
          self->ws.read_message_max(65536);
          self->ws.async_accept(self->request, [self](beast::error_code error) {
            if (error) return self->shutdown();
            self->controls.push_back(std::make_shared<const std::string>(Json{
              {"op", "serverInfo"}, {"name", "RoboMaster auto aim debug"}, {"capabilities", Json::array()}}.dump()));
            self->controls.push_back(self->owner.advertise);
            self->write();
            self->read();
          });
        });
    }

    void read()
    {
      ws.async_read(buffer, [self = shared_from_this()](beast::error_code ec, std::size_t) {
        if (ec) return self->shutdown();
        if (self->ws.got_text()) self->handle(Json::parse(beast::buffers_to_string(self->buffer.data()), nullptr, false));
        self->buffer.consume(self->buffer.size());
        if (!self->closed) self->read();
      });
    }

    void handle(const Json & message)
    {
      if (!message.is_object() || !message.contains("op") || !message["op"].is_string()) return;
      const auto & op = message["op"];
      if (op == "subscribe" && message.contains("subscriptions") && message["subscriptions"].is_array()) {
        for (const auto & item : message["subscriptions"]) {
          uint32_t id, channel;
          if (!item.is_object() || !item.contains("id") || !item.contains("channelId") ||
              !get_u32(item["id"], id) || !get_u32(item["channelId"], channel) ||
              channel == 0 || channel > CHANNEL_COUNT) continue;
          const bool reused = std::any_of(subscriptions.begin(), subscriptions.end(),
            [id](const auto & subscription) { return subscription && *subscription == id; });
          if (!reused && !subscriptions[channel - 1]) subscriptions[channel - 1] = id;
        }
      } else if (op == "unsubscribe" && message.contains("subscriptionIds") && message["subscriptionIds"].is_array()) {
        for (const auto & value : message["subscriptionIds"]) {
          uint32_t id;
          if (!get_u32(value, id)) continue;
          for (std::size_t i = 0; i < CHANNEL_COUNT; ++i) {
            if (subscriptions[i] && *subscriptions[i] == id) {
              subscriptions[i].reset();
              pending[i].reset();
            }
          }
        }
      }
      owner.update_interest();
    }

    void offer(const EncodedFrame & frame, unsigned mask)
    {
      if (closed) return;
      for (std::size_t i = 0; i < CHANNEL_COUNT; ++i) {
        if ((mask & (1U << i)) && subscriptions[i] && frame.payloads[i]) {
          pending[i] = binary_message(*subscriptions[i], frame.timestamp_ns, *frame.payloads[i]);
        }
      }
      write();
    }

    void write()
    {
      if (closed || active) return;
      bool binary = false;
      if (!controls.empty()) {
        active = std::move(controls.front());
        controls.pop_front();
      } else {
        for (std::size_t step = 0; step < CHANNEL_COUNT; ++step) {
          const auto channel = (next_channel + step) % CHANNEL_COUNT;
          if (!pending[channel]) continue;
          active = std::move(pending[channel]);
          next_channel = (channel + 1) % CHANNEL_COUNT;
          binary = true;
          break;
        }
      }
      if (!active) return;
      ws.binary(binary);
      ws.async_write(asio::buffer(*active), [self = shared_from_this()](beast::error_code ec, std::size_t) {
        self->active.reset();
        if (ec) return self->shutdown();
        self->write();
      });
    }

    void shutdown()
    {
      if (closed) return;
      auto keep_alive = shared_from_this();
      closed = true;
      beast::error_code ignored;
      beast::get_lowest_layer(ws).socket().cancel(ignored);
      beast::get_lowest_layer(ws).socket().shutdown(tcp::socket::shutdown_both, ignored);
      beast::get_lowest_layer(ws).socket().close(ignored);
      owner.sessions.erase(this);
      owner.update_interest();
    }
  };

  asio::io_context io;
  tcp::acceptor acceptor;
  asio::steady_timer drain_timer;
  std::unordered_map<Session *, std::shared_ptr<Session>> sessions;
  const std::shared_ptr<const std::string> advertise;
  const Clock::duration period, visual_period;
  uint16_t listening_port;
  std::atomic<bool> stopping{false};
  std::atomic<unsigned> interest{0};
  std::atomic<uint64_t> dropped{0};
  std::mutex mutex;
  std::condition_variable condition;
  std::optional<DebugFrame> latest;
  unsigned latest_pending = 0;
  std::array<std::shared_ptr<const EncodedFrame>, CHANNEL_COUNT> encoded;
  std::thread network_thread, encoding_thread;

  Impl(const std::string & host, uint16_t port, double rate, double visual_rate)
  : acceptor(io), drain_timer(io), advertise(std::make_shared<const std::string>(advertisement().dump())),
    period(std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(1.0 / checked_rate(rate)))),
    visual_period(std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(
      1.0 / checked_rate(visual_rate == 0.0 ? rate : visual_rate))))
  {
    const auto endpoint = tcp::endpoint(asio::ip::make_address(host), port);
    acceptor.open(endpoint.protocol());
    acceptor.set_option(tcp::acceptor::reuse_address(true));
    acceptor.bind(endpoint);
    acceptor.listen();
    listening_port = acceptor.local_endpoint().port();
    accept();
    drain();
    network_thread = std::thread([this] { io.run(); });
    try {
      encoding_thread = std::thread([this] { encode_loop(); });
    } catch (...) {
      io.stop();
      network_thread.join();
      throw;
    }
  }

  static double checked_rate(double rate)
  {
    if (!std::isfinite(rate) || rate <= 0 || rate > 240) throw std::invalid_argument("Foxglove rate_hz must be in (0, 240]");
    return rate;
  }

  ~Impl()
  {
    stopping.store(true);
    condition.notify_all();
    if (encoding_thread.joinable()) encoding_thread.join();
    asio::post(io, [this] {
      beast::error_code ignored;
      acceptor.cancel(ignored);
      acceptor.close(ignored);
      drain_timer.cancel(ignored);
      while (!sessions.empty()) sessions.begin()->second->shutdown();
      io.stop();
    });
    if (network_thread.joinable()) network_thread.join();
  }

  void accept()
  {
    acceptor.async_accept([this](beast::error_code ec, tcp::socket socket) {
      if (!ec && !stopping.load() && sessions.size() < MAX_CLIENTS) {
        auto session = std::make_shared<Session>(*this, std::move(socket));
        sessions.emplace(session.get(), session);
        session->start();
      }
      if (!stopping.load() && acceptor.is_open()) accept();
    });
  }

  void update_interest()
  {
    unsigned mask = 0;
    for (const auto & entry : sessions) {
      for (unsigned i = 0; i < CHANNEL_COUNT; ++i) if (entry.second->subscriptions[i]) mask |= 1U << i;
    }
    {
      std::lock_guard<std::mutex> lock(mutex);
      interest.store(mask);
    }
    condition.notify_one();
  }

  void drain()
  {
    drain_timer.expires_after(std::chrono::milliseconds(5));
    drain_timer.async_wait([this](beast::error_code ec) {
      if (ec || stopping.load()) return;
      std::array<std::shared_ptr<const EncodedFrame>, CHANNEL_COUNT> frames;
      {
        std::lock_guard<std::mutex> lock(mutex);
        frames.swap(encoded);
      }
      for (unsigned i = 0; i < CHANNEL_COUNT; ++i) {
        if (frames[i]) for (const auto & entry : sessions) entry.second->offer(*frames[i], 1U << i);
      }
      drain();
    });
  }

  void publish(DebugFrame frame)
  {
    // No image clone, encoding, socket operation, or frame queue on this path.
    std::optional<DebugFrame> incoming(std::move(frame));
    {
      std::lock_guard<std::mutex> lock(mutex);
      latest.swap(incoming);
      latest_pending = 7U;
    }
    if (incoming) dropped.fetch_add(1, std::memory_order_relaxed);
    condition.notify_one();
  }

  void encode_loop()
  {
    auto next_metrics = Clock::now(), next_visual = next_metrics;
    while (!stopping.load()) {
      std::optional<DebugFrame> frame;
      unsigned mask;
      {
        std::unique_lock<std::mutex> lock(mutex);
        condition.wait(lock, [this] {
          return stopping.load() || (latest && (latest_pending & interest.load()) != 0);
        });
        if (stopping.load()) return;
        const auto now = Clock::now();
        const auto wanted = latest_pending & interest.load();
        mask = (now >= next_metrics ? wanted & 1U : 0U) |
          (now >= next_visual ? wanted & 6U : 0U);
        if (!mask) {
          const auto deadline = std::min(wanted & 1U ? next_metrics : Clock::time_point::max(),
            wanted & 6U ? next_visual : Clock::time_point::max());
          condition.wait_until(lock, deadline);
          continue;
        }
        latest_pending = wanted & ~mask;
        if (!latest_pending) {
          frame.swap(latest);
        } else {
          // Keep only channels that a subscriber still needs. In particular, a
          // metrics tick must retain the final visual frame until its deadline.
          frame.emplace();
          frame->timestamp_ns = latest->timestamp_ns;
          if (mask & 1U) frame->telemetry = std::move(latest->telemetry);
          if (mask & 2U) frame->image = std::move(latest->image);
          if (mask & 4U) {
            frame->measured = std::move(latest->measured);
            frame->predicted = std::move(latest->predicted);
            frame->center = latest->center;
            frame->has_target = latest->has_target;
          }
        }
      }
      const auto now = Clock::now();
      if (mask & 1U) next_metrics = now + period;
      if (mask & 6U) next_visual = now + visual_period;
      auto result = std::make_shared<EncodedFrame>();
      result->timestamp_ns = frame->timestamp_ns;
      try {
        if (mask & 1U) {
          auto metrics = std::move(frame->telemetry);
          if (!metrics.is_object()) metrics = Json::object();
          metrics["timestamp"] = timestamp(frame->timestamp_ns);
          metrics["timestamp_ns"] = frame->timestamp_ns;
          metrics["debug_dropped"] = dropped.load(std::memory_order_relaxed);
          result->payloads[0] = std::make_shared<const std::string>(metrics.dump());
        }
        if ((mask & 2U) && !frame->image.empty()) {
          std::vector<uchar> jpeg;
          if (cv::imencode(".jpg", frame->image, jpeg, {cv::IMWRITE_JPEG_QUALITY, 80}) && jpeg.size() < MAX_MESSAGE_BYTES / 2) {
            result->payloads[1] = std::make_shared<const std::string>(Json{
              {"timestamp", timestamp(frame->timestamp_ns)}, {"frame_id", "camera_optical_frame"},
              {"format", "jpeg"}, {"data", base64(jpeg)}}.dump());
          }
        }
        if (mask & 4U) result->payloads[2] = std::make_shared<const std::string>(make_scene(*frame).dump());
      } catch (const std::exception &) {
        // A malformed debug image must never terminate inference or the server.
      }
      for (auto & payload : result->payloads) if (payload && payload->size() > MAX_MESSAGE_BYTES) payload.reset();
      {
        std::lock_guard<std::mutex> lock(mutex);
        for (unsigned i = 0; i < CHANNEL_COUNT; ++i) if (result->payloads[i]) encoded[i] = result;
      }
    }
  }
};

FoxgloveDebug::FoxgloveDebug(std::string host, uint16_t port, double rate_hz, double visual_rate_hz)
: impl_(std::make_unique<Impl>(host, port, rate_hz, visual_rate_hz)) {}
FoxgloveDebug::~FoxgloveDebug() = default;
void FoxgloveDebug::publish(DebugFrame frame) { impl_->publish(std::move(frame)); }
uint16_t FoxgloveDebug::port() const { return impl_->listening_port; }

}  // namespace sim_bridge
