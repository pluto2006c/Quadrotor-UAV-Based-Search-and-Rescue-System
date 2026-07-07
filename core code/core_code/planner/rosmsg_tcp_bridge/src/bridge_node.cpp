#include <ros/ros.h>
#include <ros/serialization.h>

#include <nav_msgs/Odometry.h>
#include <std_msgs/Empty.h>
#include <traj_utils/Bspline.h>
#include <traj_utils/MultiBsplines.h>

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <exception>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

namespace
{

constexpr uint8_t kFrameMagic[4] = {'N', 'L', 'K', '1'};
constexpr uint8_t kFrameVersion = 1;
constexpr size_t kPayloadTypeBytes = sizeof(int32_t);
constexpr size_t kMaxPayloadBytes = 1024 * 1024;

enum MessageType : int32_t
{
  ODOM = 888,
  MULTI_TRAJ,
  ONE_TRAJ,
  STOP
};

uint32_t checksumPayload(const std::vector<uint8_t> &payload)
{
  uint32_t value = 2166136261u;
  for (uint8_t byte : payload)
  {
    value ^= byte;
    value *= 16777619u;
  }
  return value;
}

void appendUint32LE(std::vector<uint8_t> &out, uint32_t value)
{
  out.push_back(static_cast<uint8_t>(value & 0xffu));
  out.push_back(static_cast<uint8_t>((value >> 8u) & 0xffu));
  out.push_back(static_cast<uint8_t>((value >> 16u) & 0xffu));
  out.push_back(static_cast<uint8_t>((value >> 24u) & 0xffu));
}

void appendInt32LE(std::vector<uint8_t> &out, int32_t value)
{
  appendUint32LE(out, static_cast<uint32_t>(value));
}

uint32_t readUint32LE(const uint8_t *data)
{
  return static_cast<uint32_t>(data[0]) |
         (static_cast<uint32_t>(data[1]) << 8u) |
         (static_cast<uint32_t>(data[2]) << 16u) |
         (static_cast<uint32_t>(data[3]) << 24u);
}

int32_t readInt32LE(const uint8_t *data)
{
  return static_cast<int32_t>(readUint32LE(data));
}

bool baudToSpeed(int baudrate, speed_t *speed)
{
  switch (baudrate)
  {
  case 9600:
    *speed = B9600;
    return true;
  case 19200:
    *speed = B19200;
    return true;
  case 38400:
    *speed = B38400;
    return true;
  case 57600:
    *speed = B57600;
    return true;
  case 115200:
    *speed = B115200;
    return true;
#ifdef B230400
  case 230400:
    *speed = B230400;
    return true;
#endif
#ifdef B460800
  case 460800:
    *speed = B460800;
    return true;
#endif
#ifdef B921600
  case 921600:
    *speed = B921600;
    return true;
#endif
#ifdef B1000000
  case 1000000:
    *speed = B1000000;
    return true;
#endif
#ifdef B1500000
  case 1500000:
    *speed = B1500000;
    return true;
#endif
  default:
    return false;
  }
}

template <typename MessageT>
std::vector<uint8_t> serializePayload(MessageType type, const MessageT &msg)
{
  const uint32_t body_len = ros::serialization::serializationLength(msg);
  std::vector<uint8_t> payload;
  payload.reserve(kPayloadTypeBytes + body_len);
  appendInt32LE(payload, static_cast<int32_t>(type));
  payload.resize(kPayloadTypeBytes + body_len);

  if (body_len > 0)
  {
    ros::serialization::OStream stream(payload.data() + kPayloadTypeBytes, body_len);
    ros::serialization::serialize(stream, msg);
  }

  return payload;
}

template <typename MessageT>
bool deserializeBody(const std::vector<uint8_t> &payload, MessageT *msg)
{
  if (payload.size() < kPayloadTypeBytes)
  {
    return false;
  }

  const uint32_t body_len = static_cast<uint32_t>(payload.size() - kPayloadTypeBytes);
  try
  {
    ros::serialization::IStream stream(const_cast<uint8_t *>(payload.data() + kPayloadTypeBytes), body_len);
    ros::serialization::deserialize(stream, *msg);
  }
  catch (const std::exception &e)
  {
    ROS_ERROR("[rosmsg_tcp_bridge] ROS message deserialize failed: %s", e.what());
    return false;
  }
  return true;
}

class NearlinkSerial
{
public:
  NearlinkSerial() = default;
  ~NearlinkSerial()
  {
    closePort();
  }

  NearlinkSerial(const NearlinkSerial &) = delete;
  NearlinkSerial &operator=(const NearlinkSerial &) = delete;

  void setTxOptions(size_t chunk_bytes, int chunk_gap_us, int peer_id)
  {
    tx_chunk_bytes_ = chunk_bytes;
    chunk_gap_us_ = std::max(0, chunk_gap_us);
    peer_id_ = peer_id;
  }

  bool openPort(const std::string &device, int baudrate)
  {
    speed_t speed = B115200;
    if (!baudToSpeed(baudrate, &speed))
    {
      ROS_ERROR("[rosmsg_tcp_bridge] Unsupported NearLink UART baudrate: %d", baudrate);
      return false;
    }

    int fd = ::open(device.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0)
    {
      ROS_ERROR("[rosmsg_tcp_bridge] Failed to open NearLink UART %s: %s",
                device.c_str(), std::strerror(errno));
      return false;
    }

    termios options;
    if (tcgetattr(fd, &options) != 0)
    {
      ROS_ERROR("[rosmsg_tcp_bridge] Failed to read UART attributes: %s", std::strerror(errno));
      ::close(fd);
      return false;
    }

    cfmakeraw(&options);
    cfsetispeed(&options, speed);
    cfsetospeed(&options, speed);

    options.c_cflag |= (CLOCAL | CREAD);
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
#ifdef CRTSCTS
    options.c_cflag &= ~CRTSCTS;
#endif
    options.c_cc[VMIN] = 0;
    options.c_cc[VTIME] = 1;

    if (tcsetattr(fd, TCSANOW, &options) != 0)
    {
      ROS_ERROR("[rosmsg_tcp_bridge] Failed to configure NearLink UART: %s", std::strerror(errno));
      ::close(fd);
      return false;
    }

    tcflush(fd, TCIOFLUSH);
    fd_ = fd;
    ROS_INFO("[rosmsg_tcp_bridge] NearLink UART opened: %s @ %d", device.c_str(), baudrate);
    return true;
  }

  void closePort()
  {
    const int fd = fd_;
    fd_ = -1;
    if (fd >= 0)
    {
      ::close(fd);
    }
  }

  bool isOpen() const
  {
    return fd_ >= 0;
  }

  bool sendFrame(const std::vector<uint8_t> &payload)
  {
    if (payload.empty() || payload.size() > kMaxPayloadBytes)
    {
      ROS_ERROR("[rosmsg_tcp_bridge] Refuse to send invalid NearLink payload length: %zu", payload.size());
      return false;
    }

    std::vector<uint8_t> frame;
    frame.reserve(14 + payload.size());
    frame.insert(frame.end(), kFrameMagic, kFrameMagic + sizeof(kFrameMagic));
    frame.push_back(kFrameVersion);
    frame.push_back(0);
    appendUint32LE(frame, static_cast<uint32_t>(payload.size()));
    appendUint32LE(frame, checksumPayload(payload));
    frame.insert(frame.end(), payload.begin(), payload.end());

    std::lock_guard<std::mutex> lock(write_mutex_);
    if (fd_ < 0)
    {
      ROS_ERROR_THROTTLE(1.0, "[rosmsg_tcp_bridge] NearLink UART is not open");
      return false;
    }

    const size_t chunk_payload_bytes = tx_chunk_bytes_ == 0 ? frame.size() : tx_chunk_bytes_;
    size_t offset = 0;
    while (offset < frame.size())
    {
      const size_t bytes_this_chunk = std::min(chunk_payload_bytes, frame.size() - offset);
      std::vector<uint8_t> chunk;
      if (peer_id_ >= 0)
      {
        chunk.push_back(static_cast<uint8_t>('0' + peer_id_));
      }
      chunk.insert(chunk.end(), frame.begin() + offset, frame.begin() + offset + bytes_this_chunk);

      if (!writeAll(chunk.data(), chunk.size()))
      {
        return false;
      }

      offset += bytes_this_chunk;
      if (chunk_gap_us_ > 0 && offset < frame.size())
      {
        tcdrain(fd_);
        usleep(static_cast<useconds_t>(chunk_gap_us_));
      }
    }

    return true;
  }

  bool readFrame(std::vector<uint8_t> *payload)
  {
    payload->clear();

    size_t matched_magic = 0;
    uint8_t byte = 0;
    while (ros::ok())
    {
      if (!readExact(&byte, 1))
      {
        return false;
      }

      if (byte == kFrameMagic[matched_magic])
      {
        ++matched_magic;
        if (matched_magic == sizeof(kFrameMagic))
        {
          break;
        }
      }
      else
      {
        matched_magic = byte == kFrameMagic[0] ? 1 : 0;
      }
    }

    uint8_t header[10] = {0};
    if (!readExact(header, sizeof(header)))
    {
      return false;
    }

    const uint8_t version = header[0];
    const uint32_t payload_len = readUint32LE(header + 2);
    const uint32_t expected_checksum = readUint32LE(header + 6);

    if (version != kFrameVersion)
    {
      ROS_WARN("[rosmsg_tcp_bridge] Drop NearLink frame with unsupported version: %u", version);
      return false;
    }
    if (payload_len == 0 || payload_len > kMaxPayloadBytes)
    {
      ROS_WARN("[rosmsg_tcp_bridge] Drop NearLink frame with invalid payload length: %u", payload_len);
      return false;
    }

    payload->resize(payload_len);
    if (!readExact(payload->data(), payload->size()))
    {
      payload->clear();
      return false;
    }

    const uint32_t actual_checksum = checksumPayload(*payload);
    if (actual_checksum != expected_checksum)
    {
      ROS_WARN("[rosmsg_tcp_bridge] Drop NearLink frame with checksum mismatch");
      payload->clear();
      return false;
    }

    return true;
  }

private:
  bool readExact(uint8_t *data, size_t length)
  {
    size_t offset = 0;
    while (offset < length && ros::ok())
    {
      const int fd = fd_;
      if (fd < 0)
      {
        return false;
      }

      const ssize_t n = ::read(fd, data + offset, length - offset);
      if (n > 0)
      {
        offset += static_cast<size_t>(n);
        continue;
      }
      if (n == 0 || errno == EINTR || errno == EAGAIN)
      {
        continue;
      }

      ROS_ERROR("[rosmsg_tcp_bridge] NearLink UART read failed: %s", std::strerror(errno));
      return false;
    }
    return offset == length;
  }

  bool writeAll(const uint8_t *data, size_t length)
  {
    size_t offset = 0;
    while (offset < length && ros::ok())
    {
      const ssize_t n = ::write(fd_, data + offset, length - offset);
      if (n > 0)
      {
        offset += static_cast<size_t>(n);
        continue;
      }
      if (n == 0 || errno == EINTR || errno == EAGAIN)
      {
        continue;
      }

      ROS_ERROR("[rosmsg_tcp_bridge] NearLink UART write failed: %s", std::strerror(errno));
      return false;
    }
    return offset == length;
  }

  int fd_ = -1;
  size_t tx_chunk_bytes_ = 240;
  int chunk_gap_us_ = 2000;
  int peer_id_ = -1;
  std::mutex write_mutex_;
};

class BridgeNode
{
public:
  BridgeNode() = default;
  ~BridgeNode()
  {
    stop();
  }

  bool init(ros::NodeHandle &nh)
  {
    nh_ = nh;

    std::string nearlink_port;
    int nearlink_baudrate = 115200;
    int tx_chunk_bytes = 240;
    int chunk_gap_us = 2000;
    int nearlink_peer_id = -1;

    nh_.param("nearlink_port", nearlink_port, std::string("/dev/ttyUSB0"));
    nh_.param("nearlink_baudrate", nearlink_baudrate, 115200);
    nh_.param("nearlink_tx_chunk_bytes", tx_chunk_bytes, 240);
    nh_.param("nearlink_chunk_gap_us", chunk_gap_us, 2000);
    nh_.param("nearlink_peer_id", nearlink_peer_id, -1);
    nh_.param("drone_id", drone_id_, -1);
    nh_.param("odom_max_freq", odom_broadcast_freq_, 1000.0);

    if (drone_id_ < 0)
    {
      ROS_ERROR("[rosmsg_tcp_bridge] Wrong drone_id");
      return false;
    }
    if (nearlink_peer_id > 9)
    {
      ROS_ERROR("[rosmsg_tcp_bridge] nearlink_peer_id must be -1 or 0..9 for the nearlink SLE UART sample");
      return false;
    }
    if (tx_chunk_bytes < 0)
    {
      ROS_ERROR("[rosmsg_tcp_bridge] nearlink_tx_chunk_bytes must be >= 0");
      return false;
    }

    serial_.setTxOptions(static_cast<size_t>(tx_chunk_bytes), chunk_gap_us, nearlink_peer_id);
    if (!serial_.openPort(nearlink_port, nearlink_baudrate))
    {
      return false;
    }

    const std::string sub_traj_topic_name =
        std::string("/drone_") + std::to_string(drone_id_) + std::string("_planning/swarm_trajs");
    swarm_trajs_sub_ = nh_.subscribe(sub_traj_topic_name.c_str(), 10, &BridgeNode::multiTrajCb, this,
                                     ros::TransportHints().tcpNoDelay());

    if (drone_id_ >= 1)
    {
      const std::string pub_traj_topic_name =
          std::string("/drone_") + std::to_string(drone_id_ - 1) + std::string("_planning/swarm_trajs");
      swarm_trajs_pub_ = nh_.advertise<traj_utils::MultiBsplines>(pub_traj_topic_name.c_str(), 10);
    }

    other_odoms_sub_ = nh_.subscribe("my_odom", 10, &BridgeNode::odomCb, this,
                                     ros::TransportHints().tcpNoDelay());
    other_odoms_pub_ = nh_.advertise<nav_msgs::Odometry>("/others_odom", 10);

    one_traj_sub_ = nh_.subscribe("/broadcast_bspline", 100, &BridgeNode::oneTrajCb, this,
                                  ros::TransportHints().tcpNoDelay());
    one_traj_pub_ = nh_.advertise<traj_utils::Bspline>("/broadcast_bspline2", 100);

    ROS_INFO("[rosmsg_tcp_bridge] NearLink bridge initialized for drone_%d", drone_id_);
    return true;
  }

  void start()
  {
    running_ = true;
    recv_thread_ = std::thread(&BridgeNode::receiveLoop, this);
  }

  void stop()
  {
    running_ = false;
    serial_.closePort();
    if (recv_thread_.joinable())
    {
      recv_thread_.join();
    }
  }

private:
  template <typename MessageT>
  bool sendMessage(MessageType type, const MessageT &msg)
  {
    const std::vector<uint8_t> payload = serializePayload(type, msg);
    if (payload.size() > kMaxPayloadBytes)
    {
      ROS_ERROR("[rosmsg_tcp_bridge] Serialized message too large for NearLink frame: %zu", payload.size());
      return false;
    }
    return serial_.sendFrame(payload);
  }

  void multiTrajCb(const traj_utils::MultiBsplinesConstPtr &msg)
  {
    if (!sendMessage(MULTI_TRAJ, *msg))
    {
      ROS_ERROR_THROTTLE(1.0, "[rosmsg_tcp_bridge] Failed to send MultiBsplines over NearLink");
    }
  }

  void odomCb(const nav_msgs::OdometryConstPtr &msg)
  {
    const ros::Time now = ros::Time::now();
    if ((now - odom_last_send_time_).toSec() * odom_broadcast_freq_ < 1.0)
    {
      return;
    }
    odom_last_send_time_ = now;

    nav_msgs::Odometry tx_msg = *msg;
    tx_msg.child_frame_id = std::string("drone_") + std::to_string(drone_id_);

    if (!sendMessage(ODOM, tx_msg))
    {
      ROS_ERROR_THROTTLE(1.0, "[rosmsg_tcp_bridge] Failed to send Odometry over NearLink");
    }
  }

  void oneTrajCb(const traj_utils::BsplineConstPtr &msg)
  {
    if (!sendMessage(ONE_TRAJ, *msg))
    {
      ROS_ERROR_THROTTLE(1.0, "[rosmsg_tcp_bridge] Failed to send Bspline over NearLink");
    }
  }

  void receiveLoop()
  {
    std::vector<uint8_t> payload;
    while (ros::ok() && running_)
    {
      if (!serial_.readFrame(&payload))
      {
        ros::Duration(0.001).sleep();
        continue;
      }
      handlePayload(payload);
    }
  }

  void handlePayload(const std::vector<uint8_t> &payload)
  {
    if (payload.size() < kPayloadTypeBytes)
    {
      ROS_WARN("[rosmsg_tcp_bridge] Drop NearLink payload without message type");
      return;
    }

    const MessageType type = static_cast<MessageType>(readInt32LE(payload.data()));
    switch (type)
    {
    case MULTI_TRAJ:
    {
      traj_utils::MultiBsplines msg;
      if (deserializeBody(payload, &msg) && swarm_trajs_pub_)
      {
        swarm_trajs_pub_.publish(msg);
      }
      break;
    }
    case ODOM:
    {
      nav_msgs::Odometry msg;
      if (deserializeBody(payload, &msg))
      {
        other_odoms_pub_.publish(msg);
      }
      break;
    }
    case ONE_TRAJ:
    {
      traj_utils::Bspline msg;
      if (deserializeBody(payload, &msg))
      {
        one_traj_pub_.publish(msg);
      }
      break;
    }
    case STOP:
    {
      std_msgs::Empty msg;
      if (deserializeBody(payload, &msg) && emergency_stop_pub_)
      {
        emergency_stop_pub_.publish(msg);
      }
      break;
    }
    default:
      ROS_WARN_THROTTLE(1.0, "[rosmsg_tcp_bridge] Drop unknown NearLink message type: %d",
                        static_cast<int>(type));
      break;
    }
  }

  ros::NodeHandle nh_;
  ros::Subscriber swarm_trajs_sub_;
  ros::Subscriber other_odoms_sub_;
  ros::Subscriber emergency_stop_sub_;
  ros::Subscriber one_traj_sub_;
  ros::Publisher swarm_trajs_pub_;
  ros::Publisher other_odoms_pub_;
  ros::Publisher emergency_stop_pub_;
  ros::Publisher one_traj_pub_;

  NearlinkSerial serial_;
  std::thread recv_thread_;
  std::atomic<bool> running_{false};

  int drone_id_ = -1;
  double odom_broadcast_freq_ = 1000.0;
  ros::Time odom_last_send_time_;
};

} // namespace

int main(int argc, char **argv)
{
  ros::init(argc, argv, "rosmsg_tcp_bridge");
  ros::NodeHandle nh("~");

  BridgeNode bridge;
  if (!bridge.init(nh))
  {
    return EXIT_FAILURE;
  }

  bridge.start();
  ROS_INFO("[rosmsg_tcp_bridge] start running with NearLink serial transport");

  ros::spin();
  bridge.stop();
  return EXIT_SUCCESS;
}
