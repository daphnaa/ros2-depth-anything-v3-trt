#include <ros_interface.h>

namespace XROS_COMM
{
/*------------------------------------------------------------------------------------------------------------------
 * name: BaseInit
 -----------------------------------------------------------------------------------------------------------------*/
void ROSInterface::BaseInit(int argc, char* argv[], std::string node_name,double period, void (*handle)())
{
  rclcpp::init(argc, argv);
  m_node_ptr = std::make_shared<rclcpp::Node>(node_name);

  VariableInit();
  ParameterInit();
  PublisherInit();
  SubscriptionInit();
  TimerInit(period, handle);

  ROSLog(LogLevel::kInfo,"\033[1;32m %s: ### ROS interface init finish ### \033[0m",node_name.data());
}
/*------------------------------------------------------------------------------------------------------------------
 * name: BaseDeinit
 -----------------------------------------------------------------------------------------------------------------*/
void ROSInterface::BaseDeinit()
{
  // timer
  m_timer.reset();

  // pub
  if(m_com_common_en && m_com_en)pub_com_common.reset();
  if(m_tcp_common_en && m_tcp_en)pub_tcp_common.reset();
  for (size_t i = 0; i < 12; i++)
  {
      if (m_com_xstd_en && m_com_en)pub_com_xstd[i].reset();
      if(m_tcp_xstd_en && m_tcp_en)pub_tcp_xstd[i].reset();
  }
  pub_device_list.reset();

  // sub
  if(m_com_xstd_en && m_com_en)sub_com_xstd.reset();
  if(m_com_common_en && m_com_en)sub_com_common.reset();
  if(m_tcp_xstd_en && m_tcp_en)sub_tcp_xstd.reset();
  if(m_tcp_common_en && m_tcp_en)sub_tcp_common.reset();

  // node
  m_node_ptr.reset();
  
  // shutdown
  Shutdown();
}
/*------------------------------------------------------------------------------------------------------------------
 * name: GetInterface
 -----------------------------------------------------------------------------------------------------------------*/
ROSInterface& ROSInterface::GetInterface()
{
  static ROSInterface ros_interface;
  return ros_interface;
}
/*------------------------------------------------------------------------------------------------------------------
 * name: ROSLog
 -----------------------------------------------------------------------------------------------------------------*/
void ROSInterface::ROSLog(LogLevel level, const char* format, ...)
{
  char* buffer;

  va_list args;
  va_start(args, format);
  int32_t len = vasprintf(&buffer, format, args);
  va_end(args);

  if (len < 0)
  {
    RCLCPP_FATAL(m_node_ptr->get_logger(), "### Wrong Log Message ###");
    return;
  }

  switch (level)
  {
    case LogLevel::kDebug:
      RCLCPP_DEBUG(m_node_ptr->get_logger(), "%s", buffer);
      break;
    case LogLevel::kInfo:
      RCLCPP_INFO(m_node_ptr->get_logger(), "%s", buffer);
      break;
    case LogLevel::kWarn:
      RCLCPP_WARN(m_node_ptr->get_logger(), "%s", buffer);
      break;
    case LogLevel::kError:
      RCLCPP_ERROR(m_node_ptr->get_logger(), "%s", buffer);
      break;
    case LogLevel::kFatal:
      RCLCPP_FATAL(m_node_ptr->get_logger(), "%s", buffer);
      break;
    default:
      RCLCPP_FATAL(m_node_ptr->get_logger(), "### Wrong Log Level ###");
      RCLCPP_FATAL(m_node_ptr->get_logger(), "%s", buffer);
      break;
  }
  free(buffer);
}
/*------------------------------------------------------------------------------------------------------------------
 * name: TimerInit
 -----------------------------------------------------------------------------------------------------------------*/
void ROSInterface::TimerInit(double period, void (*handle)())
{
  m_timer_handle = handle;
  m_timer = m_node_ptr->create_wall_timer(std::chrono::milliseconds(static_cast<int64_t>(period)), std::bind(&ROSInterface::TimerCallback, this));
}
/*------------------------------------------------------------------------------------------------------------------
 * name: ParameterInit
 -----------------------------------------------------------------------------------------------------------------*/
void ROSInterface::ParameterInit()
{
  m_node_ptr->declare_parameter<bool>("dev_list", false);
  m_node_ptr->declare_parameter<bool>("com_enable", false);
  m_node_ptr->declare_parameter<bool>("com_channel_common", false);
  m_node_ptr->declare_parameter<bool>("com_channel_xstd", false);
  m_node_ptr->declare_parameter<bool>("tcp_enable", false);
  m_node_ptr->declare_parameter<bool>("tcp_channel_common", false);
  m_node_ptr->declare_parameter<bool>("tcp_channel_xstd", false);
  m_node_ptr->declare_parameter<std::string>("tcp_addr", "115.29.240.46");
  m_node_ptr->declare_parameter<uint16_t>("tcp_port", 9000);

  m_node_ptr->get_parameter("dev_list", m_dev_list_en);
  m_node_ptr->get_parameter("com_enable", m_com_en);
  m_node_ptr->get_parameter("com_channel_common", m_com_common_en);
  m_node_ptr->get_parameter("com_channel_xstd", m_com_xstd_en);
  m_node_ptr->get_parameter("tcp_enable", m_tcp_en);
  m_node_ptr->get_parameter("tcp_channel_common", m_tcp_common_en);
  m_node_ptr->get_parameter("tcp_channel_xstd", m_tcp_xstd_en);
  m_node_ptr->get_parameter("tcp_port", m_tcp_port);
  m_node_ptr->get_parameter("tcp_addr", m_tcp_addr);
}
/*------------------------------------------------------------------------------------------------------------------
 * name: VariableInit
 -----------------------------------------------------------------------------------------------------------------*/
void ROSInterface::VariableInit()
{
  m_com_en = false;
  m_tcp_en = false;
  m_com_common_en = false;
  m_com_xstd_en = false;
  m_tcp_common_en = false;
  m_tcp_xstd_en = false;
  m_dev_list_en = false;
  m_f_com_common = false;
  m_f_com_xstd = false;
  m_f_tcp_common = false;
  m_f_tcp_xstd = false;
  m_tcp_port = 0;
}
/*------------------------------------------------------------------------------------------------------------------
 * name: PublisherInit
 -----------------------------------------------------------------------------------------------------------------*/
void ROSInterface::PublisherInit()
{
  if(m_com_xstd_en && m_com_en)
  {
      pub_com_xstd[0] = m_node_ptr->create_publisher<xpkg_msgs::msg::XmsgCommData>("/xtopic_comm/com_recv_xstd_custom", 50);
      pub_com_xstd[1] = m_node_ptr->create_publisher<xpkg_msgs::msg::XmsgCommData>("/xtopic_comm/com_recv_xstd_vehicle", 50);
      pub_com_xstd[2] = m_node_ptr->create_publisher<xpkg_msgs::msg::XmsgCommData>("/xtopic_comm/com_recv_xstd_arm", 50);
      pub_com_xstd[3] = m_node_ptr->create_publisher<xpkg_msgs::msg::XmsgCommData>("/xtopic_comm/com_recv_xstd_conveyer", 50);
      pub_com_xstd[4] = m_node_ptr->create_publisher<xpkg_msgs::msg::XmsgCommData>("/xtopic_comm/com_recv_xstd_light", 50);
      pub_com_xstd[5] = m_node_ptr->create_publisher<xpkg_msgs::msg::XmsgCommData>("/xtopic_comm/com_recv_xstd_driver", 50);
      pub_com_xstd[6] = m_node_ptr->create_publisher<xpkg_msgs::msg::XmsgCommData>("/xtopic_comm/com_recv_xstd_power", 50);
      pub_com_xstd[7] = m_node_ptr->create_publisher<xpkg_msgs::msg::XmsgCommData>("/xtopic_comm/com_recv_xstd_comm", 50);
      pub_com_xstd[8] = m_node_ptr->create_publisher<xpkg_msgs::msg::XmsgCommData>("/xtopic_comm/tcp_recv_xstd_sensor_distance", 50);
      pub_com_xstd[9] = m_node_ptr->create_publisher<xpkg_msgs::msg::XmsgCommData>("/xtopic_comm/com_recv_xstd_sensor_line", 50);
      pub_com_xstd[10] = m_node_ptr->create_publisher<xpkg_msgs::msg::XmsgCommData>("/xtopic_comm/com_recv_xstd_switch", 50);
      pub_com_xstd[11] = m_node_ptr->create_publisher<xpkg_msgs::msg::XmsgCommData>("/xtopic_comm/com_recv_xstd_sensor_imu", 50);
      pub_com_xstd[12] = m_node_ptr->create_publisher<xpkg_msgs::msg::XmsgCommData>("/xtopic_comm/com_recv_xstd_sensor_gps", 50);
      pub_com_xstd[13] = m_node_ptr->create_publisher<xpkg_msgs::msg::XmsgCommData>("/xtopic_comm/com_recv_xstd_sandbox", 50);
  }
  if(m_com_common_en && m_com_en)
  {
      pub_com_common = m_node_ptr->create_publisher<std_msgs::msg::ByteMultiArray>("/xtopic_comm/com_recv_common", 50);
  }
  if(m_tcp_xstd_en && m_tcp_en)
  {
      pub_tcp_xstd[0] = m_node_ptr->create_publisher<xpkg_msgs::msg::XmsgCommData>("/xtopic_comm/tcp_recv_xstd_custom", 50);
      pub_tcp_xstd[1] = m_node_ptr->create_publisher<xpkg_msgs::msg::XmsgCommData>("/xtopic_comm/tcp_recv_xstd_vehicle", 50);
      pub_tcp_xstd[2] = m_node_ptr->create_publisher<xpkg_msgs::msg::XmsgCommData>("/xtopic_comm/tcp_recv_xstd_arm", 50);
      pub_tcp_xstd[3] = m_node_ptr->create_publisher<xpkg_msgs::msg::XmsgCommData>("/xtopic_comm/tcp_recv_xstd_conveyer", 50);
      pub_tcp_xstd[4] = m_node_ptr->create_publisher<xpkg_msgs::msg::XmsgCommData>("/xtopic_comm/tcp_recv_xstd_light", 50);
      pub_tcp_xstd[5] = m_node_ptr->create_publisher<xpkg_msgs::msg::XmsgCommData>("/xtopic_comm/tcp_recv_xstd_driver", 50);
      pub_tcp_xstd[6] = m_node_ptr->create_publisher<xpkg_msgs::msg::XmsgCommData>("/xtopic_comm/tcp_recv_xstd_power", 50);
      pub_tcp_xstd[7] = m_node_ptr->create_publisher<xpkg_msgs::msg::XmsgCommData>("/xtopic_comm/tcp_recv_xstd_comm", 50);
      pub_tcp_xstd[8] = m_node_ptr->create_publisher<xpkg_msgs::msg::XmsgCommData>("/xtopic_comm/tcp_recv_xstd_sensor_distance", 50);
      pub_tcp_xstd[9] = m_node_ptr->create_publisher<xpkg_msgs::msg::XmsgCommData>("/xtopic_comm/tcp_recv_xstd_sensor_line", 50);
      pub_tcp_xstd[10] = m_node_ptr->create_publisher<xpkg_msgs::msg::XmsgCommData>("/xtopic_comm/tcp_recv_xstd_switch", 50);
      pub_tcp_xstd[11] =  m_node_ptr->create_publisher<xpkg_msgs::msg::XmsgCommData>("/xtopic_comm/tcp_recv_xstd_sensor_imu", 50);
      pub_tcp_xstd[12] = m_node_ptr->create_publisher<xpkg_msgs::msg::XmsgCommData>("/xtopic_comm/tcp_recv_xstd_sensor_gps", 50);
      pub_tcp_xstd[13] = m_node_ptr->create_publisher<xpkg_msgs::msg::XmsgCommData>("/xtopic_comm/tcp_recv_xstd_sandbox", 50);
  }
  if(m_tcp_common_en && m_tcp_en)
  {
      pub_tcp_common = m_node_ptr->create_publisher<std_msgs::msg::ByteMultiArray>("/xtopic_comm/tcp_recv_common", 50);
  }
  pub_device_list = m_node_ptr->create_publisher<std_msgs::msg::String>("/xtopic_comm/device_list_json", 50);
}
/*------------------------------------------------------------------------------------------------------------------
 * name: SubscriptionInit
 -----------------------------------------------------------------------------------------------------------------*/
void ROSInterface::SubscriptionInit()
{
    if(m_com_xstd_en && m_com_en)
    {
        sub_com_xstd = m_node_ptr->create_subscription<xpkg_msgs::msg::XmsgCommData>("/xtopic_comm/com_send_xstd", 1000, std::bind(&ROSInterface::ComSendXstdCallback, this, std::placeholders::_1));
    }
    if(m_com_common_en && m_com_en)
    {
        sub_com_common = m_node_ptr->create_subscription<std_msgs::msg::ByteMultiArray>("/xtopic_comm/com_send_common", 1000, std::bind(&ROSInterface::ComSendCommonCallback, this, std::placeholders::_1));
    }
    if(m_tcp_xstd_en && m_tcp_en)
    {
        sub_tcp_xstd = m_node_ptr->create_subscription<xpkg_msgs::msg::XmsgCommData>("/xtopic_comm/tcp_send_xstd", 1000, std::bind(&ROSInterface::TcpSendXstdCallback, this, std::placeholders::_1));
    }
    if(m_tcp_common_en && m_tcp_en)
    {
        sub_tcp_common = m_node_ptr->create_subscription<std_msgs::msg::ByteMultiArray>("/xtopic_comm/tcp_send_common", 1000, std::bind(&ROSInterface::TcpSendCommonCallback, this, std::placeholders::_1));
    }
}
/*------------------------------------------------------------------------------------------------------------------
 * name: PubTcpCommon
 -----------------------------------------------------------------------------------------------------------------*/
void ROSInterface::PubTcpCommon(const vector<unsigned char>& data)
{
    std_msgs::msg::ByteMultiArray data_tcp;
    data_tcp.data.insert(data_tcp.data.end(), data.begin(), data.end());
    pub_tcp_common->publish(data_tcp);
}
/*------------------------------------------------------------------------------------------------------------------
 * name: PubTcpXstd
 -----------------------------------------------------------------------------------------------------------------*/
void ROSInterface::PubTcpXstd(const XstdData& data)
{
    xpkg_msgs::msg::XmsgCommData data_tcp;
    static bool once = false;
    data_tcp.len = data.len;
    data_tcp.id_c = data.id_c;
    data_tcp.id_t = data.id_t;
    data_tcp.id_n = data.id_n;
    data_tcp.id_f = data.id_f;
    memcpy(&data_tcp.data[0], &data.data[0], data.len);
    data_tcp.stamp = m_node_ptr->get_clock()->now();
    if(data.id_c >= sizeof(pub_tcp_xstd)/sizeof(pub_com_xstd[0]))
    {
       pub_tcp_xstd[0]->publish(data_tcp);
       if(!once)
       {
           ROSLog(LogLevel::kWarn," xnode_comm: Not a registered device,will use custom topic.");
           once = true;
       }
    }
    else pub_tcp_xstd[data.id_c]->publish(data_tcp);
}
/*------------------------------------------------------------------------------------------------------------------
 * name: PubComCommon
 -----------------------------------------------------------------------------------------------------------------*/
void ROSInterface::PubComCommon(const vector<unsigned char>& data)
{
    std_msgs::msg::ByteMultiArray data_common;
    data_common.data.insert(data_common.data.end(), data.begin(), data.end());
    pub_com_common->publish(data_common);
}
/*------------------------------------------------------------------------------------------------------------------
 * name: PubComXstd
 -----------------------------------------------------------------------------------------------------------------*/
void ROSInterface::PubComXstd(const XstdData& data)
{
    xpkg_msgs::msg::XmsgCommData data_com;
    static bool once = false;
    data_com.len = data.len;
    data_com.id_c = data.id_c;
    data_com.id_t = data.id_t;
    data_com.id_n = data.id_n;
    data_com.id_f = data.id_f;
    memcpy(&data_com.data[0], &data.data[0], data.len);
    data_com.stamp = m_node_ptr->get_clock()->now();
    if(data.id_c >= sizeof(pub_com_xstd)/sizeof(pub_com_xstd[0]))
    {
       pub_com_xstd[0]->publish(data_com);
       if(!once)
       {
           ROSLog(LogLevel::kWarn," xnode_comm: Not a registered device,will use custom topic.");
           once = true;
       }
    }
    else pub_com_xstd[data.id_c]->publish(data_com);
}
/*------------------------------------------------------------------------------------------------------------------
 * name: PubDevList
 -----------------------------------------------------------------------------------------------------------------*/
void ROSInterface::PubDevList(const std::string& list)
{
    std_msgs::msg::String data;
    data.data.append(list);
    pub_device_list->publish(data);
}
/*------------------------------------------------------------------------------------------------------------------
 * name: sub callback
 -----------------------------------------------------------------------------------------------------------------*/
void ROSInterface::TcpSendXstdCallback(const xpkg_msgs::msg::XmsgCommData::SharedPtr data)
{
    m_f_tcp_xstd = true;
    XstdData data_tcp;
    data_tcp.len = data->len;
    data_tcp.id_c = data->id_c;
    data_tcp.id_t = data->id_t;
    data_tcp.id_n = data->id_n;
    data_tcp.id_f = data->id_f;
    memcpy(&data_tcp.data[0], &data->data[0], data->len);
    m_data_tcp_xstd.push_back(data_tcp);
    if (m_data_tcp_xstd.size() > 500) m_data_tcp_xstd.clear();
}
///////////////////////////////////////////////////////////////////////////////////////////////
void ROSInterface::TcpSendCommonCallback(const std_msgs::msg::ByteMultiArray::SharedPtr data)
{
    m_f_tcp_common = true;
    m_data_tcp_common.insert(m_data_tcp_common.end(), data->data.begin(), data->data.end());
    if (m_data_tcp_common.size() > 1024) m_data_tcp_common.clear();
}
///////////////////////////////////////////////////////////////////////////////////////////////
void ROSInterface::ComSendXstdCallback(const xpkg_msgs::msg::XmsgCommData::SharedPtr data)
{
    m_f_com_xstd = true;
    XstdData data_com;
    data_com.len = data->len;
    data_com.id_c = data->id_c;
    data_com.id_t = data->id_t;
    data_com.id_n = data->id_n;
    data_com.id_f = data->id_f;
    memcpy(&data_com.data[0], &data->data[0], data->len);
    m_data_com_xstd.push_back(data_com);
    if (m_data_com_xstd.size() > 500) m_data_com_xstd.clear();
}
///////////////////////////////////////////////////////////////////////////////////////////////
void ROSInterface::ComSendCommonCallback(const std_msgs::msg::ByteMultiArray::SharedPtr data)
{
    m_f_com_common = true;
    m_data_com_common.insert(m_data_com_common.end(), data->data.begin(), data->data.end());
    if (m_data_com_common.size() > 1024) m_data_com_common.clear();
}

}//namespace XROS_COMM
