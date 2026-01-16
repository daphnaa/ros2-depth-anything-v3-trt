#ifndef INTERNAL_ROS1_INTERFACE_H
#define INTERNAL_ROS1_INTERFACE_H

#include <string>
#include <vector>
#include <memory>
#include <ros/ros.h>
#include <stdarg.h>

//add message include below////////////////////////////////
#include <xpkg_msgs/XmsgCommData.h>
#include <std_msgs/ByteMultiArray.h>
#include <std_msgs/String.h>

using namespace std;
namespace XROS_COMM
{
//////////////////////////////////////////////////////////////////////////
enum class LogLevel
{
  kDebug = 0,
  kInfo,
  kWarn,
  kError,
  kFatal
};
struct XstdData
{
  unsigned char id_c;
  unsigned char id_t;
  unsigned char id_n;
  unsigned char id_f;
  unsigned char len;
  unsigned char data[8];
  double time;
};

////////////////////////////////////////////////////////////
class ROSInterface
{
  /*********************************************************
                    Base function zone
  *********************************************************/
  public:
    static ROSInterface& GetInterface();
    void BaseInit(int argc, char* argv[], std::string node_name,double period, void (*handle)());
    void BaseDeinit();
    inline void Work() {ros::spin();}
    inline void WorkOnce() {ros::spinOnce();}
    inline void Shutdown() {ros::shutdown();}
    inline bool Ok() {return ros::ok();}
    inline ros::Time GetTime(){return ros::Time::now();}
    void ROSLog(LogLevel, const char*, ...);
  
  private:
    ROSInterface() = default;
    virtual ~ROSInterface() = default;
    void VariableInit();
    void ParameterInit();
    void PublisherInit();
    void SubscriptionInit();
    void TimerInit(double period, void (*handle)());

  protected:
    inline void TimerCallback(const ros::TimerEvent&) { m_timer_handle(); }

  private:
    ros::NodeHandle* m_node_ptr;
    ros::NodeHandle* m_node_local_ptr;
    ros::Timer m_timer;
    void (*m_timer_handle)();
  
  /*********************************************************
                        Custom zone
  *********************************************************/
  public:  
    //add pub function below////////////////////////////////
    void PubComXstd(const XstdData& data);
    void PubTcpXstd(const XstdData& data);
    void PubComCommon(const vector<unsigned char>& data);
    void PubTcpCommon(const vector<unsigned char>& data);
    void PubDevList(const std::string& list);

    //add sub function below////////////////////////////////
    inline bool GetTcpXstdFlag() { return m_f_tcp_xstd; }
    inline bool GetTcpCommonFlag() { return m_f_tcp_common; }
    inline bool GetComXstdFlag() { return m_f_com_xstd; }
    inline bool GetComCommonFlag() { return m_f_com_common; }
    inline void ResetTcpXstdFlag() { m_f_tcp_xstd = false; }
    inline void ResetTcpCommonFlag() { m_f_tcp_common = false; } 
    inline void ResetComXstdFlag() { m_f_com_xstd = false; } 
    inline void ResetComCommonFlag() { m_f_com_common = false; }
    inline vector<XstdData> GetTcpXstdMsg() { return m_data_tcp_xstd; }
    inline vector<XstdData> GetComXstdMsg() { return m_data_com_xstd; }
    inline vector<unsigned char> GetTcpCommonMsg() { return m_data_tcp_common; }
    inline vector<unsigned char> GetComCommonMsg() { return m_data_com_common; }
    inline void ClearTcpXstdMsg() { m_data_tcp_xstd.clear(); }
    inline void ClearComXstdMsg() { m_data_com_xstd.clear(); }
    inline void ClearTcpCommonMsg() { m_data_tcp_common.clear(); }
    inline void ClearComCommonMsg() { m_data_com_common.clear(); }

    //add sub callback below////////////////////////////////////
    void TcpSendXstdCallback(const xpkg_msgs::XmsgCommDataPtr& data);
    void TcpSendCommonCallback(const std_msgs::ByteMultiArrayPtr& data);
    void ComSendXstdCallback(const xpkg_msgs::XmsgCommDataPtr& data);
    void ComSendCommonCallback(const std_msgs::ByteMultiArrayPtr& data);
  
  public:
    //add param Variable below///////////////////////////////
    bool m_com_en;
    bool m_tcp_en;
    bool m_com_common_en;
    bool m_com_xstd_en;
    bool m_tcp_common_en;
    bool m_tcp_xstd_en;
    bool m_dev_list_en;
    std::string m_tcp_addr;
    int m_tcp_port; 

  private: 
    //add sub flag below/////////////////////////////////////
    bool m_f_com_common;
    bool m_f_com_xstd;
    bool m_f_tcp_common;
    bool m_f_tcp_xstd;

    //add pub Variable below/////////////////////////////////
    ros::Publisher pub_com_common;
    ros::Publisher pub_tcp_common;
    ros::Publisher pub_com_xstd[14];
    ros::Publisher pub_tcp_xstd[14];
    ros::Publisher pub_device_list;

    //add sub Variable below/////////////////////////////////
    ros::Subscriber sub_com_xstd;
    ros::Subscriber sub_tcp_xstd;
    ros::Subscriber sub_com_common;    
    ros::Subscriber sub_tcp_common;

    //add nomal Variable below///////////////////////////////
    vector<XstdData> m_data_tcp_xstd;
    vector<XstdData> m_data_com_xstd;
    vector<unsigned char> m_data_tcp_common;
    vector<unsigned char> m_data_com_common;
   
};

}//namespace XROS_COMM
#endif // INTERNAL_ROS1_INTERFACE_H
