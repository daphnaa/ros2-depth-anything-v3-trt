#ifndef COMM_FUNC_H
#define COMM_FUNC_H

#include <string>
#include <vector>
#include <unistd.h>
#include <comm_inf_com.h>
#include <comm_inf_tcp.h>
#include <ros_interface.h>
#include <ArduinoJson.h>

using namespace std;
using namespace XROS_COMM;
//////////////////////////////////////////////////////////////////
struct DevState
{
  unsigned char dev_class;
  unsigned char dev_type;
  unsigned char dev_number;
  unsigned char dev_enable;
};
//////////////////////////////////////////////////////////////////
class CommFUNC
{
public:
    explicit CommFUNC(void);
    static CommFUNC& GetCommFUNC();
    bool BaseInit();
    bool ComInit();
    bool TcpInit();
    void DeviceListClear(void); 
    void DevMsgEnable(bool en);
    void TCPParamSet(std::string addr, uint16_t port);
    void OnlineCheck(unsigned char c,unsigned char t,unsigned char n,unsigned char en);
    void PubDevState();
    void ComSendData();
    void TcpSendData();
    unsigned long ComRecvData();
    unsigned long TcpRecvData();

private:
    static unsigned char DataCheckSum(unsigned char* data);

private:
    vector<DevState> m_device_list;
    bool m_dev_list_en;
    CommINFCOM m_comm_com;
    CommINFTCP m_comm_tcp;

};

#endif // COMM_FUNC_H
