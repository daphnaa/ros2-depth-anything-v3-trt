#include <comm_func.h>
using namespace XROS_COMM;
#define DEV_NODE_NAME "xnode_comm"

CommFUNC::CommFUNC()
{
    m_dev_list_en = false;
}
/*------------------------------------------------------------------------------------------------------------------
 * name: GetInterface
 -----------------------------------------------------------------------------------------------------------------*/
CommFUNC& CommFUNC::GetCommFUNC()
{
    static CommFUNC comm_func;
    return comm_func;
}
/*------------------------------------------------------------------------------------------------------------------
 * name: CommInit
 -----------------------------------------------------------------------------------------------------------------*/
bool CommFUNC::BaseInit()
{
    ROSInterface& ros_interface = ROSInterface::GetInterface();
    //show device list or not////////////////////
    DevMsgEnable(ros_interface.m_dev_list_en);
    //set TCP parameter//////////////////////////
    TCPParamSet(ros_interface.m_tcp_addr,static_cast<uint16_t>(ros_interface.m_tcp_port));
    //cheak interface////////////////////////////
    if(!(ros_interface.m_com_en | ros_interface.m_tcp_en))
    {
        ros_interface.ROSLog(LogLevel::kError," %s: All communication interface is closed,please enable one",DEV_NODE_NAME);
        ros_interface.ROSLog(LogLevel::kError," %s: Node close",DEV_NODE_NAME);
        return false;
    }
    if(ros_interface.m_com_en && !m_comm_com.COMIsOpen())ComInit();
    if(ros_interface.m_tcp_en && !m_comm_tcp.TCPIsOpen())TcpInit();
    ros_interface.ROSLog(LogLevel::kInfo,"\033[1;32m %s: ### communication init finish ### \033[0m",DEV_NODE_NAME);
    return true;
}
/*------------------------------------------------------------------------------------------------------------------
 * name: DataCheckSum
 -----------------------------------------------------------------------------------------------------------------*/
unsigned char CommFUNC::DataCheckSum(unsigned char* data)
{
    unsigned char checksum = 0x00;
    for(int i = 0 ; i < data[1] - 1; i++)
    {
        checksum += data[i];
    }
    return checksum;
}
/*------------------------------------------------------------------------------------------------------------------
 * name: DataCheckSum
 -----------------------------------------------------------------------------------------------------------------*/
 void CommFUNC::TCPParamSet(std::string addr, uint16_t port)
{
    m_comm_tcp.TCPAddrSet(addr,port);
}
 /*------------------------------------------------------------------------------------------------------------------
  * name: DeviceListClear
  -----------------------------------------------------------------------------------------------------------------*/
 void CommFUNC::DeviceListClear(void)
 {
    ROSInterface& ros_interface = ROSInterface::GetInterface();
    std::string send_data;
    m_device_list.clear();
    DynamicJsonDocument doc(1024);
    doc["dev_count"] = m_device_list.size();
    serializeJson(doc, send_data);
    ros_interface.PubDevList(send_data);
 }
 /*------------------------------------------------------------------------------------------------------------------
  * name: ComMsgEnable
  -----------------------------------------------------------------------------------------------------------------*/
 void CommFUNC::DevMsgEnable(bool en)
 {
     if(en)m_dev_list_en = true;
     else m_dev_list_en = false;
 }
/*------------------------------------------------------------------------------------------------------------------
 * name: ComInit
 -----------------------------------------------------------------------------------------------------------------*/
bool CommFUNC::ComInit()
{
    if(m_comm_com.COMIsOpen())return true;
    if(!m_comm_com.COMInitDefault())sleep(3);
    m_device_list.clear();
    return false;
}
/*------------------------------------------------------------------------------------------------------------------
 * name: TcpInit
 -----------------------------------------------------------------------------------------------------------------*/
bool CommFUNC::TcpInit()
{
    if(m_comm_tcp.TCPIsOpen())return true;
    if(!m_comm_tcp.TCPOpen()) sleep(3);
    m_device_list.clear();
    return false;
}
/*------------------------------------------------------------------------------------------------------------------
 * name: ComSendData
 -----------------------------------------------------------------------------------------------------------------*/
void CommFUNC::ComSendData()
{
    vector<unsigned char> com_data_send;
    unsigned char buff[16];
    ROSInterface& ros_interface = ROSInterface::GetInterface();
    //xstd data////////////////////////////////////////////////////////
    if(ros_interface.GetComXstdFlag() && ros_interface.m_com_xstd_en)
    {
        vector<XstdData> data_xstd = ros_interface.GetComXstdMsg();
        for(unsigned long i=0;i<data_xstd.size();i++)
        {
            buff[0] = 0x55;
            buff[1] = data_xstd[i].len+8;
            buff[2] = data_xstd[i].id_c;
            buff[3] = data_xstd[i].id_t;
            buff[4] = data_xstd[i].id_n;
            buff[5] = data_xstd[i].id_f;
            memcpy(&buff[6],&data_xstd[i].data[0],data_xstd[i].len);
            buff[data_xstd[i].len+6] = 0x01;
            buff[data_xstd[i].len+7] = this->DataCheckSum(buff);
            for(unsigned long j=0;j<buff[1];j++)com_data_send.push_back(buff[j]);
            if(com_data_send.size()>1024)
            {
              m_comm_com.COMDataSend(com_data_send);
              com_data_send.clear();
            }
        }
        ros_interface.ResetComXstdFlag();
        ros_interface.ClearComXstdMsg();
    }
    //common data/////////////////////////////////////////////////////
    if(ros_interface.GetComCommonFlag() && ros_interface.m_com_common_en)
    {
        vector<unsigned char> data_common = ros_interface.GetComCommonMsg();
        com_data_send.insert(com_data_send.end(),data_common.begin(),data_common.end());
        ros_interface.ResetComCommonFlag();
        ros_interface.ClearComCommonMsg();
    }   
    if(com_data_send.size())m_comm_com.COMDataSend(com_data_send);
}
/*------------------------------------------------------------------------------------------------------------------
 * name: TcpSendData
 -----------------------------------------------------------------------------------------------------------------*/
void CommFUNC::TcpSendData()
{
    vector<unsigned char> tcp_data_send;
    unsigned char buff[16];
    ROSInterface& ros_interface = ROSInterface::GetInterface();
    //xstd data////////////////////////////////////////////////////////
    if(ros_interface.GetTcpXstdFlag() && ros_interface.m_tcp_xstd_en)
    {
        vector<XstdData> data_xstd = ros_interface.GetTcpXstdMsg();
        for(unsigned long i=0;i<data_xstd.size();i++)
        {
            buff[0] = 0x55;
            buff[1] = data_xstd[i].len+8;
            buff[2] = data_xstd[i].id_c;
            buff[3] = data_xstd[i].id_t;
            buff[4] = data_xstd[i].id_n;
            buff[5] = data_xstd[i].id_f;
            memcpy(&buff[6],&data_xstd[i].data[0],data_xstd[i].len);
            buff[data_xstd[i].len+6] = 0x01;
            buff[data_xstd[i].len+7] = this->DataCheckSum(buff);
            for(unsigned long j=0;j<buff[1];j++)tcp_data_send.push_back(buff[j]);
            if(tcp_data_send.size()>1024)
            {
              m_comm_tcp.TCPDataSend(tcp_data_send);
              tcp_data_send.clear();
            }
        }
        ros_interface.ResetTcpXstdFlag();
        ros_interface.ClearTcpXstdMsg();
    }
    //common data/////////////////////////////////////////////////////
    if(ros_interface.GetTcpCommonFlag() && ros_interface.m_tcp_common_en)
    {
        vector<unsigned char> data_common = ros_interface.GetTcpCommonMsg();
        tcp_data_send.insert(tcp_data_send.end(),data_common.begin(),data_common.end());
        ros_interface.ResetTcpCommonFlag();
        ros_interface.ClearTcpCommonMsg();
    }
    if(tcp_data_send.size())m_comm_tcp.TCPDataSend(tcp_data_send);
}
/*------------------------------------------------------------------------------------------------------------------
 * name: ComRecvData
 -----------------------------------------------------------------------------------------------------------------*/
unsigned long CommFUNC::ComRecvData()
{
    vector<unsigned char> data;
    ROSInterface& ros_interface = ROSInterface::GetInterface();
    unsigned long count;
    static int no_data_count=0;
    data = m_comm_com.COMDataRecv();
    count =data.size();
    if(count == 0)
    {
        no_data_count++;
        if(no_data_count == 1000)ros_interface.ROSLog(LogLevel::kInfo,"\033[5;33m %s: COM received no data,check cable connect?\033[0m",DEV_NODE_NAME);
        return 0;
    }
    no_data_count = 0;
    if(ros_interface.m_com_common_en)ros_interface.PubComCommon(data);
    if(ros_interface.m_com_xstd_en)
    {

      static unsigned int data_err_count = 0;
      static vector<unsigned char> com_data_recv;
      unsigned long i;
      XstdData com_buff;
      com_data_recv.insert(com_data_recv.end(),data.begin(),data.end());
      if(com_data_recv.size() > 1000)com_data_recv.clear();
      for(i=0;i<com_data_recv.size();i++)
      {
          if(com_data_recv.size()-i <8)break;
          if(com_data_recv.at(i) != 0x55 || com_data_recv.at(i+1) <8 || com_data_recv.at(i+1) >16)continue;  //head cheak
          if(com_data_recv.size()-i < com_data_recv.at(i+1))break;//broken data
          if(DataCheckSum(com_data_recv.data()+i) != com_data_recv.at(i + com_data_recv.at(i+1)-1))   //sum cheak
          {
              i += com_data_recv.at(i+1)-1;
              data_err_count++;
              ros_interface.ROSLog(LogLevel::kError," %s: COM receive data error No.%d",DEV_NODE_NAME,data_err_count);
              if(data_err_count>10000)data_err_count = 0;
              continue;
          }

          com_buff.len = com_data_recv.at(i+1)-8;
          com_buff.id_c = com_data_recv.at(i+2);
          com_buff.id_t = com_data_recv.at(i+3);
          com_buff.id_n = com_data_recv.at(i+4);
          com_buff.id_f = com_data_recv.at(i+5);
          memcpy(com_buff.data,com_data_recv.data()+i+6,com_buff.len);

          i += com_data_recv.at(i+1)-1;
          if(com_buff.id_f == 0xB0)OnlineCheck(com_buff.id_c,com_buff.id_t,com_buff.id_n,com_buff.data[0]);
          ros_interface.PubComXstd(com_buff);
      }
      if(com_data_recv.size()>0)com_data_recv.erase(com_data_recv.begin(),com_data_recv.begin()+static_cast<long>(i));
    }

    return count;
}
/*------------------------------------------------------------------------------------------------------------------
 * name: TcpRecvData
 -----------------------------------------------------------------------------------------------------------------*/
unsigned long CommFUNC::TcpRecvData()
{
    vector<unsigned char> data;
    ROSInterface& ros_interface = ROSInterface::GetInterface();
    unsigned long count;
    data = m_comm_tcp.TCPDataRecv();
    count =data.size();
    if(count == 0) return 0;
    if(ros_interface.m_tcp_common_en)ros_interface.PubTcpCommon(data);
    if(ros_interface.m_tcp_xstd_en)
    {
      static unsigned int data_err_count = 0;
      static vector<unsigned char> tcp_data_recv;
      unsigned long i;
      XstdData tcp_buff;
      tcp_data_recv.insert(tcp_data_recv.end(),data.begin(),data.end());
      if(tcp_data_recv.size() > 1000)tcp_data_recv.clear();
      for(i=0;i<tcp_data_recv.size();i++)
      {
          if(tcp_data_recv.size()-i <8)break;
          if(tcp_data_recv.at(i) != 0x55 || tcp_data_recv.at(i+1) <8 || tcp_data_recv.at(i+1) >16)continue;  //head cheak
          if(tcp_data_recv.size()-i < tcp_data_recv.at(i+1))break;//broken data
          if(DataCheckSum(tcp_data_recv.data()+i) != tcp_data_recv.at(i + tcp_data_recv.at(i+1)-1))   //sum cheak
          {
              i += tcp_data_recv.at(i+1)-1;
              data_err_count++;
              ros_interface.ROSLog(LogLevel::kError," %s: TCP receive data error No.%d",DEV_NODE_NAME,data_err_count);
              if(data_err_count>10000)data_err_count = 0;
              continue;
          }

          tcp_buff.len = tcp_data_recv.at(i+1)-8;
          tcp_buff.id_c = tcp_data_recv.at(i+2);
          tcp_buff.id_t = tcp_data_recv.at(i+3);
          tcp_buff.id_n = tcp_data_recv.at(i+4);
          tcp_buff.id_f = tcp_data_recv.at(i+5);
          memcpy(tcp_buff.data,tcp_data_recv.data()+i+6,tcp_buff.len);

          i += tcp_data_recv.at(i+1)-1;
          if(tcp_buff.id_f == 0xB0)OnlineCheck(tcp_buff.id_c,tcp_buff.id_t,tcp_buff.id_n,tcp_buff.data[0]);
          ros_interface.PubTcpXstd(tcp_buff);
      }
      if(tcp_data_recv.size()>0)tcp_data_recv.erase(tcp_data_recv.begin(),tcp_data_recv.begin()+static_cast<long>(i));

    }
    return count;
}
/*------------------------------------------------------------------------------------------------------------------
 * name: OnlineCheak
 * detail:
 -----------------------------------------------------------------------------------------------------------------*/
void CommFUNC::OnlineCheck(unsigned char c,unsigned char t,unsigned char n,unsigned char en)
{
    unsigned long i;    
    //exist device check//////////////////////////////////
    for(i=0;i<m_device_list.size();i++)
    {
        if(m_device_list[i].dev_class == c && m_device_list[i].dev_type == t && m_device_list[i].dev_number == n)
        {
            if(m_device_list[i].dev_enable == en)return;
            m_device_list[i].dev_enable = en;
            break;
        }
    }
    //new device///////////////////////////////////////////
    if(i == m_device_list.size())
    {
        m_device_list.resize(m_device_list.size()+1);
        m_device_list[m_device_list.size()-1].dev_class =c;
        m_device_list[m_device_list.size()-1].dev_number =n;
        m_device_list[m_device_list.size()-1].dev_type =t;
        m_device_list[m_device_list.size()-1].dev_enable =en;
    }
    //send new device msg//////////////////////////////////
    PubDevState();
}
/*------------------------------------------------------------------------------------------------------------------
 * name: PubDevState
 * detail:Send device state list msg
 -----------------------------------------------------------------------------------------------------------------*/
void CommFUNC::PubDevState()
{
    ROSInterface& ros_interface = ROSInterface::GetInterface();
    std::string send_data;
    DynamicJsonDocument doc(1024);
    doc["dev_count"] = m_device_list.size();
    for(unsigned long i=0;i<m_device_list.size();i++)
    {
       doc["dev_class"][i] = m_device_list[i].dev_class;
       doc["dev_type"][i] = m_device_list[i].dev_type;
       doc["dev_number"][i] = m_device_list[i].dev_number;
       doc["dev_enable"][i] = m_device_list[i].dev_enable;
    }
    serializeJson(doc, send_data);
    ros_interface.PubDevList(send_data);
    //show device list or not//////////////////////////////
    if(m_dev_list_en)
    {
        ros_interface.ROSLog(LogLevel::kInfo,"\033[1;44;37m %s: ====== New device online ====== \033[0m",DEV_NODE_NAME);
        for(unsigned long i=0;i<m_device_list.size();i++)
        {
            ros_interface.ROSLog(LogLevel::kInfo,"\033[1;34m No.%d --- class:%X type:%X number:%d enable:%d \033[0m",
            i+1, m_device_list[i].dev_class, m_device_list[i].dev_type,
            m_device_list[i].dev_number, m_device_list[i].dev_enable);
        }
    }
}
