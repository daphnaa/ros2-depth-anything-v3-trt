//////////////////////////////////////////////////
//xnode for communication V3.0
//xpkg_comm
//////////////////////////////////////////////////
#include <comm_func.h>
#include <ros_interface.h>

using namespace XROS_COMM;

void TimeCallback() {
  CommFUNC& comm_func = CommFUNC::GetCommFUNC();
  ROSInterface& ros_interface = ROSInterface::GetInterface();
  if(ros_interface.m_tcp_en)
  {
      if (comm_func.TcpInit())
      {
        comm_func.TcpSendData();
        comm_func.TcpRecvData();
      }
  }
  if(ros_interface.m_com_en)
  {
      if (comm_func.ComInit())
      {
        comm_func.ComSendData();
        comm_func.ComRecvData();
      }
  }
}
/*------------------------------------------------------------------------------------------------------------------
 * name: main
 -----------------------------------------------------------------------------------------------------------------*/
int main(int argc, char **argv)
{
    ROSInterface& ros_interface = ROSInterface::GetInterface();
    ros_interface.BaseInit(argc, argv, "xnode_comm", 10.0, TimeCallback);

    CommFUNC& comm_func = CommFUNC::GetCommFUNC();
    if(comm_func.BaseInit() == false)
    {
        ros_interface.BaseDeinit();
        return 0;
    }

    ros_interface.Work();
    ros_interface.BaseDeinit();
    return 0;
}
