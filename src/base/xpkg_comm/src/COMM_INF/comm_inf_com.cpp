#include <comm_inf_com.h>
#include <iostream>
#include <stdio.h>
using namespace std;

CommINFCOM::CommINFCOM(void)
{
    m_COM_fd = -1;
    f_COM_disconnect = false;
}
CommINFCOM::~CommINFCOM()
{
  COMClose();
}
/*------------------------------------------------------------------------------------------------------------------
 * name: COMIsOpen
 * detail: Return COM state
 -----------------------------------------------------------------------------------------------------------------*/
bool CommINFCOM::COMIsOpen()
{
    return m_COM_fd < 0 ? false : true;
}
/*------------------------------------------------------------------------------------------------------------------
 * name: COMOpen
 * detail: Open COM with the sequence
 -----------------------------------------------------------------------------------------------------------------*/
bool CommINFCOM::COMOpen()
{
    static bool note = false;
    static unsigned int connect_count = 0;
    int count = sizeof (arr_device)/sizeof (arr_device[0]);
    if (COMIsOpen()==true)return false;
    for(int i=0;i<count;i++)
    {      
        m_COM_fd = open(arr_device[i].data(),O_RDWR | O_NOCTTY | O_NONBLOCK);
        if(m_COM_fd!=-1)
        {            
            std::cout << "\033[1;32m COM open success \033[0m" << std::endl;
            std::cout << "\033[1;34m COM device : " << arr_device[i].data() << "\033[0m" << std::endl;
            note = false;
            connect_count = 0;
            return true;
        }
    }
    std::cout << "\033[1;31m Open COM failed,retry "<< connect_count << " \033[0m" << std::endl;
    connect_count++;
    if(!note)
    {
         std::cout << "\033[1;33m Cable disconnect or forget to add /xpkg_comm/scripts/CAN_COM_HUB.rules? \033[0m" << std::endl;
         note = true;
    }
    return false;
}

/*------------------------------------------------------------------------------------------------------------------
 * name: COMClose
 * detail: Close COM device
 -----------------------------------------------------------------------------------------------------------------*/
void CommINFCOM::COMClose()
{
    if(m_COM_fd < 0)return;
    close(m_COM_fd);
    m_COM_fd = -1;
}
/*------------------------------------------------------------------------------------------------------------------
 * name: COMSet
 * detail: Set COM parameter
 -----------------------------------------------------------------------------------------------------------------*/
bool CommINFCOM::COMSet(int baudrate, int flow_ctrl, int databits, int stopbits, int parity)
{
    if (COMIsOpen() == false)
        return false;
    struct termios options;
    if (tcgetattr(m_COM_fd, &options) != 0)
    {
        std::cout << "\033[1;31m COM options failed \033[0m" << std::endl;
        return (false);
    }
    ///////////////baudrate set//////////////////////////////////////
    for (unsigned long i = 0; i < sizeof(arr_baud) / sizeof(int); i++)
    {
        if (baudrate == arr_baud[i])
        {
            cfsetispeed(&options, arr_baud_set[i]);
            cfsetospeed(&options, arr_baud_set[i]);
            break;
        }
    }
    //修改控制模式，保证程序不会占用串口
    options.c_cflag |= CLOCAL;
    //修改控制模式，使得能够从串口中读取输入数据
    options.c_cflag |= CREAD;
    // 解决二进制传输时，数据遇到0x0d , 0x11,0x13 会被丢掉的问题
    options.c_iflag &= static_cast<unsigned int>(~(BRKINT | ICRNL | ISTRIP | IXON));
    ///////////////flow ctrl set//////////////////////////////////////
    switch (flow_ctrl)
    {
    case 0: //no ctrl
        options.c_cflag &= ~CRTSCTS;
        break;
    case 1: //hardware ctrl
        options.c_cflag |= CRTSCTS;
        break;
    case 2: //software ctrl
        options.c_cflag |= IXON | IXOFF | IXANY;
        break;
    }
    //////////databits set////////////////////////////////////////////
    options.c_cflag &= static_cast<unsigned int>(~CSIZE);
    switch (databits)
    {
    case 5:
        options.c_cflag |= CS5;
        break;
    case 6:
        options.c_cflag |= CS6;
        break;
    case 7:
        options.c_cflag |= CS7;
        break;
    case 8:
        options.c_cflag |= CS8;
        break;
    default:
        std::cout << "\033[1;31m Wrong databits \033[0m" << std::endl;
        return (false);
    }
    ////////////////parity set//////////////////////////////////////
    switch (parity)
    {
    case 'n':
    case 'N': //无奇偶校验位。
        options.c_cflag &= static_cast<unsigned int>(~PARENB);
        options.c_iflag &= static_cast<unsigned int>(~INPCK);
        break;
    case 'o':
    case 'O': //设置为奇校验
        options.c_cflag |= (PARODD | PARENB);
        options.c_iflag |= INPCK;
        break;
    case 'e':
    case 'E': //设置为偶校验
        options.c_cflag |= PARENB;
        options.c_cflag &= static_cast<unsigned int>(~PARODD);
        options.c_iflag |= INPCK;
        break;
    case 's':
    case 'S': //设置为空格
        options.c_cflag &= static_cast<unsigned int>(~PARENB);
        options.c_cflag &= static_cast<unsigned int>(~CSTOPB);
        break;
    default:
        std::cout << "\033[1;31m Wrong parity \033[0m" << std::endl;
        return (false);
    }
    ////////////////stopbits set//////////////////////////////////////
    switch (stopbits)
    {
    case 1:
        options.c_cflag &= static_cast<unsigned int>(~CSTOPB);
        break;
    case 2:
        options.c_cflag |= CSTOPB;
        break;
    default:
        std::cout << "\033[1;31m Wrong stopbits \033[0m" << std::endl;
        return (false);
    }

    //修改输出模式，原始数据输出
    options.c_oflag &= static_cast<unsigned int>(~OPOST);
    options.c_lflag &= static_cast<unsigned int>(~(ICANON | ECHO | ECHOE | ISIG));

    //设置等待时间和最小接收字符
    options.c_cc[VTIME] = 1; /* 读取一个字符等待1*(1/10)s */
    options.c_cc[VMIN] = 1;  /* 读取字符的最少个数为1 */

    //如果发生数据溢出，接收数据，但是不再读取 刷新收到的数据但是不读
    tcflush(m_COM_fd, TCIFLUSH);

    //激活配置 (将修改后的termios数据设置到串口中）
    if (tcsetattr(m_COM_fd, TCSANOW, &options) != 0)
    {
        std::cout << "\033[1;31m options set failed \033[0m" << std::endl;
        return (false);
    }
    return (true);
}
/*------------------------------------------------------------------------------------------------------------------
 * name: COMInitDefault
 * detail: Open and set COM default parameter
 -----------------------------------------------------------------------------------------------------------------*/
bool CommINFCOM::COMInitDefault()
{
    if(COMOpen() == false)return false;
    if(COMSet(460800,0,8,1,'n') == false)return false;
    tcflush(m_COM_fd, TCIOFLUSH); //flush IO quene
    return true;
}
/*------------------------------------------------------------------------------------------------------------------
 * name: COMDataSend
 * detail: As name.
 -----------------------------------------------------------------------------------------------------------------*/
long CommINFCOM::COMDataSend(vector<unsigned char> data)
{
    static int disconnect_count = 0;
    if(COMIsOpen()==false)return false;
    unsigned long data_len = data.size();
    long len = 0;
    if(data_len == 0)return false;    
    while(1)
    {
        len = write(m_COM_fd, &data[0], data_len);
        //ROS_INFO("xxcomm_com: %d",len);
        if (len == static_cast<long>(data_len))
        {
            disconnect_count = 0;
            return len;
        }
        //COM send error//////////////////////
        if(len == -1 || len == 0)
        {
            disconnect_count++;
            //disconnect handle
            if(disconnect_count == 100)
            {
                disconnect_count = 0;
                f_COM_disconnect = true;
                std::cout << "\033[1;31m COM disconnect with send error \033[0m" << std::endl;
                COMClose();
            }
            return false;
        }
        //not send all data//////////////////
        if (len < static_cast<long>(data_len) && len > 0)
        {
            std::cout << "\033[1;31m COM lost send data \033[0m" << std::endl;
            disconnect_count = 0;
            return len;
        }
        return len;
    }
}
/*------------------------------------------------------------------------------------------------------------------
 * name: DataRecv
 * detail: As name.
 -----------------------------------------------------------------------------------------------------------------*/
vector<unsigned char> CommINFCOM::COMDataRecv(void)
{
    static int disconnect_count = 0;
    fd_set fs_read;
    vector<unsigned char> recv_buff;
    unsigned char buff[100] = {0};
    struct timeval time;
    time.tv_sec = 0;
    time.tv_usec = 1000;
    long len;

    recv_buff.clear();
    if (COMIsOpen()==false)return recv_buff;
    while(1)
    {
        FD_ZERO(&fs_read);
        FD_SET(m_COM_fd, &fs_read);
        //fs_sel = select(m_COM_fd + 1, &fs_read, NULL, NULL, &time);//使用select实现串口的多路通信
        len = read(m_COM_fd, buff , 100);
        //available data/////////////////////////
        if(len > 0)
        {
            disconnect_count = 0;
            for(int i=0;i<len;i++)
            {
              recv_buff.push_back(buff[i]);
            }
            //too long data
            if(recv_buff.size()>2048)return recv_buff;
            continue;
        }
        //no more data///////////////////////////
        if(len == -1)
        {
            disconnect_count = 0;
            return recv_buff;
        }
        //COM error//////////////////////////////
        if(len == 0)
        {
            disconnect_count++;
            //std::cout << disconnect_count << std::endl;
            //lost connect handle
            if(disconnect_count == 100)
            {
                disconnect_count = 0;
                f_COM_disconnect = true;
                std::cout << "\033[1;31m COM disconnect due to com error \033[0m" << std::endl;
                COMClose();
            }
            return recv_buff;
        }
        return recv_buff;
    }
}
/*------------------------------------------------------------------------------------------------------------------
 * name: DataClear
 * detail: As name.
 -----------------------------------------------------------------------------------------------------------------*/
void CommINFCOM::DataClear()
{
    if (COMIsOpen()==false)return;
    tcflush(m_COM_fd, TCIOFLUSH);
}

