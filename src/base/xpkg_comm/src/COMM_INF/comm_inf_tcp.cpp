#include "comm_inf_tcp.h"
#include <cstring>
#include <iostream>
#include <stdio.h>
#include <netinet/tcp.h>

CommINFTCP::CommINFTCP(void)
{
    m_f_open = false;
    m_f_tcp_disconnect = false;
    m_addr.clear();
    m_port = 0;
}
CommINFTCP::~CommINFTCP()
{
    m_f_open = false;
    TCPClose();
    sleep(1);
}
/*------------------------------------------------------------------------------------------------------------------
 * name: TCPAddrSet
 -----------------------------------------------------------------------------------------------------------------*/
void CommINFTCP::TCPAddrSet(std::string addr, uint16_t port)
{
    m_addr = addr;
    m_port = port;
}
/*------------------------------------------------------------------------------------------------------------------
 * name: TCPLoginSet
 -----------------------------------------------------------------------------------------------------------------*/
void CommINFTCP::TCPLoginSet(std::string device_id, std::string password)
{
    m_device_id = device_id;
    m_password = password;
}
/*------------------------------------------------------------------------------------------------------------------
 * name: TCPLogin
 -----------------------------------------------------------------------------------------------------------------*/
void CommINFTCP::TCPLogin()
{
    std::string register_string = "ep=" + std::string(m_device_id) + "&pw=" + std::string(m_password);
    write(m_socket_fd, register_string.c_str(), register_string.size());
}
/*------------------------------------------------------------------------------------------------------------------
 * name: TCPIsOpen
 -----------------------------------------------------------------------------------------------------------------*/
bool CommINFTCP::TCPIsOpen(void)
{
    return m_f_open;
}
/*------------------------------------------------------------------------------------------------------------------
 * name: TCPOpen
 -----------------------------------------------------------------------------------------------------------------*/
bool CommINFTCP::TCPOpen()
{
    // initialize socket
    m_socket_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (m_socket_fd == -1)
    {
        std::cout << "\033[1;31m socket create failed \033[0m" << std::endl;
        m_f_open = false;
        return false;
    }
    std::cout << "\033[1;32m socket create success \033[0m" << std::endl;

    //set default addr
    if(m_addr.size() == 0)
    {
        m_addr = default_addr;
        m_port = default_port;
    }

    // initialize socket address
    struct sockaddr_in socket_address;
    socket_address.sin_family = AF_INET;
    socket_address.sin_port = htons(m_port);
    socket_address.sin_addr.s_addr = inet_addr(m_addr.data());

    // bind socket
    int res = connect(m_socket_fd, (struct sockaddr*)&socket_address,sizeof(socket_address));
    if (res == -1)
    {
        std::cout << "\033[1;31m socket connect failed \033[0m" << std::endl;
        m_f_open = false;
        return false;
    }
    std::cout << "\033[1;32m TCP connect success \033[0m" << std::endl;
    std::cout << "\033[1;34m TCP IP : " << m_addr.data() <<"port : "<<m_port<<"\033[0m" << std::endl;
    m_f_open = true;

    // 设置非阻塞
    int flags = fcntl(m_socket_fd, F_GETFL, 0);
    fcntl(m_socket_fd, F_SETFL, flags | O_NONBLOCK);
    int flag = 1;
    setsockopt(m_socket_fd, IPPROTO_TCP, TCP_NODELAY, &flag, sizeof(int));        
    return true;
}
/*------------------------------------------------------------------------------------------------------------------
 * name: TCPClose
 -----------------------------------------------------------------------------------------------------------------*/
void CommINFTCP::TCPClose()
{
    if(m_socket_fd < 0)return;
    close(m_socket_fd);
    m_socket_fd = -1;
    m_f_open = false;
}
/*------------------------------------------------------------------------------------------------------------------
 * name: TCPDataRecv
 -----------------------------------------------------------------------------------------------------------------*/
vector<unsigned char> CommINFTCP::TCPDataRecv()
{
    static int disconnect_count = 0;
    vector<unsigned char> recv_buff;
    unsigned char buff[100] = {0};
    long len;
    recv_buff.clear();
    if (m_f_open==false)return recv_buff;

    while(1)
    {
        len = read(m_socket_fd, buff, 100);        
        //available data///////////////////////
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
        //TCP diconnect////////////////////////
        if(len == 0)
        {
            disconnect_count++;
            if(disconnect_count == 500)
            {
                m_f_tcp_disconnect = true;
                std::cout << "\033[1;31m TCP diconnect with error \033[0m" << std::endl;
                TCPClose();
            }
            return recv_buff;
        }
        //TCP no data//////////////////////////
        if(len == -1)
        {            
            disconnect_count = 0;
            return recv_buff;
        }        
    }
}
/*------------------------------------------------------------------------------------------------------------------
 * name: TCPDataSend
 -----------------------------------------------------------------------------------------------------------------*/
long CommINFTCP::TCPDataSend(vector<unsigned char> data)
{
    static int disconnect_count = 0;
    if (m_f_open==false)return false;
    unsigned long data_len = data.size();
    long len = 0;
    if(data_len == 0)return false;
    while(1)
    {
        len = write(m_socket_fd, &data[0], data_len);
        //ROS_INFO("xxcomm_tcp: %d",len);
        //send all data/////////////////////////////
        if (len == static_cast<long>(data_len))
        {
            disconnect_count = 0;
            m_f_tcp_disconnect = false;
            return len;
        }
        //send error////////////////////////////////
        if(len == -1 || len == 0)
        {
            disconnect_count++;
            //disconnect handle
            if(disconnect_count == 100)
            {
                disconnect_count = 0;
                m_f_tcp_disconnect = true;
                std::cout << "\033[1;31m TCP diconnect with send error \033[0m" << std::endl;
                TCPClose();                
            }
            return false;
        }
        //not send all data//////////////////////////
        if (len < static_cast<long>(data_len) && len > 0)
        {
            std::cout << "\033[1;31m TCP lost send data \033[0m" << std::endl;
            disconnect_count = 0;
            return len;
        }
        return len;
    }
}

