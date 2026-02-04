
#ifndef COMM_INF_TCP_H_
#define COMM_INF_TCP_H_

#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>
#include <fcntl.h>      //file ctrl define
#include <string>
#include <vector>
#include <stdio.h>
#include <stdlib.h>
/*------------------------------------------------------------------------------------------------------------------
*PROJECT OF XROS
*SETPS TO USE
 * 1.Use TCPAddrSet() to add TCP address
 * 2.Use TCPOpen() to start TCP connect
 * 3.Use TCPLoginSet() to set cloud user info(only for BEACON system)
 * 4.Use TCPLogin() to login in cloud(only for BEACON system)
 * 5.TCPDataRecv() will return data from vector buffer of max size 2048
 * 6.TCPDataSend() need input vector buffer
 * 7.Use TCPClose() to close channel
 * 8.Use TCPIsOpen() to return state of TCP
 -----------------------------------------------------------------------------------------------------------------*/

/*-----------------------------------------------------------------
 * default address
 * --------------------------------------------------------------*/


using namespace std;
class CommINFTCP
{
public:
    explicit CommINFTCP(void);
    ~CommINFTCP();
    void TCPLoginSet(std::string device_id, std::string password);
    void TCPLogin();
    void TCPAddrSet(std::string addr, uint16_t port);
    bool TCPOpen();
    void TCPClose();
    bool TCPIsOpen(void);
    vector<unsigned char> TCPDataRecv();
    long TCPDataSend(vector<unsigned char> data);

private:
    static void* WorkThread(void*);

private:
    bool m_f_open;
    bool m_f_tcp_disconnect;
    vector<unsigned char> m_recv_buff;
    vector<unsigned char> m_send_buff;
    std::string m_device_id;
    std::string m_password;
    std::string m_addr;
    uint16_t m_port;
    int m_socket_fd;

    std::string default_addr = {"115.29.240.46"};
    uint16_t default_port = 9000;
};

#endif  // COMM_INF_TCP_H_
