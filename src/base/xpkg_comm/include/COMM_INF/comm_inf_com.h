
#ifndef COMM_INF_COM_H
#define COMM_INF_COM_H

#include <string>
#include <vector>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>      //file ctrl define
#include <termios.h>    //PPSIX terminal define
#include <errno.h>      //error define
using namespace std;

/*------------------------------------------------------------------------------------------------------------------
*PROJECT OF XROS
*SETPS TO USE
 * 1.Use COMOpen() to add new channel
 * 2.Use COMSet() to set parameter
 * 3.COMDataRecv() will return data from vector buffer of max size 2048
 * 4.COMDataSend() need input vector buffer
 * 5.Use COMClose() to close channel
 * 6.Use COMIsOpen() to return state of COM
 * 7.Use DataClear() to clear remain date in COM buffer
 * PS: If use default mod, please use COMInitDefault() to start without COMOpen() and COMSet()
 -----------------------------------------------------------------------------------------------------------------*/

/*-----------------------------------------------------------------
 * Serial device sequence
 * Add your needed device here
 * --------------------------------------------------------------*/
static std::string arr_device[] = {"/dev/CAN2COM_HUB","/dev/NANO_USART","/dev/ttyUSB0","/dev/ttyS0"};




class CommINFCOM
{
public:
    explicit CommINFCOM(void);
    ~CommINFCOM();

    bool COMOpen(void);
    bool COMIsOpen(void);
    void COMClose(void);
    bool COMSet(int baudrate, int flow_ctrl, int databits, int stopbits, int parity);
    bool COMInitDefault(void);
    vector<unsigned char> COMDataRecv(void);
    long COMDataSend(vector<unsigned char> data);
    void DataClear(void);

private:
    int m_COM_fd;           //COM feedback ID
    bool f_COM_disconnect;  //true = disconnect

    int arr_baud[10] = {460800, 115200, 57600, 38400, 19200, 9600, 4800, 2400, 1200, 300};
    unsigned int arr_baud_set[10] = {B460800, B115200, B57600, B38400, B19200, B9600, B4800, B2400, B1200, B300};

};

#endif // COMM_INF_COM_H
