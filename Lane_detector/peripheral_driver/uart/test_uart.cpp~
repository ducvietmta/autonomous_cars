
#include "api_uart.h"
#include <termios.h>
#include <iostream>
#include <unistd.h>
using namespace std;

int main()
{
 
    char buf_send[BUFF_SIZE],buf_recv[BUFF_SIZE];
    int cport_nr = api_uart_open();
    sprintf(buf_send, "ssss");
    api_uart_write(cport_nr, buf_send);
    if( cport_nr != -1 )
    {
        while(1)
        {
            sprintf(buf_send, "2020");
             api_uart_write(cport_nr, buf_send);
            printf("truyen: %s\n",buf_send);
            usleep(3000000);
            api_uart_read(cport_nr, buf_recv);
            printf("nhan: %s",buf_recv);
             sprintf(buf_send, "3030");
             api_uart_write(cport_nr, buf_send);
            printf("truyen: %s\n",buf_send);
            usleep(3000000);
api_uart_read(cport_nr, buf_recv);
            printf("nhan: %s",buf_recv);
             sprintf(buf_send, "ffff");
             api_uart_write(cport_nr, buf_send);
            printf("truyen: %s\n",buf_send);
            usleep(3000000);
            api_uart_read(cport_nr, buf_recv);
            printf("nhan: %s",buf_recv);
        }
    }

    return(0);
}
