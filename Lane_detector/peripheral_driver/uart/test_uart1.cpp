#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>

int main(int argc,char** argv)
{
        struct termios tio;
        struct termios stdio;
        struct termios old_stdio;
        int tty_fd;

        unsigned char c='D';
        tcgetattr(STDOUT_FILENO,&old_stdio);

        printf("Please start with %s /dev/ttyACM0 (for example)\n",argv[0]);
        memset(&stdio,0,sizeof(stdio));
        stdio.c_iflag=0;
        stdio.c_oflag=0;
        stdio.c_cflag=0;
        stdio.c_lflag=0;
        stdio.c_cc[VMIN]=1;
        stdio.c_cc[VTIME]=0;
        tcsetattr(STDOUT_FILENO,TCSANOW,&stdio);
        tcsetattr(STDOUT_FILENO,TCSAFLUSH,&stdio);
        fcntl(STDIN_FILENO, F_SETFL, O_NONBLOCK);       // make the reads non-blocking

        memset(&tio,0,sizeof(tio));
        tio.c_iflag=0;
        tio.c_oflag=0;
        tio.c_cflag=CS8|CREAD|CLOCAL;           // 8n1, see termios.h for more information
        tio.c_lflag=0;
        tio.c_cc[VMIN]=1;
        tio.c_cc[VTIME]=5;

        tty_fd=open("/dev/ttyACM0", O_RDWR | O_NONBLOCK);
        cfsetospeed(&tio,B9600);            // 115200 baud
        cfsetispeed(&tio,B9600);            // 115200 baud

        tcsetattr(tty_fd,TCSANOW,&tio);


        char serial_buffer_recv[16];

        for( int i = 0; i < 16; i++ )
            serial_buffer_recv[i] = '0';

        int nbyes = 15;

        while (true)
        {
            for( int i = 0; i < 100; i++)
            {
                int serial_read_ret = read(tty_fd, serial_buffer_recv, nbyes);

                if(serial_read_ret < 1 )
                    printf("\nError\n");
                else
                {
                    break;
                }
                usleep(100);
            }

            printf("%s\n\r", serial_buffer_recv);

            for( int i = 0; i < 15; i++ )
                serial_buffer_recv[i] = '0';

            usleep(450000);
        }

//        while (true)
//        {
//            for( int k = 0; k < 15; k++ )
//            {
//                for( int i = 0; i < 100; i++)
//                {
//                    int serial_read_ret = read(tty_fd, &serial_buffer_recv[k], 1);

//                    if(serial_read_ret < 1 )
//                        printf("\nError\n");
//                    else
//                    {
//                        break;
//                    }
//                    usleep(100);
//                }
//                usleep(1000);
//            }


//            printf("Read from serial port: %s\n\r", serial_buffer_recv);


//            for( int i = 0; i < 15; i++ )
//                serial_buffer_recv[i] = '0';

//            usleep(500000);
//        }

        close(tty_fd);
        tcsetattr(STDOUT_FILENO,TCSANOW,&old_stdio);

        return EXIT_SUCCESS;
}
