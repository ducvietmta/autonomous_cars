#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>


#define dev "/dev/ttyACM0"

int main(void)
{
    int fd;
    struct termios old_termios;
    struct termios new_termios;

    fd = open(dev, O_RDWR | O_NOCTTY);
    if (fd < 0) {
        fprintf(stderr, "error, counldn't open file %s\n", dev);
        return 1;
    }
    if (tcgetattr(fd, &old_termios) != 0) {
        fprintf(stderr, "tcgetattr(fd, &old_termios) failed: %s\n", strerror(errno));
        return 1;
    }
    memset(&new_termios, 0, sizeof(new_termios));
    new_termios.c_iflag = IGNPAR;
    new_termios.c_oflag = 0;
    new_termios.c_cflag = CS8 | CREAD | CLOCAL | HUPCL;
    new_termios.c_lflag = 0;
    new_termios.c_cc[VINTR]    = 0;
    new_termios.c_cc[VQUIT]    = 0;
    new_termios.c_cc[VERASE]   = 0;
    new_termios.c_cc[VKILL]    = 0;
    new_termios.c_cc[VEOF]     = 4;
    new_termios.c_cc[VTIME]    = 0;
    new_termios.c_cc[VMIN]     = 1;
    new_termios.c_cc[VSWTC]    = 0;
    new_termios.c_cc[VSTART]   = 0;
    new_termios.c_cc[VSTOP]    = 0;
    new_termios.c_cc[VSUSP]    = 0;
    new_termios.c_cc[VEOL]     = 0;
    new_termios.c_cc[VREPRINT] = 0;
    new_termios.c_cc[VDISCARD] = 0;
    new_termios.c_cc[VWERASE]  = 0;
    new_termios.c_cc[VLNEXT]   = 0;
    new_termios.c_cc[VEOL2]    = 0;

    if (cfsetispeed(&new_termios, B9600) != 0) {
        fprintf(stderr, "cfsetispeed(&new_termios, B57600) failed: %s\n", strerror(errno));
        return 1;
    }
    if (cfsetospeed(&new_termios, B9600) != 0) {
        fprintf(stderr, "cfsetospeed(&new_termios, B57600) failed: %s\n", strerror(errno));
        return 1;
    }
    if (tcsetattr(fd, TCSANOW, &new_termios) != 0) {
        fprintf(stderr, "tcsetattr(fd, TCSANOW, &new_termios) failed: %s\n", strerror(errno));
        return 1;
    }


    // Now read() and write() to the device at your heart's delight

    char c;
    while (true)
    {
        for( int k = 0; k < 15; k++ )
        {
            for( int i = 0; i < 100; i++)
            {
                int serial_read_ret = read(fd, &c, 1);

                if(serial_read_ret < 1 )
                    printf("\nError\n");
                else
                {
                    printf("%c", c);
                    break;
                }
                usleep(100);
            }
            usleep(1000);
        }

        usleep(500000);
    }

    // Before leaving, reset the old serial settings.
    tcsetattr(fd, TCSANOW, &old_termios);
    return 0;
}
