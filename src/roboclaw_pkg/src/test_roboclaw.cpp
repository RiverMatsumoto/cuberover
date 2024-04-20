#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <sys/select.h>
#include <fcntl.h>
#include <termios.h>
#include <csignal>

#include "roboclaw_pkg/roboclaw.hpp"
#include "roboclaw_pkg/simple_uart.h"
#include <iostream>

using namespace std;

struct simple_uart
{
    int fd;
    unsigned int char_delay_us;
    FILE *logfile;
};

static void print_usage(const char *program)
{
    fprintf(stderr, "%s [-p port] [-b baudrate] [-f flags] | -l\n", program);
}

int done = 0;
void end_program(int signum)
{
    done = 1;
}

int main(int argc, char *argv[])
{
    signal(SIGINT, end_program);
    // CONSTRUCTOR START
    // data
    const char *port = "/dev/ttyTHS0";
    int baudrate = 115200;
    const char *flags = "8N1";

    if (!port)
    {
        print_usage(argv[0]);
        exit(1);
    }

    struct simple_uart *uart;

    uart = simple_uart_open(port, baudrate, flags);
    if (!uart)
    {
        fprintf(stderr, "Unable to open %s:%d:%s\n", port, baudrate, flags);
        exit(1);
    }
    // fprintf(stderr, "uart fd: %d\n", uart->fd);

    /* Disable buffering on stdin */
    struct termios tty;
    tty.c_cflag &= ~PARENB;         // don't use parity bit
    tty.c_cflag &= ~CSTOPB;         // use 1 stop bit
    tty.c_cflag |= CS8;             // 8 bit characters
    tty.c_cflag |= CREAD | CLOCAL;  // ignore modem controls

    //cfmakeraw(&tty);
    tty.c_cflag &= ~CRTSCTS;        // no hardware flowcontrol
    tty.c_lflag &= ~ICANON;
    tty.c_lflag &= ~ECHO;
    tty.c_lflag &= ~ECHOE;
    tty.c_lflag &= ~ECHONL;
    tty.c_lflag &= ~ISIG;

    // setup for non canonical mode
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);

    tty.c_oflag &= ~OPOST;
    tty.c_oflag &= ~ONLCR;
    tty.c_cc[VTIME] = 0;
    tty.c_cc[VMIN]  = 1;

    if (tcgetattr(uart->fd, &tty))
    {
        fprintf(stderr, "tcgetattr\n");
    }

    // save flag settings
    if (tcsetattr(uart->fd, TCSANOW, &tty))
    {
        fprintf(stderr, "tcsetattr\n");
    }
    // CONSTRUCTOR END
    while (!done)
    {

        uint8_t readBuffer[128];
        uint8_t buffer[128];
        // START create msg and crc
        buffer[0] = 128;
        buffer[1] = 24;
        // END create msg and crc

        // Write a message to the UART port
        if (simple_uart_write(uart, buffer, 2) < 0)
        {
            fprintf(stderr, "Failed to write to UART\n");
            simple_uart_close(uart);
        }

        // Now, wait for a response
        fd_set readfds;
        struct timeval tv;
        int nfds = simple_uart_get_fd(uart) + 1;
        FD_ZERO(&readfds);
        FD_SET(simple_uart_get_fd(uart), &readfds);

        // Set up a timeout (e.g., 5 seconds)
        tv.tv_sec = 0;
        tv.tv_usec = 10000;

        int retval = select(nfds, &readfds, NULL, NULL, &tv);
        if (retval == -1)
        {
            perror("select()");
        }
        else if (retval)
        {
            // Data is available to read
            ssize_t len = simple_uart_read(uart, readBuffer, 6);
            if (len > 0)
            {
                // readBuffer[len] = '\0'; // Null-terminate the string
                float voltage = (readBuffer[0] << 8 | readBuffer[1]) / 10.0f;
                // printf("byte 1: %d\n", readBuffer[0]);
                // printf("byte 2: %d\n", readBuffer[1]);
                // printf("byte 3: %d\n", readBuffer[2]);
                // printf("byte 4: %d\n", readBuffer[3]);
                // printf("byte 5: %d\n", readBuffer[4]);
                // printf("byte 6: %d\n", readBuffer[5]);
                cout << "Voltage: " << voltage << endl;
            }
            else
            {
                printf("No data was read\n");
            }
        }
        else
        {
            printf("No data within five seconds.\n");
        }
        usleep(100000);
    }

    simple_uart_close(uart);
    return 0;
}