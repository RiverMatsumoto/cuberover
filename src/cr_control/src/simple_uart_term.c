#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <sys/select.h>
#include <fcntl.h>
#include <termios.h>

#include "cr_control/simple_uart.h"

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

int main(int argc, char *argv[])
{
	const char *port = "/dev/ttyTHS0";
	int baudrate = 115200;
	const char *flags = "8N1";
	int opt;

	if (!port) {
		print_usage(argv[0]);
		exit(1);
	}

	struct simple_uart *uart;

	uart = simple_uart_open(port, baudrate, flags);
	if (!uart) {
		fprintf(stderr, "Unable to open %s:%d:%s\n", port, baudrate, flags);
		exit(1);
	}
	// fprintf(stderr, "uart fd: %d\n", uart->fd);

	/* Disable buffering on stdin */
	struct termios tty;

	if (tcgetattr(uart->fd, &tty)) {
    	fprintf(stderr, "tcgetattr\n");
    	exit(-1);
  	}

    // save flag settings
  	if (tcsetattr(uart->fd, TCSANOW, &tty)) {
    	fprintf(stderr, "tcsetattr\n");
    	exit(-1);
  	}

	uint8_t readBuffer[128];
	uint8_t buffer[128];
	buffer[0] = 128;
	buffer[1] = 24;
	// Write a message to the UART port
	if (simple_uart_write(uart, buffer, 2) < 0) {
        fprintf(stderr, "Failed to write to UART\n");
        simple_uart_close(uart);
        exit(1);
    }

    // Now, wait for a response
    fd_set readfds;
    struct timeval tv;
    int nfds = simple_uart_get_fd(uart) + 1;
    FD_ZERO(&readfds);
    FD_SET(simple_uart_get_fd(uart), &readfds);

    // Set up a timeout (e.g., 5 seconds)
    tv.tv_sec = 5;
    tv.tv_usec = 0;

    int retval = select(nfds, &readfds, NULL, NULL, &tv);
    if (retval == -1) {
        perror("select()");
    } else if (retval) {
        // Data is available to read
        ssize_t len = simple_uart_read(uart, readBuffer, 6);
        if (len > 0) {
            readBuffer[len] = '\0'; // Null-terminate the string
            printf("Voltage: %d\n", (uint16_t)(readBuffer[0] << 8 | readBuffer[1]));
        } else {
            printf("No data was read\n");
        }
    } else {
        printf("No data within five seconds.\n");
    }

    simple_uart_close(uart);
    return 0;
}