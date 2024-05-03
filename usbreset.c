/* usbreset -- send a USB port reset to a USB device */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <string.h>

#include <linux/usbdevice_fs.h>


int main(int argc, char **argv)
{
    FILE *fp;
    char path[2048];

    // Run the lsusb command and open the pipe for reading its output
    fp = popen("lsusb", "r");
    if (fp == NULL) {
        printf("Failed to run command\n");
        exit(1);
    }

    char bus[4];
    char device[4];

    // Read the output a line at a time and process it
    while (fgets(path, sizeof(path), fp) != NULL) {
        // Check if the current line contains the MAC address
        if (strstr(path, "2dc8:3106") != NULL) {
            printf("Found the line: %s", path);

            // Tokenize the line to split by spaces
            char *token;
            char *rest = path;
            int index = 0;
            while ((token = strtok_r(rest, " ", &rest))) {
                // Check if this is the token at index 3
                if (index == 1)
                {
                    printf("Token at index 1: %c\n", token[2]);
                    strncpy(bus, token, 3);
                }
                if (index == 3) {
                    printf("Token at index 3: %c\n", token[2]);
                    strncpy(device, token, 3);
                    break; // Exit the loop after finding the token
                }
                index++;
            }
            break; // Exit the loop after processing the correct line
        }
    }

    char *filepath = (char *)malloc(sizeof(char) * 128);
    strncat(filepath, "/dev/bus/usb/", 14);
    strncat(filepath, bus, 3);
    strncat(filepath, "/", 2);
    strncat(filepath, device, 3);
    printf("Path to 8bitdo controller: %s\n", filepath);

    // Close the pipe
    pclose(fp);

    // reset controller device
    int fd;
    int rc;

    fd = open(filepath, O_WRONLY);
    if (fd < 0) {
        perror("Error opening output file");
        return 1;
    }

    printf("Resetting USB device %s\n", filepath);
    rc = ioctl(fd, USBDEVFS_RESET, 0);
    if (rc < 0) {
        perror("Error in ioctl");
        return 1;
    }
    printf("Reset successful\n");

    close(fd);
    return 0;
}
