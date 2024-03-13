#!/usr/bin/bash
sudo systemctl stop nvgetty && \
sudo systemctl disable nvgetty && \
sudo udevadm trigger
sudo usermod -a -G tty,dialout,gpio $USER