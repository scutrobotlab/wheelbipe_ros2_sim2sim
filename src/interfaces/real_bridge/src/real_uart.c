// Copyright (c) 2025 SCUTRobotLab
// SPDX-License-Identifier: MIT

#include "real_bridge/real_uart.h"

#include <errno.h>
#include <fcntl.h>
#include <stdint.h>
#include <termios.h>
#include <unistd.h>

static speed_t termios_baudrate(int baudrate) {
  switch (baudrate) {
    case 9600:
      return B9600;
    case 115200:
      return B115200;
#ifdef B460800
    case 460800:
      return B460800;
#endif
#ifdef B921600
    case 921600:
      return B921600;
#endif
#ifdef B1000000
    case 1000000:
      return B1000000;
#endif
#ifdef B2000000
    case 2000000:
      return B2000000;
#endif
    default:
      errno = EINVAL;
      return (speed_t)0;
  }
}

int wheelbipe_serial_open(const char* device, int baudrate) {
  if (device == NULL) {
    errno = EINVAL;
    return -1;
  }
  const speed_t speed = termios_baudrate(baudrate);
  if (speed == (speed_t)0) {
    return -1;
  }

  const int fd = open(device, O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd < 0) {
    return -1;
  }

  struct termios tty;
  if (tcgetattr(fd, &tty) != 0) {
    close(fd);
    return -1;
  }
  cfmakeraw(&tty);
  cfsetospeed(&tty, speed);
  cfsetispeed(&tty, speed);
  tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
  tty.c_cflag |= CLOCAL | CREAD;
  tty.c_cflag &= ~(PARENB | PARODD | CSTOPB | CRTSCTS);
  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 0;
  if (tcsetattr(fd, TCSANOW, &tty) != 0) {
    close(fd);
    return -1;
  }
  tcflush(fd, TCIOFLUSH);
  return fd;
}

int wheelbipe_serial_write(int fd, const void* data, size_t size) {
  if (fd < 0 || data == NULL || size == 0) {
    errno = EINVAL;
    return -1;
  }
  const uint8_t* bytes = (const uint8_t*)data;
  size_t sent = 0;
  while (sent < size) {
    const ssize_t result = write(fd, bytes + sent, size - sent);
    if (result > 0) {
      sent += (size_t)result;
      continue;
    }
    if (result < 0 && errno == EINTR) {
      continue;
    }
    if (result < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
      return -1;
    }
    if (result == 0) {
      errno = EIO;
    }
    return -1;
  }
  return (int)sent;
}

void wheelbipe_serial_close(int fd) {
  if (fd >= 0) {
    close(fd);
  }
}
