// Copyright (c) 2025 SCUTRobotLab
// SPDX-License-Identifier: MIT

#pragma once

#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

int wheelbipe_serial_open(const char* device, int baudrate);
int wheelbipe_serial_write(int fd, const void* data, size_t size);
void wheelbipe_serial_close(int fd);

#ifdef __cplusplus
}
#endif
