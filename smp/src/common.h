#ifndef COMMON_H
#define COMMON_H

#include <zephyr/shell/shell.h>

void start_smp_bluetooth_adverts(void);
int start_smp_udp(void);
void start_smp_sensor(const struct shell *shell);
void stop_smp_sensor(void);
int sensor_read_once(char *buf, size_t len);

#endif