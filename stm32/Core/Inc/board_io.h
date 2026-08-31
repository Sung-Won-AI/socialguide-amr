#ifndef BOARD_IO_H
#define BOARD_IO_H

#include <stdbool.h>
#include <stdint.h>

#include "amr_app.h"

void BoardIO_Init(void);
void BoardIO_ReadHardwareInputs(AmrHardwareInputs *inputs_out);
bool BoardIO_IsReady(void);

/* Override this weak function when an ADC or BMS driver is available. */
uint16_t BoardIO_ReadBatteryVoltageMv(void);

#endif
