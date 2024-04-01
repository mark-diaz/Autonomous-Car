#ifndef SENSOR_H
#define SENSOR_H

#include <stdint.h>
#include "msp.h"
#include "Encoder.h"
#include "QTRSensors.h"

void Sensor_Init();
void Sensor_read_IR(uint16_t *);

#endif
