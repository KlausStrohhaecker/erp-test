#pragma once

#include <stdint.h>

uint16_t DEVCTL_isDeviceControlMsg(uint8_t** const pBuff, uint32_t* const pLen);
void     DEVCTL_init(uint8_t const port);
void     DEVCTL_processMsg(uint8_t* buff, uint32_t len);
