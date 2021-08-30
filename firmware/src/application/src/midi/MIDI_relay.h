#pragma once

#include <inttypes.h>

void MIDI_Relay_Init(void);
void MIDI_Relay_ProcessFast(void);
void MIDI_Relay_Process(void);

int  ReadyForErpTransfer(void);
void SendERP(uint8_t *buff, uint32_t len);
