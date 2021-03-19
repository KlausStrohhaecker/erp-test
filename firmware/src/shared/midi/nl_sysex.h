#pragma once

#include <stdint.h>

// ---- functions for plain MIDI data
//
// return number of bytes of decoded data, 0 if error
uint16_t MIDI_decodeSysex(uint8_t const* const src, uint32_t const len, uint8_t* dest);
//
// return number of bytes of encoding
uint16_t MIDI_encodeSysex(uint8_t const* src, uint32_t len, uint8_t* const dest);

// ---- functions for raw USB MIDI data (@ USB driver level)
//
// return number of bytes of decoded raw data, 0 if error
uint16_t MIDI_decodeRawSysex(uint8_t const* const src, uint32_t const len, uint8_t* dest);
//
// return number of bytes of raw encoding
uint16_t MIDI_encodeRawSysex(uint8_t const* src, uint32_t len, uint8_t* const dest);
