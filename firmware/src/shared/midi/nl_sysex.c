#include "midi/nl_sysex.h"
#include "sys/nl_stdlib.h"

// functions for plain MIDI data

// return number of bytes of decoded data, 0 if error
uint16_t MIDI_decodeSysex(uint8_t const* const src, uint32_t const len, uint8_t* dest)
{
  if ((len < 2) || (src[0] != 0xF0))  // too short or not a sysex ?
    return 0;
  uint16_t i           = 1;  // we will be beyond the F0 header already
  uint8_t  topBitsMask = 0;
  uint8_t  topBits;
  uint8_t  byte;
  uint8_t* savedDest = dest;

  while ((byte = src[i]) != 0xF7)  // wait for sysex tail
  {
    if (topBitsMask == 0)
    {
      topBitsMask = 0x40;    // reset top bit mask to first bit (bit6)
      topBits     = src[i];  // save top bits
    }
    else
    {
      if (topBits & topBitsMask)
        byte |= 0x80;  // set top bit when required
      topBitsMask >>= 1;
      *(dest++) = byte;
    }
    i++;
    if (i >= len)  // buffer ends before sysex tail was found ?
      return 0;
  }
  i++;
  if (i != len)  // when there is more data than one single sysex this is an error, too
    return 0;

  return dest - savedDest;
}

// return number of bytes of encoding
uint16_t MIDI_encodeSysex(uint8_t const* src, uint32_t len, uint8_t* const dest)
{
  uint8_t* dst         = (uint8_t*) dest;
  uint8_t  topBitsMask = 0;
  uint8_t* topBits     = NULL;

  // write start of sysex
  *(dst++) = 0xF0;

  while (len)
  {
    if (topBitsMask == 0)  // need to start insert a new top bits byte ?
    {
      topBitsMask = 0x40;   // reset top bit mask to first bit (bit6)
      topBits     = dst++;  // save top bits position...
      *topBits    = 0;      // ...and clear top bits
    }
    else  // regular byte
    {
      uint8_t byte = *(src++);
      *(dst++)     = byte & 0x7F;
      if (byte & 0x80)
        *topBits |= topBitsMask;  // add in top bit for this byte
      topBitsMask >>= 1;
      len--;
    }
  }

  // write end of sysex
  *(dst++) = 0xF7;
  return dst - dest;  // total bytes written to code buffer;
}

// functions for raw USB MIDI data (@ USB driver level)

// we assume a sysex stream that is organized in blocks like this:
// top-bits byte0 [byte1 byte2 byte3 byte4 byte5 byte6]
// top-bits bit6..0 contains the top bits for (up to) 6 following bytes
// So, 8 bytes, all with bit7==0 as required by midi-protocol, contain 7 net bytes
//
// Note that the input buffer is raw USB-MIDI data which is always 4-byte mutliples,
// the first byte of each block containing a cable-number and data-type ID which has
// to be discarded

// return number of bytes of decoded data, 0 if error
uint16_t MIDI_decodeRawSysex(uint8_t const* const src, uint32_t const len, uint8_t* dest)
{
  if ((len < 4) || (src[1] != 0xF0))  // too short or not a sysex ?
    return 0;
  uint16_t i           = 2;  // we will be beyond the F0 header already
  uint8_t  topBitsMask = 0;
  uint8_t  topBits;
  uint8_t  byte;
  uint8_t* savedDest = dest;

  while ((byte = src[i]) != 0xF7)  // wait for sysex tail
  {
    if (topBitsMask == 0)
    {
      topBitsMask = 0x40;    // reset top bit mask to first bit (bit6)
      topBits     = src[i];  // save top bits
    }
    else
    {
      if (topBits & topBitsMask)
        byte |= 0x80;  // set top bit when required
      topBitsMask >>= 1;
      *(dest++) = byte;
    }
    i++;
    if (i >= len)  // buffer ends before sysex tail was found ?
      return 0;
    if ((i & 0b11) == 0)  // on USB raw frame header byte ?
      i++;                // then skip it
  }
  while ((i & 0b11) != 0)  // advance to next packet ...
    i++;                   // .. boundary, if required
  if (i != len)            // when there is more data than one single sysex this is an error, too
    return 0;
  return dest - savedDest;
}

// return number of bytes of raw encoding
uint16_t MIDI_encodeRawSysex(uint8_t const* src, uint32_t len, uint8_t* const dest)
{
  uint8_t* packet = (uint8_t*) dest;
  uint8_t  packetPayloadIndex;
  uint8_t  topBitsMask = 0;
  uint8_t* topBits     = NULL;

  // write start of sysex
  packet[0]          = 0x04;
  packet[1]          = 0xF0;
  packetPayloadIndex = 2;

  while (len)
  {
    if (topBitsMask == 0)  // need to start insert a new top bits byte ?
    {
      topBitsMask = 0x40;                           // reset top bit mask to first bit (bit6)
      topBits     = &packet[packetPayloadIndex++];  // save top bits position...
      *topBits    = 0;                              // ...and clear top bits
    }
    else  // regular byte
    {
      uint8_t byte                 = *(src++);
      packet[packetPayloadIndex++] = byte & 0x7F;
      if (byte & 0x80)
        *topBits |= topBitsMask;  // add in top bit for this byte
      topBitsMask >>= 1;
      len--;
    }

    if (packetPayloadIndex == 4)  // packet full
    {
      packet[0]          = 0x04;
      packetPayloadIndex = 1;
      packet += 4;
    }
  }

  // write end of sysex
  packet[packetPayloadIndex++] = 0xF7;
  switch (packetPayloadIndex - 1)
  {
    case 1:
      packet[0] = 0x05;
      packet[2] = packet[3] = 0;
      break;
    case 2:
      packet[0] = 0x06;
      packet[3] = 0;
      break;
    case 3:
      packet[0] = 0x07;
      break;
  }

  return &packet[4] - dest;  // total bytes written to code buffer;
}
