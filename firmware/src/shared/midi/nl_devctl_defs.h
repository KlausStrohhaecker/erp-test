#pragma once

#include <stdint.h>

// three-byte MIDI Manufacturer ID
// see https://www.midi.org/specifications-old/item/manufacturer-id-numbers
#define MMID0 (0)
#define MMID1 (0x21)
#define MMID2 (0x58)

#define DEVICEID (0x01)  // only 0x01 == MIDI Bridge, currently
#define MSGTYPE1 (0x44)  // 'D', D.evice
#define MSGTYPE2 (0x43)  // 'C', C.ontrol

#define CMD_GET_AND_PROG_L (0x01)  // command 0x0101 : get upload data ...
#define CMD_GET_AND_PROG_H (0x01)  // ... and flash it on receive finished
#define CMD_GET_AND_PROG   ((CMD_GET_AND_PROG_H << 8) | CMD_GET_AND_PROG_L)

#define CMD_LED_TEST_L (0x02)  // command 0x0102 : Test LEDs ...
#define CMD_LED_TEST_H (0x01)  // ... for all colors (also for alignment during assembly
#define CMD_LED_TEST   ((CMD_LED_TEST_H << 8) | CMD_LED_TEST_L)

// The ID is mandatory after each 0xF0 sysex start so that other devices will
// ignore the sysex properly in case it actually reaches the device. This can happen
// for example in the fw-uploader which issues an INFO request before it continue
// (if something is returned)

// a device control message structure is always :
// F0 00 4E 4C 01 44 43 cmdID-L cmdID-H [(possibly encoded) data bytes] F7
// 16bit command ID cmdID is comprised of low and high bytes, both of
// which always have bit7==0 to be legal
// depending on the command optional data bytes might follow, which may
// or may not be encoded 8-bit source data, the command parser is in charge for that

// "\0NL"
// sysex: f0 00 4e 4c 01 44 43 ...
static const uint8_t NLMB_DevCtlSignature[] = {
  0xF0, MMID0, MMID1, MMID2, DEVICEID, MSGTYPE1, MSGTYPE2
};

static const uint8_t NLMB_DevCtlSignature_RAW[] = {
  0x04,
  0xF0,
  MMID0,
  MMID1,
  0x04,
  MMID2,
  DEVICEID,
  MSGTYPE1,
  0x04,
  MSGTYPE2
};
