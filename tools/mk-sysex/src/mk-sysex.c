#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdarg.h>
#include <unistd.h>

#include "midi/nl_devctl_defs.h"

static void error(char const *const format, ...)
{
  va_list ap;
  fflush(stdout);
  va_start(ap, format);
  vfprintf(stderr, format, ap);
  va_end(ap);
  putc('\n', stderr);
  fflush(stderr);
}

static void usage(void)
{
  fflush(stderr);
  puts("Usage: mk-sysex <in-file> [<out-file>]");
  puts(" Generate a sysex (.syx) binary file from a binary input file,");
  puts(" to be used to upload the binary to the NLL Midi Bridge.");
  puts(" Payload data is encoded with the NLL 8-to-7 bit scheme.");
  puts(" If <out-file> is not given a name for it will be generated");
  puts(" as 'nlmb-fw-update-VX.YZ.syx', where X.YZ is the version ID");
  puts(" found in the input file");
}

static char fwVersion[] = "X.YZ";

void scanVersionString(uint8_t const byte)
{
  static int step = 0;
  switch (step)
  {
    case 0:
      if (byte == 'V')
        step++;
      else
        step = 0;
      break;

    case 1:
      if (byte == 'E')
        step++;
      else
        step = 0;
      break;

    case 2:
      if (byte == 'R')
        step++;
      else
        step = 0;
      break;

    case 3:
      if (byte == 'S')
        step++;
      else
        step = 0;
      break;

    case 4:
      if (byte == 'I')
        step++;
      else
        step = 0;
      break;

    case 5:
      if (byte == 'O')
        step++;
      else
        step = 0;
      break;

    case 6:
      if (byte == 'N')
        step++;
      else
        step = 0;
      break;

    case 7:
      if (byte == ':')
        step++;
      else
        step = 0;
      break;

    case 8:
      if (byte == ' ')
        step++;
      else
        step = 0;
      break;

    case 9:
      fwVersion[0] = byte;
      step++;
      break;

    case 10:
      step++;
      break;

    case 11:
      fwVersion[2] = byte;
      step++;
      break;

    case 12:
      fwVersion[3] = byte;
      step++;
      break;

    default:
      break;
  }
}

#pragma pack(push, 1)
static struct
{
  uint8_t topBits;
  uint8_t lsbytes[7];
} encodeData;
#pragma pack(pop)

static uint8_t topBitsMask;
static int     fillCount;

static void flushAnyRemaingBuffer(FILE *outfile)
{
  if (fillCount != 0)
  {
    if (1 != fwrite(&encodeData, fillCount + 1, 1, outfile))
    {
      error("could not write to output file!");
      exit(3);
    }
  }
}

static void encodeAndWrite(uint8_t *buffer, size_t len, FILE *outfile)
{
  while (len--)
  {
    if (topBitsMask == 0)  // need to start insert a new top bits byte ?
    {
      topBitsMask        = 0x40;  // reset top bit mask to first bit (bit6)
      encodeData.topBits = 0;     // ...and clear top bits
      fillCount          = 0;
    }
    uint8_t byte = *(buffer++);
    scanVersionString(byte);
    if (byte & 0x80)
      encodeData.topBits |= topBitsMask;  // add in top bit for this byte
    topBitsMask >>= 1;

    byte &= 0x7F;
    encodeData.lsbytes[fillCount] = byte;
    fillCount++;
    if (fillCount == 7)
    {
      if (1 != fwrite(&encodeData, sizeof(encodeData), 1, outfile))
      {
        error("could not write to output file!");
        exit(3);
      }
      fillCount = 0;
    }
  }
}

int main(int argc, char *argv[])
{
  if (argc != 2 && argc != 3)
  {
    error("Wrong number of arguments!");
    usage();
    return 3;
  }

  char autoName[] = "nlmb-fw-update-VX.YZ.syx";

  char *outFileName;
  if (argc == 3)
    outFileName = argv[2];
  else
    outFileName = "__temp__";

  FILE *inFile;
  FILE *outFile;
  if (NULL == (inFile = fopen(argv[1], "rb")))
  {
    error("Could not open input file %s", argv[1]);
    return 3;
  }
  if (NULL == (outFile = fopen(outFileName, "wb")))
  {
    error("Could not open output file %s", argv[2]);
    return 3;
  }

#define BLOCKSIZE (4096)
  uint8_t inBuffer[BLOCKSIZE];  // read blocks of data from file

  puts(" writing device control header...");
  if (1 != fwrite(NLMB_DevCtlSignature, sizeof(NLMB_DevCtlSignature), 1, outFile))
  {
    error("could not write to output file!");
    return 3;
  }
  uint16_t cmd = CMD_GET_AND_PROG;
  if (1 != fwrite(&cmd, sizeof(cmd), 1, outFile))
  {
    error("could not write to output file!");
    return 3;
  }

  puts(" encoding payload...");
  size_t bytes;
  size_t total = 0;
  while (BLOCKSIZE == (bytes = fread(inBuffer, 1, BLOCKSIZE, inFile)))
  {
    encodeAndWrite(inBuffer, BLOCKSIZE, outFile);
    total += BLOCKSIZE;
  }
  encodeAndWrite(inBuffer, bytes, outFile);
  flushAnyRemaingBuffer(outFile);
  total += bytes;
  printf(" encoded %zu bytes of payload data.\n", total);
  fclose(inFile);
  printf(" firmware version ID found: %s\n", fwVersion);

  uint8_t byte = 0xF7;
  if (1 != fwrite(&byte, sizeof(byte), 1, outFile))
  {
    error("could not write to output file!");
    return 3;
  }
  fclose(outFile);

  if (argc == 2)
  {
    autoName[16] = fwVersion[0];
    autoName[18] = fwVersion[2];
    autoName[19] = fwVersion[3];
    rename(outFileName, autoName);
  }
  return 0;
}
