#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdarg.h>
#include <string.h>
#include <ctype.h>
#include <getopt.h>
#include <errno.h>
#include <signal.h>
#include <sys/types.h>
#include <sys/poll.h>
#include <sys/stat.h>
#include <unistd.h>
#include <fcntl.h>
#include <alsa/asoundlib.h>

#include "erp_decode.h"

#define PAYLOAD_BUFFER_SIZE (100000ul)

#define RAW_TX_BUFFER_SIZE (PAYLOAD_BUFFER_SIZE * 2ul)  // need headroom for 8-to-7 bit encoding of the payload
#define RAW_RX_BUFFER_SIZE (PAYLOAD_BUFFER_SIZE * 2ul)  // should have headroom for 8-to-7 bit encoding of the payload
#define PARSER_BUFFER_SIZE (RAW_RX_BUFFER_SIZE)

#define SEND_WAIT_MS (1ul)  // milliseconds to wait when send buffer is going to overrun

typedef enum
{
  FALSE = 0,
  TRUE
} BOOL;

static BOOL           send;   // flag for program function: 1-->send , 0-->receive
static char const *   pName;  // port name
static snd_rawmidi_t *port;   // MIDI port
static BOOL           stop;

static BOOL dump = FALSE;

static void error(char const *const format, ...)
{
  va_list ap;

  va_start(ap, format);
  vfprintf(stderr, format, ap);
  va_end(ap);
  putc('\n', stderr);
}

static void cursorUp(uint8_t lines)
{
  char buffer[12];
  sprintf(buffer, "\033[%dA", lines);
  printf("%s", buffer);
}

static void usage(void)
{
  printf(
      "Usage: perf-test -s|-r port\n"
      "\n"
      "-s     : send test data\n"
      "-r     : receive test data\n"
      "port   : MIDI port to test (in hw:x,y,z notation, see ouput of 'amidi -l'\n");
}

static inline void getCmdLineParams(int const argc, char const *const argv[])
{
  if (argc != 3)
  {
    error("missing parameters\n");
    usage();
    exit(1);
  }

  if (strlen(argv[1]) == 2 && 0 == strcmp(argv[1], "-s"))
    send = TRUE;
  else if (strlen(argv[1]) == 2 && 0 == strcmp(argv[1], "-r"))
    send = FALSE;
  else
  {
    error("illegal parameter '%s'\n", argv[1]);
    usage();
    exit(1);
  }

  pName = argv[2];
}

static inline void openPort(void)
{
  int err;
  if (send)
    err = snd_rawmidi_open(NULL, &port, pName, 0);  // send will be blocking
  else
    err = snd_rawmidi_open(&port, NULL, pName, SND_RAWMIDI_NONBLOCK);
  if (err < 0)
  {
    error("cannot open port \"%s\": %s", pName, snd_strerror(err));
    exit(1);
  }
}

static inline void closePort(void)
{
  snd_rawmidi_close(port);
}

static void sigHandler(int dummy)
{
  stop  = TRUE;
  dummy = dummy;
}

static uint64_t getTimeUSec(void)
{
  struct timespec tp;

  if (clock_gettime(CLOCK_MONOTONIC, &tp))
    return 0;
  else
    return 1000000ul * (uint64_t)(tp.tv_sec) + (uint64_t)(tp.tv_nsec) / 1000ul;
}

//
// -------- functions for sending --------
//

static inline int encodeSysex(void const *const pSrc, int len, void *const pDest)
{
  uint8_t *src         = (uint8_t *) pSrc;
  uint8_t *dst         = (uint8_t *) pDest;
  uint8_t  topBitsMask = 0;
  uint8_t *topBits;

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
  return (void *) dst - pDest;  // total bytes written to destination buffer;
}

static inline void doSend(void)
{
  int     messageLen;
  uint8_t dataBuf[PAYLOAD_BUFFER_SIZE];
  uint8_t sendBuf[RAW_TX_BUFFER_SIZE];
  int     err, written;

  snd_rawmidi_status_t *pStatus;
  snd_rawmidi_status_alloca(&pStatus);

#if 01
  snd_rawmidi_params_t *pParams;
  snd_rawmidi_params_alloca(&pParams);

  if ((err = snd_rawmidi_params_set_buffer_size(port, pParams, sizeof sendBuf)) < 0)
  {
    error("cannot set send buffer size of %d : %s", sizeof sendBuf, snd_strerror(err));
    return;
  }
#endif

  if ((err = snd_rawmidi_nonblock(port, 0)) < 0)
  {
    error("cannot set blocking mode: %s", snd_strerror(err));
    return;
  }

  // messageLen = encodeSysex(dataBuf, PAYLOAD_SIZE, sendBuf);
  // printf("sysex size: %d\n", messageLen);
  uint8_t  runningCntr = 0;
  uint64_t messageNo   = 0;

  printf("Sending data to port: %s\n\n\n", pName);

  do
  {
    memset(dataBuf, runningCntr++, sizeof(dataBuf));
    if (runningCntr >= 128)
      runningCntr = 0;

    ((uint64_t *) dataBuf)[0] = getTimeUSec();
    ((uint64_t *) dataBuf)[1] = messageNo;

    unsigned expo = ((unsigned) rand()) % 11;  // 0..10
    unsigned size = 1 << expo;                 // 2^0...2^10(1024)
    size += ((unsigned) rand()) & (size - 1);  // 0..2047
    if (size & 1)
      size++;
    size += 24;
    ((uint64_t *) dataBuf)[2] = size;

    uint16_t *p   = (uint16_t *) &(dataBuf[24]);
    uint16_t  val = messageNo & 0xFFFF;
    for (unsigned i = 0; i < (size - 24) / 2; i++)
      *p++ = val++;

    cursorUp(1);
    printf("n:%lu, s:%5u\n", messageNo++, size);
    fflush(stdout);

    messageLen = encodeSysex(dataBuf, size, sendBuf);
    int total  = 0;

    uint64_t messageTime = getTimeUSec();
    while (total != messageLen)
    {
      int toTransfer = messageLen - total;

      if ((err = snd_rawmidi_status(port, pStatus)))
      {
        error("cannot get status: %s", snd_strerror(errno));
        return;
      }
      int avail = snd_rawmidi_status_get_avail(pStatus);
      if (0)  //toTransfer > avail)
      {
        error("rawmidi send larger than buffer by: %d", toTransfer - avail);
        usleep(1000ul * SEND_WAIT_MS);
        continue;
      }

      uint64_t time = getTimeUSec();
      written       = snd_rawmidi_write(port, &(sendBuf[total]), toTransfer);

      if ((err = snd_rawmidi_drain(port)) < 0)
      {
        error("cannot drain buffer: %s", snd_strerror(err));
        return;
      }
      if (written < 0)
      {
        error("cannot send data: %s", snd_strerror(written));
        return;
      }
      // printf("chunk of %d bytes transferred\n", written);

      total += written;
      time = getTimeUSec() - time;
      time *= 1 + (((unsigned) rand()) & 0xF);  // * 1..16
      //time /= 3;
      if (time > 1000ul * 1000ul)
        time = 1000ul * 1000ul;

      uint64_t sleepTime = getTimeUSec();
      usleep(time);
      sleepTime = getTimeUSec() - sleepTime;
      messageTime += sleepTime;
    }
    if (written != total)
    {
      ;  //printf("total of %d bytes transferred\n", total);
    }

    messageTime = getTimeUSec() - messageTime;

    static uint64_t min = ~(uint64_t) 0;
    static uint64_t max = 0;
    static uint32_t cnt = 0;
    static uint64_t sum = 0;

    if (messageTime > max)
      max = messageTime;
    if (messageTime < min)
      min = messageTime;
    sum += messageTime;
    cnt++;

    if (cnt == 1000)
    {
      cnt /= 2;
      sum /= 2;
    }
    if (cnt % 50 == 0)
    {
      cursorUp(2);
      printf("%6.2lfms(min) %6.1lfms(max) %6.1lfms(avg)\n\n",
             ((double) min) / 1000.0, ((double) max) / 1000.0, ((double) sum) / 1000.0 / cnt);
      fflush(stdout);
      max = max * 0.99;
      min = min / 0.80;
    }

  } while (TRUE);
}

//
// -------- functions for receiving --------
//

uint64_t packetCntr;

static inline BOOL examineContent(void const *const data, unsigned const len)
{
  if (len != 16)
  {
    error("receive: payload has wrong length %d, expected %d", len, 16);
    return FALSE;
  }

  static uint64_t packetNumber;

  static uint16_t adcValues[8];
  for (int i = 0; i < 8; i++)
    adcValues[i] = ((uint16_t *) data)[i];

  int delta = ERP_getIncrement(adcValues[0], adcValues[4]);

  static int sum;

  sum += delta;

  cursorUp(1);
  printf("#%lu\n", ++packetNumber);
  cursorUp(2);
  printf("wiper 1 =%4u, wiper 2 =%4u, a=%7.1lf \n\n",
         (unsigned) adcValues[0], (unsigned) adcValues[4],
         ERP_incrementTo360deg(sum));
  fflush(stdout);
  return TRUE;
}

static inline BOOL parseReceivedByte(uint8_t byte)
{
  static int step;

  static uint8_t  buf[PARSER_BUFFER_SIZE];
  static unsigned bufPos = 0;
  static uint8_t  topBitsMask;
  static uint8_t  topBits;

  switch (step)
  {
    case 0:  // wait for sysex begin
      if (byte == 0xF0)
      {
        bufPos      = 0;
        topBitsMask = 0;
        step        = 1;
        if (dump)
          printf("\n%2X ", byte);
      }
      return TRUE;
    case 1:              // sysex content
      if (byte == 0xF7)  // end of sysex ?
      {
        if (dump)
          printf("%02X\n", byte);
        step = 0;
        return examineContent(buf, bufPos);
      }
      if (dump)
        printf("%02X ", byte);
      if (byte & 0x80)
      {
        error("unexpected byte >= 0x80 within sysex!");
        return FALSE;
      }
      if (topBitsMask == 0)
      {
        topBitsMask = 0x40;  // reset top bit mask to first bit (bit6)
        topBits     = byte;  // save top bits
      }
      else
      {
        if (topBits & topBitsMask)
          byte |= 0x80;  // set top bit when required
        topBitsMask >>= 1;
        if (bufPos >= PARSER_BUFFER_SIZE)
        {
          error("unexpected buffer overrun (no sysex terminator found)!");
          return FALSE;
        }
        buf[bufPos++] = byte;
      }
      break;
  }
  return TRUE;
}

static inline void doReceive(void)
{
  int            npfds;
  struct pollfd *pfds;
  int            read;
  uint8_t        buf[RAW_RX_BUFFER_SIZE];
  int            err;
  unsigned short revents;

  snd_rawmidi_status_t *pStatus;
  snd_rawmidi_status_alloca(&pStatus);

#if 01
  snd_rawmidi_params_t *pParams;

  snd_rawmidi_params_alloca(&pParams);

  if ((err = snd_rawmidi_params_set_buffer_size(port, pParams, sizeof buf)) < 0)
  {
    error("cannot set receive buffer size of %d : %s", sizeof buf, snd_strerror(err));
    return;
  }
#endif

  if ((err = snd_rawmidi_nonblock(port, 1)) < 0)
  {
    error("cannot set non-blocking mode: %s", snd_strerror(err));
    return;
  }

  npfds = snd_rawmidi_poll_descriptors_count(port);
  pfds  = alloca(npfds * sizeof(struct pollfd));
  snd_rawmidi_poll_descriptors(port, pfds, npfds);
  signal(SIGINT, sigHandler);

  printf("Receiving data from port: %s\n\n\n", pName);

  do
  {
    err = poll(pfds, npfds, 0);               // poll as fast as possible
    if (stop || (err < 0 && errno == EINTR))  // interrupted ?
      break;
    if (err < 0)
    {
      error("poll failed: %s", strerror(errno));
      break;
    }

    if ((err = snd_rawmidi_poll_descriptors_revents(port, pfds, npfds, &revents)) < 0)
    {
      error("cannot get poll events: %s", snd_strerror(errno));
      break;
    }
    if (revents & (POLLERR | POLLHUP))
      break;
    if (!(revents & POLLIN))
      continue;

    read = snd_rawmidi_read(port, buf, sizeof buf);
    if (read == -EAGAIN)
      continue;  // nothing read yet

    if (read < 0)
    {
      error("cannot read from port \"%s\": %s", pName, snd_strerror(read));
      break;
    }

    if ((err = snd_rawmidi_status(port, pStatus)))
    {
      error("cannot get status: %s", snd_strerror(errno));
      break;
    }

    for (int i = 0; i < read; ++i)
    {
      if (!parseReceivedByte(buf[i]))  // parser error
      {
        for (int j = 0; j < read; j++)
        {
          if (j == i)
            printf(">>%02X<< ", buf[j]);  // mark the offending byte
          else
            printf("%02X ", buf[j]);
        }
        stop = TRUE;
      }
    }
    if ((err = snd_rawmidi_status_get_xruns(pStatus)))
    {
      error("rawmidi receive buffer overrun: %d", err);
      break;
    }
  } while (TRUE);

  printf("\n");
  fflush(stdout);
}

//
// ------------------------------------------
//
int main(int argc, char const *const argv[])
{
  getCmdLineParams(argc, argv);
  openPort();
  if (send)
    doSend();
  else
    doReceive();
  closePort();
  return 0;
}
