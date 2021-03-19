#include "MIDI_statemonitor.h"
#include "drv/nl_leds.h"
#include "sys/fwdisplay.h"

#define usToTicks(x) ((x + 75ul) / 125ul)     // usecs to 125 ticker counts
#define msToTicks(x) (((x) *1000ul) / 125ul)  // msecs to 125 ticker counts

#define LATE_TIME                     usToTicks(1125)  // time until packet is considered LATE
#define STALE_TIME                    msToTicks(10)    // time until packet is considered STALE
#define NORMAL_HOT_INDICATOR_TIMEOUT  msToTicks(20)    // minimum hot display duration after end of packets
#define DROPPED_HOT_INDICATOR_TIMEOUT msToTicks(300)   // hot display time of any dropped packets
#define LATE_INDICATOR_TIMEOUT        msToTicks(2000)  // display time of "had late packets recently"
#define STALE_INDICATOR_TIMEOUT       msToTicks(4000)  // display time of "had stale packets recently"
#define DROPPED_INDICATOR_TIMEOUT     msToTicks(6000)  // display time of "had dropped packets recently"
#define BLINK_TIME                    msToTicks(3000)  // blink cycle time for offline status display
#define BLINK_TIME_FIRST              msToTicks(4000)  // blink cycle time for offline status display
#define BLINK_TIME_ON_TIME            msToTicks(300)   // active portion of blink time
#define BLINK_TIME_OFF_TIME           (BLINK_TIME - BLINK_TIME_ON_TIME)

// PWM ratio for full dimmed LED, do NOT change without checking effect in doDisplay()
#define PWM_RELOAD (32)

typedef enum
{
  DIM = 0,
  NORMAL,
  BRIGHT,
} Brightness_t;

typedef enum
{
  SOLID = 0,
  BLINKING,
} Blink_t;

typedef enum
{
  REALTIME,
  LATE,
  STALE,
  DROPPED,
} Latency_t;

static struct
{
  int powered;  // flag: port is powered
  int online;   // flag: port is ready to use

  unsigned  packetTime;     // packet transmit running time
  Latency_t packetLatency;  // current latency range
  int       dropped;        // flag:packed had to be dropped

  unsigned packetDisplayTimer;   // packet display runtime
  unsigned lateDisplayTimer;     // hold time for "late packets" display
  unsigned staleDisplayTimer;    // hold time for "stale packets" display
  unsigned droppedDisplayTimer;  // hold time for "dropped packets" display
} state[2];

static struct
{
  LedColor_t   color;
  Brightness_t bright;
  Blink_t      blink;
} led[2];

static unsigned blinkCntr = BLINK_TIME;
static unsigned pwmCntr   = PWM_RELOAD;
static int      ledTest;     // flag: display LED test pattern instead of normal traffic
static int      ledDisable;  // flag

static inline void processLeds(void);
static inline void setupDisplay(uint8_t const port);
static inline void doDisplay(uint8_t const port);
static inline void setLED(uint8_t const port, LedColor_t const color, Brightness_t const bright, Blink_t const blink);
static inline void doTimers(void);
static inline void setLedDirect(uint8_t const port, LedColor_t const color);

// may/will be called from within interrupt callbacks !
void SMON_monitorEvent(uint8_t const port, MonitorEvent_t const event)
{
  switch (event)
  {
    case LED_TEST:
      ledTest = 1;
      break;

    case LED_DISABLE:
      ledDisable = 1;
      break;

    case UNPOWERED:
    case POWERED:
      state[port].powered = (event == POWERED);
      blinkCntr           = BLINK_TIME_FIRST;  // force blinker to restart with LED on and for a longer time so
      break;

    case OFFLINE:
      state[port].online = 0;
      blinkCntr          = BLINK_TIME_FIRST;  // force blinker to restart with LED on and for a longer time so
      break;

    case ONLINE:
      state[port].online = 1;
      break;

    case DROPPED_INCOMING:
      state[port].dropped = 1;
      break;

    case PACKET_START:
      state[port].packetTime = 1;  // start packet timer
      setupDisplay(port);          // start display ...
      doDisplay(port);             // ... immediately
      break;

    case PACKET_DELIVERED:
      state[port].packetTime = 0;
      break;

    case PACKET_DROPPED:
      state[port].dropped    = 1;
      state[port].packetTime = 0;
      break;
  }
}

void SMON_Process(void)
{
  processLeds();
  doTimers();
}

static inline void processLeds(void)
{
  setupDisplay(0);
  setupDisplay(1);

  if (ledTest)
  {
    static uint8_t  color = 0xFF;
    static unsigned cntr  = msToTicks(1500);

    if (!--cntr)
    {
      cntr = msToTicks(1500);
      color++;
    }
    LED_SetDirect(0, color);
    LED_SetDirect(1, color);
    return;
  }

  doDisplay(0);
  doDisplay(1);
}

static inline void doTimers(void)
{
  for (int port = 0; port < 2; port++)
  {
    if (state[port].online && state[port].packetTime)
      state[port].packetTime++;

    if (state[port].packetDisplayTimer)
      state[port].packetDisplayTimer--;

    if (state[port].lateDisplayTimer)
      state[port].lateDisplayTimer--;

    if (state[port].staleDisplayTimer)
      state[port].staleDisplayTimer--;

    if (state[port].droppedDisplayTimer)
      state[port].droppedDisplayTimer--;
  }

  if (!--pwmCntr)
    pwmCntr = PWM_RELOAD;
  if (!--blinkCntr)
    blinkCntr = BLINK_TIME;
}

static inline void setupDisplay(uint8_t const port)
{
  if (state[port].online)
  {
    if (state[port].packetTime)  // packet is running
    {
      state[port].packetDisplayTimer = NORMAL_HOT_INDICATOR_TIMEOUT;
      // set up current latency range and start their display timers
      if (state[port].packetTime < LATE_TIME)
        state[port].packetLatency = REALTIME;
      else if (state[port].packetTime < STALE_TIME)
      {
        state[port].packetLatency    = LATE;
        state[port].lateDisplayTimer = LATE_INDICATOR_TIMEOUT;
      }
      else
      {
        state[port].packetLatency     = STALE;
        state[port].staleDisplayTimer = STALE_INDICATOR_TIMEOUT;
      }

      switch (state[port].packetLatency)
      {
        case REALTIME:
          // during physical run time of the packet, display color
          // is governed by the strongest current history display state.
          if (state[port].droppedDisplayTimer)
            setLED(port, COLOR_MAGENTA, BRIGHT, SOLID);
          else if (state[port].staleDisplayTimer)
            setLED(port, COLOR_RED, BRIGHT, SOLID);
          else if (state[port].lateDisplayTimer)
            setLED(port, COLOR_YELLOW, BRIGHT, SOLID);
          else
            setLED(port, COLOR_CYAN, BRIGHT, SOLID);
          break;

        // the following cases are seldom met in normal operation, so they force
        // display color unconditionally
        case LATE:
          setLED(port, COLOR_YELLOW, BRIGHT, SOLID);
          break;

        case STALE:
          setLED(port, COLOR_RED, BRIGHT, SOLID);
          break;

        default:
          break;
      }
    }
    else
    {
      // no packet running, so first display the lengthened tail of the packet state,
      // then later the packet state history
      if (state[port].dropped)
      {
        state[port].dropped             = 0;
        state[port].packetDisplayTimer  = DROPPED_HOT_INDICATOR_TIMEOUT;
        state[port].droppedDisplayTimer = DROPPED_INDICATOR_TIMEOUT;
        state[port].packetLatency       = DROPPED;
      }

      if (state[port].packetDisplayTimer)  // packet display still running
      {
        switch (state[port].packetLatency)
        {
          case REALTIME:
            setLED(port, COLOR_GREEN, NORMAL, SOLID);
            break;
          case LATE:
            setLED(port, COLOR_YELLOW, NORMAL, SOLID);
            break;
          case STALE:
            setLED(port, COLOR_RED, NORMAL, SOLID);
            break;
          case DROPPED:
            setLED(port, COLOR_MAGENTA, BRIGHT, SOLID);
            break;
        }
      }
      else  // no packets, show latency history
      {
        if (state[port].droppedDisplayTimer)
          setLED(port, COLOR_MAGENTA, NORMAL, SOLID);
        else if (state[port].staleDisplayTimer)
          setLED(port, COLOR_RED, DIM, SOLID);
        else if (state[port].lateDisplayTimer)
          setLED(port, COLOR_YELLOW, DIM, SOLID);
        else
          setLED(port, COLOR_GREEN, DIM, SOLID);
      }
    }
  }
  else  // offline
  {
    if (state[port].powered)
      setLED(port, COLOR_CYAN, DIM, BLINKING);
    else
      setLED(port, COLOR_BLUE, DIM, BLINKING);
  }
}

static inline void doDisplay(uint8_t const port)
{
  int output = 0;
  switch (led[port].bright)
  {
    case DIM:
      output = (pwmCntr == PWM_RELOAD);  // 1:32 duty cycle
      break;
    case NORMAL:
      output = ((pwmCntr & 0b111) == 0);  // 1:8 duty cycle
      break;
    case BRIGHT:
      output = 1;  // 1:1 duty cycle
      break;
  }

  if (led[port].blink && (blinkCntr < BLINK_TIME_OFF_TIME))
    output = 0;
  if (output)
    setLedDirect(port, led[port].color);
  else
    setLedDirect(port, COLOR_OFF);
}

static inline void setLedDirect(uint8_t const port, LedColor_t const color)
{
  if (showFirmwareVersion())
    return;
  if (ledDisable)
    return;
  LED_SetDirect(port, color);
}

static inline void setLED(uint8_t const port, LedColor_t const color, Brightness_t const bright, Blink_t const blink)
{
  led[port].color  = color;
  led[port].bright = bright;
  led[port].blink  = blink;
}
