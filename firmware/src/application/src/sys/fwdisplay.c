#include "sys/nl_version.h"
#include "drv/nl_leds.h"

#define SHORT  (250000ul / 125)
#define MEDIUM (500000ul / 125)
#define LONG   (1000000ul / 125)

// returns 1 as long as display is still not finished
int showFirmwareVersion(void)
{
  static int step  = 1;
  static int major = SW_VERSION_MAJOR;
  static int minor = SW_VERSION_MINOR_H * 10 + SW_VERSION_MINOR_L;
  static int wait  = 0;

  if (step == 0)
    return 0;

  if (wait)
  {
    wait--;
    return 1;
  }

  switch (step)
  {
    case 1:  // both LEDs off for a while
      LED_SetDirect(0, 0);
      LED_SetDirect(1, 0);
      wait = LONG;
      step++;
      break;

    case 2:  // LED display of major number
      LED_SetDirect(0, 0b011);
      wait = MEDIUM;  // on time
      step++;
      break;

    case 3:
      LED_SetDirect(0, 0b000);
      if (--major)
      {
        step = 2;       // repeat
        wait = MEDIUM;  // off time
      }
      else
      {
        step++;
        wait = LONG;  // delimiter
      }
      break;

    case 4:  // LED display of minor number
      LED_SetDirect(1, 0b110);
      wait = MEDIUM;  // on time
      step++;
      break;

    case 5:
      LED_SetDirect(1, 0b000);
      if (--minor)
      {
        step = 4;       // repeat
        wait = MEDIUM;  // off time
      }
      else
      {
        step = 6;
        wait = LONG;  // delimiter
      }
      break;

    case 6:
      step = 0;  // done
      break;
  }
  return 1;  // still displaying
}
