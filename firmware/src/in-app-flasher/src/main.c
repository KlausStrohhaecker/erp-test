#include <stdint.h>
#include "sys/nl_stdlib.h"
#include "drv/error_display.h"
#include "sys/flash.h"
#include "CPU_clock.h"
#include "io/pins.h"
#include "sys/nl_version.h"
#include "sys/nl_watchdog.h"

// these three symbols are provided by the dependent project "application",
// which first makes a binary image file (*.image) from it's compile
// and then a linkable object file (*_image.object) from that, which creates
// these automatic symbols.
extern uint8_t image_start;
extern uint8_t image_size;

// references needed to clear the zero-initialized data section,
// created by custom linker script
extern uint32_t *__bss_section_start;
extern uint32_t  __bss_section_size;

static uint32_t stack = 0x2000C000;  // stack at RamAHB16 (rwx) : ORIGIN = 0x20008000, LENGTH = 0x4000

// note the use of lower-case for 'firmware version:' to avoid the firmware version scanner finds this instead of the ID in the main image
static const char LOCAL_VERSION_STRING[] = "\n\nNLL MIDI Host-to-Host Bridge In-Application Flasher, LPC4337, firmware version: " SW_VERSION " \n\n\0\0\0";

volatile char dummy;

// must be public so compiler doesn't optimize it away
void dummyFunction(const char *string)
{
  while (*string)
  {
    dummy = *string++;
  }
}

__attribute__((section(".codeentry"))) int main(void)
{
  // Note we set up a new stack, to contain a 4kB buffer space, as we won't return anyway.
  // Initialized (.data) and zero-initialized (.bss) data segments are allowed.
  // Initialized data is not copied to RAM as we already reside in RAM.
  // Also note that the linker definitions for the RAM we are living in must be correct
  // and exactly the same for both this and the main project (see linker scripts).
  // Further, it must be a different RAM that the one used for the stack!
  // Currently, the RAM at RamLoc40 is used for our code/data (RamLoc40 (rwx) : ORIGIN = 0x10080000, LENGTH = 0xa000)

  __disable_irq();
  asm volatile("ldr sp, [%0]" ::"r"(&stack));  // setup our stack
  dummyFunction(LOCAL_VERSION_STRING);         // reference the string so compiler doesn't optimize it away
  PINS_Init();

  // zero the zero-initialized data section (.bss)
  uint32_t  count = __bss_section_size >> 2;  // get word count
  uint32_t *p     = __bss_section_start;
  while (count--)
    *(p++) = 0;

  CPU_ConfigureClocks();  // go full speed, to make the buffer copying as fast as possible for fail-safeness

  FLASH_Init();
  // flash into bank A
  int fail = flashMemory((uint32_t *) &image_start, (uint32_t) &image_size, 0);

  switch (fail)
  {
    case 0:
      SYS_WatchDogInit(10ul * 1000ul);  // 10 seconds until reboot
      DisplayErrorAndHalt(E_PROG_SUCCESS);
    case 1:
      DisplayErrorAndHalt(E_PROG_ZERO_DATA);
    case 2:
      DisplayErrorAndHalt(E_PROG_ERASE);
    case 3:
      DisplayErrorAndHalt(E_PROG_WRITEPREPARE);
    case 4:
      DisplayErrorAndHalt(E_PROG_WRITE);
    default:
      DisplayErrorAndHalt(E_CODE_ERROR);
  }

  // !! main must NEVER return as we've overwritten the caller's code and stack !!
}
