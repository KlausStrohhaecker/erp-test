#include "midi/MIDI_relay.h"
#include "usb/nl_usb_midi.h"
#include "usb/nl_usb_core.h"
#include "io/pins.h"
#include "drv/error_display.h"
#include "sys/nl_stdlib.h"
#include "sys/ticker.h"
#include "midi/MIDI_statemonitor.h"
#include "devctl/devctl.h"
#include "midi/nl_devctl_defs.h"
#include "midi/nl_sysex.h"

#define usToTicks(x) ((x + 75ul) / 125ul)     // usecs to 125 ticker counts
#define msToTicks(x) (((x) *1000ul) / 125ul)  // msecs to 125 ticker counts

//timeout in usecs until a first packet is aborted
#define PACKET_TIMEOUT msToTicks(100)

// timeout in usecs until the next packet is aborted
#define PACKET_TIMEOUT_SHORT msToTicks(5)

typedef enum
{
  IDLE = 0,
  RECEIVED,
  WAIT_FOR_XMIT_READY,
  WAIT_FOR_XMIT_DONE,
} PacketState_t;

struct PacketTransfer;

struct PacketTransfer
{
  uint8_t const                portNo;
  uint8_t const                outgoingPortNo;
  struct PacketTransfer *const outgoingTransfer;
  int                          powered;
  int                          online;
  int                          first;

  PacketState_t state;
  uint8_t *     pData[2];
  int32_t       len[2];
  uint64_t      packetTimeout;
  uint64_t      packetTime;
  int           dropped;
};

typedef struct PacketTransfer PacketTransfer_t;

static PacketTransfer_t packetTransfer[2] =  // port number is referring to incoming packet !
    {
      { .first = 1, .portNo = 0, .outgoingPortNo = 1, .outgoingTransfer = &packetTransfer[1] },
      { .first = 1, .portNo = 1, .outgoingPortNo = 0, .outgoingTransfer = &packetTransfer[0] },
    };

// macros for providing a C++ - style "this" pointer
#define OP         PacketTransfer_t *const t
#define mkOP(name) PacketTransfer_t *const name

static inline void packetTransferReset(OP)
{
  t->state = IDLE;
  USB_MIDI_SuspendReceive(t->portNo, 0);  // keep receiver enabled
  USB_MIDI_ClearReceive(t->portNo);

  t->pData[0] = t->pData[1] = NULL;
  t->len[0] = t->len[1] = 0;
  t->dropped            = 0;
  t->packetTimeout      = PACKET_TIMEOUT;
}

static inline void processTransfers(OP)
{
  if (!t->outgoingTransfer->online)
    return;

  if (t->state == IDLE)
    return;

  uint64_t const now = ticker;

  switch (t->state)
  {  // main sending state machine
    default:
      break;

    case RECEIVED:  // packet was received, so start transmit process
      t->packetTimeout = (!t->dropped) ? PACKET_TIMEOUT : PACKET_TIMEOUT_SHORT;
      t->dropped       = 0;
      t->packetTime    = now;
      SMON_monitorEvent(t->portNo, PACKET_START);
      t->state = WAIT_FOR_XMIT_READY;
      // fall-through is on purpose

    case WAIT_FOR_XMIT_READY:
      if (USB_MIDI_Send(t->outgoingPortNo, t->pData[0], t->len[0]) < 0)
        break;  /// could not start transfer now, try later
      t->state = WAIT_FOR_XMIT_DONE;
      break;

    case WAIT_FOR_XMIT_DONE:
      if (USB_MIDI_BytesToSend(t->outgoingPortNo) > 0)
        break;  // still sending...
      __disable_irq();
      t->pData[0] = t->pData[1];
      t->len[0]   = t->len[1];
      if (t->pData[1])
      {
        t->pData[1] = 0;
        t->len[1]   = 0;
        t->state    = RECEIVED;
        __enable_irq();
        break;
      }
      t->pData[1] = 0;
      t->len[1]   = 0;
      t->state    = IDLE;
      __enable_irq();
      USB_MIDI_SuspendReceive(t->portNo, 0);  // re-enable receiver
      USB_MIDI_primeReceive(t->portNo);
      SMON_monitorEvent(t->portNo, PACKET_DELIVERED);
      break;
  }

  if (t->state >= WAIT_FOR_XMIT_READY)
  {
    if ((now - t->packetTime) > t->packetTimeout)  // packet could not be submitted
    {
      t->dropped = 1;
      USB_MIDI_KillTransmit(t->outgoingPortNo);
      t->state = IDLE;
      USB_MIDI_SuspendReceive(t->portNo, 0);  // re-enable receiver
      USB_MIDI_primeReceive(t->portNo);
      SMON_monitorEvent(t->portNo, PACKET_DROPPED);
      return;
    }
  }
}

static inline void checkPortStatus(OP)
{
  if (USB_GetError(t->portNo))
  {
    __disable_irq();
    USB_MIDI_DeInit(t->portNo);
    USB_MIDI_DeInit(t->outgoingPortNo);
    MIDI_Relay_Init();
    __enable_irq();
    return;
  }
  uint32_t online  = USB_MIDI_IsConfigured(t->portNo);
  int      powered = (t->portNo == 0) ? pinUSB0_VBUS() : pinUSB1_VBUS();

  if (t->first)
  {
    t->first   = 0;
    t->online  = !online;
    t->powered = !powered;
  }

  if (online != t->online)
  {
    t->online = online;
    if (online)
    {
      USB_MIDI_SuspendReceive(t->portNo, 0);
      USB_MIDI_primeReceive(t->portNo);
      SMON_monitorEvent(t->portNo, ONLINE);
    }
    else
    {
      packetTransferReset(t);
      SMON_monitorEvent(t->portNo, OFFLINE);
    }
  }

  if (powered != t->powered)
  {
    t->powered = powered;
    if (powered)
      SMON_monitorEvent(t->portNo, POWERED);
    else
      SMON_monitorEvent(t->portNo, UNPOWERED);
  }
}

void MIDI_Relay_ProcessFast(void)
{
  checkPortStatus(&packetTransfer[0]);
  checkPortStatus(&packetTransfer[1]);

  processTransfers(&packetTransfer[0]);
  processTransfers(&packetTransfer[1]);
}

// ------------------------------------------------------------

static inline void onReceive(OP, uint8_t *buff, uint32_t len)
{
  if (len == 0)
    return;  // empty packet will not cause any action and is simply ignored

  if (!t->online)
    return;  // just in case incoming port went offline and we still got an interrupt

  if (len > 512)  // we should never ever receive a packet longer than the fixed(!) 512Bytes HS bulk size max
    DisplayErrorAndHalt(E_USB_PACKET_SIZE);

  if (!t->outgoingTransfer->online)  // outgoing port is offline, mark packet as dismissed
  {
    SMON_monitorEvent(t->portNo, DROPPED_INCOMING);
    return;
  }

  if ((t->state == IDLE) || (t->pData[1] == 0))
  {
    // setup packet transfer data ...
	unsigned idx  = (t->pData[1] != 0);
    t->pData[idx] = buff;
    t->len[idx]   = len;
    if (idx == 0)
      t->state    = RECEIVED;
    // ... and block receiver until transmit finished/failed
    USB_MIDI_SuspendReceive(t->portNo, 1);
    return;
  }

  // we should never receive a packet when not IDLE
  DisplayErrorAndHalt(E_USB_UNEXPECTED_PACKET);
}

static void Receive_IRQ_Callback_0(uint8_t const port, uint8_t *buff, uint32_t len)
{
  onReceive(&packetTransfer[0], buff, len);
}

static void Receive_IRQ_Callback_1(uint8_t const port, uint8_t *buff, uint32_t len)
{
  onReceive(&packetTransfer[1], buff, len);
}

static void Receive_IRQ_DevCtlCallback(uint8_t const port, uint8_t *buff, uint32_t len)
{
  DEVCTL_processMsg(buff, len);
}

static void Receive_IRQ_FirstCallback(uint8_t const port, uint8_t *buff, uint32_t len)
{
  USB_MIDI_Config(0, Receive_IRQ_Callback_0);
  USB_MIDI_Config(1, Receive_IRQ_Callback_1);

  uint16_t cmd = DEVCTL_isDeviceControlMsg(&buff, &len);
  if (cmd == CMD_GET_AND_PROG)
  {
    USB_MIDI_DeInit(port ^ 1);
    USB_MIDI_Config(port ^ 1, NULL);
    USB_MIDI_Config(port, Receive_IRQ_DevCtlCallback);

    DEVCTL_init(port);
    DEVCTL_processMsg(buff, len);
    return;
  }

  if (cmd == CMD_LED_TEST)
  {
    SMON_monitorEvent(0, LED_TEST);
    return;
  }

  onReceive(&packetTransfer[port], buff, len);
}

void MIDI_Relay_Init(void)
{
  packetTransferReset(&packetTransfer[0]);
  packetTransferReset(&packetTransfer[1]);
  USB_MIDI_Config(0, Receive_IRQ_FirstCallback);
  USB_MIDI_Config(1, Receive_IRQ_FirstCallback);
  USB_MIDI_Init(0);
  USB_MIDI_Init(1);
}

void SendERP(void *buff, uint32_t len, void *sysexBuff)
{
  OP = &packetTransfer[1];

  if (!t->outgoingTransfer->online)
    return;
  __disable_irq();
  if ((t->state == IDLE) || ((t->pData[1] == 0) && (t->pData[0] != sysexBuff)))
  {
    uint16_t size = MIDI_encodeRawSysex((uint8_t *) buff, len, sysexBuff, 1);
    unsigned idx  = (t->pData[1] != 0);
    t->pData[idx] = sysexBuff;
    t->len[idx]   = size;
    if (idx == 0)
      t->state      = RECEIVED;
    __enable_irq();
    return;
  }

  SMON_monitorEvent(t->portNo, DROPPED_INCOMING);
  LED_DBG2 = 1;
  __enable_irq();
  return;
}
