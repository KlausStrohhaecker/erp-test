/******************************************************************************/
/** @file		nl_usb_core.c
    @date		2014-12-11
    @brief    	Core functionality for the USB
    @example
    @ingroup	nl_drv_modules
    @author		Nemanja Nikodijevic 2014-12-11
*******************************************************************************/
#include "cmsis/LPC43xx.h"
#include "cmsis/core_cm4.h"
#include "cmsis/lpc_types.h"
#include "usb/nl_usbd.h"
#include "usb/nl_usb_core.h"
#include "sys/nl_stdlib.h"
#include "io/pins.h"

//#define CHECK_SIZES_AT_COMPILE_TIME
#ifdef CHECK_SIZES_AT_COMPILE_TIME
#define checkSize(x) char(*x##_check)[sizeof(x)] = 1
#else
#define checkSize(x)
#endif

static uint32_t EPAdr(uint32_t const EPNum);
static void     SetAddress(uint8_t const port, uint32_t const adr);
static void     Reset(uint8_t const port);
static uint32_t ReadSetupPkt(uint8_t const port, uint32_t const EPNum, uint32_t *const pData);
static void     SetupStage(uint8_t const port);
static void     DataInStage(uint8_t const port);
static void     DataOutStage(uint8_t const port);
static void     StatusInStage(uint8_t const port);
static void     StatusOutStage(uint8_t const port);
static void     WakeUpCfg(uint32_t const cfg);
static void     Configure(uint32_t const cfg);
static void     DirCtrlEP(uint32_t const dir);
static uint32_t USB_SetTestMode(uint8_t const port, uint8_t const mode);
static void     SetStallEP(uint8_t const port, uint32_t const EPNum);
static void     ClrStallEP(uint8_t const port, uint32_t const EPNum);
static uint32_t ReqSetClrFeature(uint8_t const port, uint32_t const sc);
static void     ConfigEP(uint8_t const port, USB_ENDPOINT_DESCRIPTOR const *const pEPD);
static void     EnableEP(uint8_t const port, uint32_t const EPNum);
static void     DisableEP(uint8_t const port, uint32_t const EPNum);
static void     EndPoint0(uint8_t const port, uint32_t const event);
static void     Handler(uint8_t const port);
static void     ClearDTD(uint8_t const port, uint32_t Edpt);

static DQH_T ep_QH_0[EP_NUM_MAX] __attribute__((aligned(2048)));
static DTD_T ep_TD_0[EP_NUM_MAX] __attribute__((aligned(64)));

static DQH_T ep_QH_1[EP_NUM_MAX] __attribute__((aligned(2048)));
static DTD_T ep_TD_1[EP_NUM_MAX] __attribute__((aligned(64)));

static void USB_DummyEPHandler(uint8_t const port, uint32_t const event)
{
}

typedef struct
{
  LPC_USB0_Type *       hardware;
  DQH_T *               ep_QH;
  DTD_T *               ep_TD;
  uint32_t              ep_read_len[3];
  EndpointCallback      P_EPCallback[USB_EP_NUM];
  InterfaceEventHandler Interface_Event;
  ClassRequestHandler   Class_Specific_Request;
  SOFHandler            SOF_Event;
  uint16_t              DeviceStatus;
  uint8_t               DeviceAddress;
  uint8_t               Configuration;
  uint32_t              EndPointMask;
  uint32_t              EndPointHalt;
  uint32_t              EndPointStall; /* EP must stay stalled */
  uint8_t               NumInterfaces;
  uint8_t               AltSetting[USB_IF_NUM];
  USB_EP_DATA           EP0Data;
  uint8_t               EP0Buf[USB_MAX_PACKET0];
  USB_SETUP_PACKET      SetupPacket;
  volatile uint32_t     DevStatusFS2HS;
  uint8_t const *       DeviceDescriptor;
  uint8_t const *       FSConfigDescriptor;
  uint8_t const *       HSConfigDescriptor;
  uint8_t const *       StringDescriptor;
  uint8_t const *       DeviceQualifier;
  uint8_t const *       FSOtherSpeedConfiguration;
  uint8_t const *       HSOtherSpeedConfiguration;
  uint8_t               activity;
  uint8_t               error;
  uint8_t               gotConfigDescriptorRequest;  // NOT RELIABLE, do not use !!
  uint8_t               connectionEstablished;
} usb_core_t;

static usb_core_t usb[2] = {
  {
      .hardware     = ((LPC_USB0_Type *) LPC_USB0_BASE),
      .ep_QH        = &ep_QH_0[0],
      .ep_TD        = &ep_TD_0[0],
      .P_EPCallback = { USB_DummyEPHandler, USB_DummyEPHandler, USB_DummyEPHandler },
  },
  {
      .hardware     = ((LPC_USB0_Type *) LPC_USB1_BASE),
      .ep_QH        = &ep_QH_1[0],
      .ep_TD        = &ep_TD_1[0],
      .P_EPCallback = { USB_DummyEPHandler, USB_DummyEPHandler, USB_DummyEPHandler },
  },
};

#ifdef CHECK_SIZES_AT_COMPILE_TIME
checkSize(usb);
checkSize(ep_QH_0);
checkSize(ep_TD_0);
#endif

/******************************************************************************/
/** @brief		Translates the logical endpoint address to physical
    @param[in]	EPNum	endpoint number
    @return		Physical endpoint address
*******************************************************************************/
static inline uint32_t EPAdr(uint32_t const EPNum)
{
  uint32_t val;

  val = (EPNum & 0x0F) << 1;
  if (EPNum & 0x80)
  {
    val += 1;
  }
  return (val);
}

/******************************************************************************/
/** @brief		Set the USB device address
    @param[in]	adr		Device address
*******************************************************************************/
static void inline SetAddress(uint8_t const port, uint32_t const adr)
{
  usb[port].hardware->DEVICEADDR = USBDEV_ADDR(adr);
  usb[port].hardware->DEVICEADDR |= USBDEV_ADDR_AD;
}

/******************************************************************************/
/** @brief    Function for reseting the USB MIDI driver
*******************************************************************************/
static void Reset(uint8_t const port)
{
  uint32_t i;

  usb[port].DevStatusFS2HS = FALSE;
  /* disable all EPs */
  usb[port].hardware->ENDPTCTRL0 &= ~(EPCTRL_RXE | EPCTRL_TXE);
  usb[port].hardware->ENDPTCTRL1 &= ~(EPCTRL_RXE | EPCTRL_TXE);
  usb[port].hardware->ENDPTCTRL2 &= ~(EPCTRL_RXE | EPCTRL_TXE);
  usb[port].hardware->ENDPTCTRL3 &= ~(EPCTRL_RXE | EPCTRL_TXE);

  /* Clear all pending interrupts */
  usb[port].hardware->ENDPTNAK       = 0xFFFFFFFF;
  usb[port].hardware->ENDPTNAKEN     = 0;
  usb[port].hardware->USBSTS_D       = 0xFFFFFFFF;
  usb[port].hardware->ENDPTSETUPSTAT = usb[port].hardware->ENDPTSETUPSTAT;
  usb[port].hardware->ENDPTCOMPLETE  = usb[port].hardware->ENDPTCOMPLETE;
  while (usb[port].hardware->ENDPTPRIME) /* Wait until all bits are 0 */
  {
  }
  usb[port].hardware->ENDPTFLUSH = 0xFFFFFFFF;
  while (usb[port].hardware->ENDPTFLUSH)
    ; /* Wait until all bits are 0 */

  /* Set the interrupt Threshold control interval to 0 */
  usb[port].hardware->USBCMD_D &= ~0x00FF0000;

  /* Zero out the Endpoint queue heads */
  memset((void *) usb[port].ep_QH, 0, EP_NUM_MAX * sizeof(DQH_T));
  /* Zero out the device transfer descriptors */
  memset((void *) usb[port].ep_TD, 0, EP_NUM_MAX * sizeof(DTD_T));
  memset((void *) usb[port].ep_read_len, 0, sizeof(usb[port].ep_read_len));
  /* Configure the Endpoint List Address */
  /* make sure it in on 64 byte boundary !!! */
  /* init list address */
  usb[port].hardware->ENDPOINTLISTADDR = (uint32_t) & (usb[port].ep_QH[0]);
  /* Initialize device queue heads for non ISO endpoint only */
  for (i = 0; i < EP_NUM_MAX; i++)
  {
    usb[port].ep_QH[i].next_dTD = (uint32_t) & (usb[port].ep_TD[i]);
  }
  /* Set DMA Burst Size */
  if (port == 0)
    usb[port].hardware->SBUSCFG = 0x07;  // as per user manual
  else
    usb[port].hardware->BURSTSIZE = (16 << 8) | (16 << 0);  // one 32bit word at a time
  /* Enable interrupts */
  usb[port].hardware->USBINTR_D = USBSTS_UI
      | USBSTS_UEI
      | USBSTS_SEI
      | USBSTS_PCI
      | USBSTS_URI
      | USBSTS_SRI /* Start-of-Frame */
      | USBSTS_SLI
      | USBSTS_NAKI;
  /* enable ep0 IN and ep0 OUT */
  usb[port].ep_QH[0].cap = QH_MAXP(USB_MAX_PACKET0)
      | QH_IOS
      | QH_ZLT;
  usb[port].ep_QH[1].cap = QH_MAXP(USB_MAX_PACKET0)
      | QH_IOS
      | QH_ZLT;
  /* enable EP0 */
  usb[port].hardware->ENDPTCTRL0 = EPCTRL_RXE | EPCTRL_RXR | EPCTRL_TXE | EPCTRL_TXR;
  return;
}

/******************************************************************************/
/** @brief		Read USB setup packet from an endpoint
    @param[in]	EPNum	Endpoint number and direction
    					7	Direction (0 - out; 1-in)
    					3:0	Endpoint number
    @param[in]	pData	Pointer to data buffer
    @return		Number of bytes read
*******************************************************************************/
static uint32_t ReadSetupPkt(uint8_t const port, uint32_t const EPNum, uint32_t *const pData)
{
  uint32_t setup_int, cnt = 0;
  uint32_t num = EPAdr(EPNum);

  setup_int = usb[port].hardware->ENDPTSETUPSTAT;
  /* Clear the setup interrupt */
  usb[port].hardware->ENDPTSETUPSTAT = setup_int;

  /*  Check if we have received a setup */
  if (setup_int & (1 << 0)) /* Check only for bit 0 */
  /* No setup are admitted on other endpoints than 0 */
  {
    do
    {
      /* Setup in a setup - must consider only the second setup */
      /*- Set the tripwire */
      usb[port].hardware->USBCMD_D |= USBCMD_SUTW;

      /* Transfer Set-up data to the gtmudsCore_Request buffer */
      pData[0] = usb[port].ep_QH[num].setup[0];
      pData[1] = usb[port].ep_QH[num].setup[1];
      cnt      = 8;

    } while (!(usb[port].hardware->USBCMD_D & USBCMD_SUTW));

    /* setup in a setup - Clear the tripwire */
    usb[port].hardware->USBCMD_D &= (~USBCMD_SUTW);
  }
  while ((setup_int = usb[port].hardware->ENDPTSETUPSTAT) != 0)
  {
    /* Clear the setup interrupt */
    usb[port].hardware->ENDPTSETUPSTAT = setup_int;
  }
  return cnt;
}

/******************************************************************************/
/** @brief		Init USB core
*******************************************************************************/
void USB_Core_Init(uint8_t const port)
{
  usb[port].P_EPCallback[0] = EndPoint0;
  usb[port].error           = 0;

  /* Turn on the phy */
  if (port == 0)
    LPC_CREG->CREG0 &= ~(1 << 5);
  else
    /*                USB_AIM    USB_ESEA   USB_EPD    USB_EPWR   USB_VBUS */
    LPC_SCU->SFSUSB = (0 << 0) | (1 << 1) | (0 << 2) | (1 << 4) | (1 << 5);

  /* reset the controller */
  usb[port].hardware->USBCMD_D = USBCMD_RST;
  /* wait for reset to complete */
  while (usb[port].hardware->USBCMD_D & USBCMD_RST)
    ;

  /* Program the controller to be the USB device controller */
  usb[port].hardware->USBMODE_D = USBMODE_CM_DEV
      | USBMODE_SDIS
      | USBMODE_SLOM;

  if (port == 0)
  {
    /* set OTG transceiver in proper state */
    /*                VBUS=1     MODE=DEVICE */
    usb[port].hardware->OTGSC = (1 << 1) | (1 << 3);
  }

#if USB_POLLING

  if (port == 0)
    NVIC_DisableIRQ(USB0_IRQn);
  else
    NVIC_DisableIRQ(USB1_IRQn);

#else

  if (port == 0)
    NVIC_EnableIRQ(USB0_IRQn);
  else
    NVIC_EnableIRQ(USB1_IRQn);

#endif

  Reset(port);
  SetAddress(port, 0);

  /* set IRQ threshold to "immediate" */
  usb[port].hardware->USBCMD_D &= 0xFF00FFFF;

  /* USB Connect */
  usb[port].hardware->USBCMD_D |= USBCMD_RS;
}

uint8_t USB_GetActivity(uint8_t const port)
{
  uint8_t ret        = usb[port].activity;
  usb[port].activity = 0;
  return ret;
}

uint8_t USB_GetError(uint8_t const port)
{
  uint8_t ret     = usb[port].error;
  usb[port].error = 0;
  return ret;
}

uint8_t USB_SetupComplete(uint8_t const port)
{
  return usb[port].gotConfigDescriptorRequest;
}

void USB_Core_DeInit(uint8_t const port)
{
  usb[port].gotConfigDescriptorRequest = 0;
  usb[port].connectionEstablished      = 0;

  /* Turn off the phy */
  if (port == 0)
    LPC_CREG->CREG0 |= (1 << 5);
  else
    /*                USB_AIM    USB_ESEA   USB_EPD    USB_EPWR   USB_VBUS */
    LPC_SCU->SFSUSB = (0 << 0) | (0 << 1) | (0 << 2) | (1 << 4) | (0 << 5);
}

/*****************************************************************************/
/** @brief		Force the Full Speed for the USB controller
*******************************************************************************/
void USB_Core_ForceFullSpeed(uint8_t const port)
{
  usb[port].hardware->PORTSC1_D |= (1 << 24);
}

/******************************************************************************/
/** @brief		Set the Endpoint Callback function
    @param[in]	ep		Endpoint number
    @param[in]	cb		Callback function pointer
*******************************************************************************/
void USB_Core_Endpoint_Callback_Set(uint8_t const port, uint8_t const ep, EndpointCallback const cb)
{
  if ((ep >= EP_NUM_MAX) || (ep == 0))
    return;
  usb[port].P_EPCallback[ep] = cb;
}

/******************************************************************************/
/** @brief		Checks whether the USB is connected and configured, up and running
    @return		1 - Success ; 0 - Failure
*******************************************************************************/
uint8_t USB_Core_IsConfigured(uint8_t const port)
{
  return (usb[port].hardware->PORTSC1_D & (1 << 0))
      && usb[port].Configuration
      // && usb[port].gotConfigDescriptorRequest  // unreliable
      && usb[port].connectionEstablished;
}

uint8_t USB_Core_ConfigStatus(uint8_t const port)
{
  return ((usb[port].hardware->PORTSC1_D & 1) << 0)
      | ((usb[port].Configuration != 0) << 1)
      // | ((usb[port].gotConfigDescriptorRequest != 0) << 2)  // unreliable
      | ((usb[port].connectionEstablished != 0) << 2);
}

/******************************************************************************/
/** @brief		Checks if the endpoint descriptor is writable at the moment
	@param[in]	ep	Endpoint number
    @return		1 - Success ; 0 - Failure
*******************************************************************************/
uint8_t USB_Core_ReadyToWrite(uint8_t const port, uint8_t const epnum)
{
  uint32_t ep = EPAdr(epnum);
  if ((usb[port].ep_TD[ep].next_dTD & 1) && ((usb[port].ep_TD[ep].total_bytes & 1 << 7) == 0))
    return 1;
  else
    return 0;
}

/******************************************************************************/
/** @brief		Reset USB core
*******************************************************************************/
void USB_ResetCore(uint8_t const port)
{

  usb[port].DeviceStatus  = 1;
  usb[port].DeviceAddress = 0;
  usb[port].Configuration = 0;
  usb[port].EndPointMask  = 0x00010001;
  usb[port].EndPointHalt  = 0x00000000;
  usb[port].EndPointStall = 0x00000000;
}

/******************************************************************************/
/** @brief		USB Request - Setup stage
*******************************************************************************/
static inline void SetupStage(uint8_t const port)
{
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Waddress-of-packed-member"
#pragma GCC diagnostic ignored "-Wattributes"
  ReadSetupPkt(port, 0x00, (uint32_t *) &usb[port].SetupPacket);
#pragma GCC diagnostic push
}

/******************************************************************************/
/** @brief		USB Request - Data in stage
*******************************************************************************/
static inline void DataInStage(uint8_t const port)
{
  uint32_t cnt;

  if (usb[port].EP0Data.Count > USB_MAX_PACKET0)
  {
    cnt = USB_MAX_PACKET0;
  }
  else
  {
    cnt = usb[port].EP0Data.Count;
  }
  cnt = USB_WriteEP(port, 0x80, usb[port].EP0Data.pData, cnt);
  usb[port].EP0Data.pData += cnt;
  usb[port].EP0Data.Count -= cnt;
}

/******************************************************************************/
/** @brief		USB Request - Data out stage
*******************************************************************************/
static inline void DataOutStage(uint8_t const port)
{
  uint32_t cnt;

  cnt = USB_ReadEP(port, 0x00);
  usb[port].EP0Data.pData += cnt;
  usb[port].EP0Data.Count -= cnt;
}

/******************************************************************************/
/** @brief		USB Request - Status in stage
*******************************************************************************/
static inline void StatusInStage(uint8_t const port)
{
  USB_WriteEP(port, 0x80, NULL, 0);
}

/******************************************************************************/
/** @brief		USB Request - Status out stage
*******************************************************************************/
static void inline StatusOutStage(uint8_t const port)
{
  USB_ReadEP(port, 0x00);
}

/******************************************************************************/
/** @brief		Get Status USB request
    @return		TRUE - success; FALSE - error
*******************************************************************************/
uint32_t USB_ReqGetStatus(uint8_t const port)
{
  uint32_t n, m;

  switch (usb[port].SetupPacket.bmRequestType.BM.Recipient)
  {
    case REQUEST_TO_DEVICE:
      usb[port].EP0Data.pData = (uint8_t *) &usb[port].DeviceStatus;
      break;
    case REQUEST_TO_INTERFACE:
      if ((usb[port].Configuration != 0) && (usb[port].SetupPacket.wIndex.WB.L < usb[port].NumInterfaces))
      {
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wstrict-aliasing"
#pragma GCC diagnostic ignored "-Wattributes"
        *((__attribute__((__packed__)) uint16_t *) usb[port].EP0Buf) = 0;
#pragma GCC diagnostic pop
        usb[port].EP0Data.pData = usb[port].EP0Buf;
      }
      else
      {
        return (FALSE);
      }
      break;
    case REQUEST_TO_ENDPOINT:
      n = usb[port].SetupPacket.wIndex.WB.L & 0x8F;
      m = (n & 0x80) ? ((1 << 16) << (n & 0x0F)) : (1 << n);
      if (((usb[port].Configuration != 0) || ((n & 0x0F) == 0)) && (usb[port].EndPointMask & m))
      {
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wstrict-aliasing"
        *((__attribute__((__packed__)) uint16_t *) usb[port].EP0Buf) = (usb[port].EndPointHalt & m) ? 1 : 0;
#pragma GCC diagnostic pop
        usb[port].EP0Data.pData = usb[port].EP0Buf;
      }
      else
      {
        return (FALSE);
      }
      break;
    default:
      return (FALSE);
  }
  return (TRUE);
}

static inline void WakeUpCfg(uint32_t const cfg)
{
  /* Not needed */
}

static inline void Configure(uint32_t const cfg)
{
  /* Not needed */
}

static inline void DirCtrlEP(uint32_t const dir)
{
  /* Not needed */
}

/******************************************************************************/
/** @brief		USB set test mode Function
    @param[in]	mode	Test mode
    @return		TRUE if supported, else FALSE
*******************************************************************************/
static inline uint32_t USB_SetTestMode(uint8_t const port, uint8_t const mode)
{
  uint32_t portsc;

  if ((mode > 0) && (mode < 8))
  {
    portsc = usb[port].hardware->PORTSC1_D & ~(0xF << 16);

    usb[port].hardware->PORTSC1_D = portsc | (mode << 16);
    return TRUE;
  }
  return (FALSE);
}

/******************************************************************************/
/** @brief		Set Stall for USB Endpoint
    @param[in]	EPNum	Endpoint number and direction
    					7	Direction (0 - out; 1-in)
    					3:0	Endpoint number
*******************************************************************************/
static inline void SetStallEP(uint8_t const port, uint32_t const EPNum)
{
  uint32_t lep;

  lep = EPNum & 0x0F;
  if (EPNum & 0x80)
  {
    ((uint32_t *) &(usb[port].hardware->ENDPTCTRL0))[lep] |= EPCTRL_TXS;
  }
  else
  {
    ((uint32_t *) &(usb[port].hardware->ENDPTCTRL0))[lep] |= EPCTRL_RXS;
  }
}

/******************************************************************************/
/** @brief		Clear Stall for USB Endpoint
    @param[in]	EPNum	Endpoint number and direction
    					7	Direction (0 - out; 1-in)
    					3:0	Endpoint number
*******************************************************************************/
static inline void ClrStallEP(uint8_t const port, uint32_t const EPNum)
{
  uint32_t lep;

  lep = EPNum & 0x0F;
  if (EPNum & 0x80)
  {
    ((uint32_t *) &(usb[port].hardware->ENDPTCTRL0))[lep] &= ~EPCTRL_TXS;
    /* reset data toggle */
    ((uint32_t *) &(usb[port].hardware->ENDPTCTRL0))[lep] |= EPCTRL_TXR;
  }
  else
  {
    ((uint32_t *) &(usb[port].hardware->ENDPTCTRL0))[lep] &= ~EPCTRL_RXS;
    /* reset data toggle */
    ((uint32_t *) &(usb[port].hardware->ENDPTCTRL0))[lep] |= EPCTRL_RXR;
  }
}

/******************************************************************************/
/** @brief		Set/Clear Feature USB request
    @param[in]	sc	0 - clear; 1 - set
    @return		TRUE - success; FALSE - error
*******************************************************************************/
static uint32_t ReqSetClrFeature(uint8_t const port, uint32_t const sc)
{
  uint32_t n, m;

  switch (usb[port].SetupPacket.bmRequestType.BM.Recipient)
  {
    case REQUEST_TO_DEVICE:
      if (usb[port].SetupPacket.wValue.W == USB_FEATURE_REMOTE_WAKEUP)
      {
        if (sc)
        {
          WakeUpCfg(TRUE);
          usb[port].DeviceStatus |= USB_GETSTATUS_REMOTE_WAKEUP;
        }
        else
        {
          WakeUpCfg(FALSE);
          usb[port].DeviceStatus &= ~USB_GETSTATUS_REMOTE_WAKEUP;
        }
      }
      else if (usb[port].SetupPacket.wValue.W == USB_FEATURE_TEST_MODE)
      {
        return USB_SetTestMode(port, usb[port].SetupPacket.wIndex.WB.H);
      }
      else
      {
        return (FALSE);
      }
      break;
    case REQUEST_TO_INTERFACE:
      return (FALSE);
    case REQUEST_TO_ENDPOINT:
      n = usb[port].SetupPacket.wIndex.WB.L & 0x8F;
      m = (n & 0x80) ? ((1 << 16) << (n & 0x0F)) : (1 << n);
      if ((usb[port].Configuration != 0) && ((n & 0x0F) != 0) && (usb[port].EndPointMask & m))
      {
        if (usb[port].SetupPacket.wValue.W == USB_FEATURE_ENDPOINT_STALL)
        {
          if (sc)
          {
            SetStallEP(port, n);
            usb[port].EndPointHalt |= m;
          }
          else
          {
            if ((usb[port].EndPointStall & m) != 0)
            {
              return (TRUE);
            }
            ClrStallEP(port, n);
            usb[port].EndPointHalt &= ~m;
          }
        }
        else
        {
          return (FALSE);
        }
      }
      else
      {
        return (FALSE);
      }
      break;
    default:
      return (FALSE);
  }
  return (TRUE);
}

/******************************************************************************/
/** @brief		Set Address USB request
    @return		TRUE - success; FALSE - error
*******************************************************************************/
uint32_t USB_ReqSetAddress(uint8_t const port)
{

  switch (usb[port].SetupPacket.bmRequestType.BM.Recipient)
  {
    case REQUEST_TO_DEVICE:
      usb[port].DeviceAddress = 0x80 | usb[port].SetupPacket.wValue.WB.L;
      break;
    default:
      return (FALSE);
  }
  return (TRUE);
}

/******************************************************************************/
/** @brief		Get Descriptor USB request
    @return		TRUE - success; FALSE - error
*******************************************************************************/
uint32_t USB_ReqGetDescriptor(uint8_t const port)
{
  uint8_t *pD;
  uint32_t len, n;

  switch (usb[port].SetupPacket.bmRequestType.BM.Recipient)
  {
    case REQUEST_TO_DEVICE:
      switch (usb[port].SetupPacket.wValue.WB.H)
      {
        case USB_DEVICE_DESCRIPTOR_TYPE:
          usb[port].EP0Data.pData = (uint8_t *) usb[port].DeviceDescriptor;
          len                     = USB_DEVICE_DESC_SIZE;
          break;
        case USB_CONFIGURATION_DESCRIPTOR_TYPE:
          if (usb[port].DevStatusFS2HS == FALSE)
          {
            pD = (uint8_t *) usb[port].FSConfigDescriptor;
          }
          else
          {
            pD = (uint8_t *) usb[port].HSConfigDescriptor;
          }
          for (n = 0; n != usb[port].SetupPacket.wValue.WB.L; n++)
          {
            if (((USB_CONFIGURATION_DESCRIPTOR *) pD)->bLength != 0)
            {
              pD += ((USB_CONFIGURATION_DESCRIPTOR *) pD)->wTotalLength;
            }
          }
          if (((USB_CONFIGURATION_DESCRIPTOR *) pD)->bLength == 0)
          {
            return (FALSE);
          }
          usb[port].EP0Data.pData              = pD;
          len                                  = ((USB_CONFIGURATION_DESCRIPTOR *) pD)->wTotalLength;
          usb[port].gotConfigDescriptorRequest = 1;
          break;
        case USB_STRING_DESCRIPTOR_TYPE:
          pD = (uint8_t *) usb[port].StringDescriptor;
          for (n = 0; n != usb[port].SetupPacket.wValue.WB.L; n++)
          {
            if (((USB_STRING_DESCRIPTOR *) pD)->bLength != 0)
            {
              pD += ((USB_STRING_DESCRIPTOR *) pD)->bLength;
            }
          }
          if (((USB_STRING_DESCRIPTOR *) pD)->bLength == 0)
          {
            return (FALSE);
          }
          usb[port].EP0Data.pData = pD;
          len                     = ((USB_STRING_DESCRIPTOR *) pD)->bLength;
          break;
        case USB_DEVICE_QUALIFIER_DESCRIPTOR_TYPE:
          /* USB Chapter 9. page 9.6.2 */
          if (usb[port].DevStatusFS2HS == FALSE)
          {
            return (FALSE);
          }
          else
          {
            usb[port].EP0Data.pData = (uint8_t *) usb[port].DeviceQualifier;
            len                     = USB_DEVICE_QUALI_SIZE;
          }
          break;
        case USB_OTHER_SPEED_CONFIG_DESCRIPTOR_TYPE:
          if (usb[port].DevStatusFS2HS == TRUE)
          {
            pD = (uint8_t *) usb[port].FSOtherSpeedConfiguration;
          }
          else
          {
            pD = (uint8_t *) usb[port].HSOtherSpeedConfiguration;
          }

          for (n = 0; n != usb[port].SetupPacket.wValue.WB.L; n++)
          {
            if (((USB_OTHER_SPEED_CONFIGURATION *) pD)->bLength != 0)
            {
              pD += ((USB_OTHER_SPEED_CONFIGURATION *) pD)->wTotalLength;
            }
          }
          if (((USB_OTHER_SPEED_CONFIGURATION *) pD)->bLength == 0)
          {
            return (FALSE);
          }
          usb[port].EP0Data.pData = pD;
          len                     = ((USB_OTHER_SPEED_CONFIGURATION *) pD)->wTotalLength;
          break;
        default:
          return (FALSE);
      }
      break;
    case REQUEST_TO_INTERFACE:
      switch (usb[port].SetupPacket.wValue.WB.H)
      {
        default:
          return (FALSE);
      }
      break;
    default:
      return (FALSE);
  }

  if (usb[port].EP0Data.Count > len)
  {
    usb[port].EP0Data.Count = len;
  }

  return (TRUE);
}

/******************************************************************************/
/** @brief		Configure the USB endpoint according to the descriptor
    @param[in]	pEPD	Pointer to the endpoint descriptor
*******************************************************************************/
static void ConfigEP(uint8_t const port, USB_ENDPOINT_DESCRIPTOR const *const pEPD)
{
  uint32_t num, lep;
  uint32_t ep_cfg;
  uint8_t  bmAttributes;

  lep = pEPD->bEndpointAddress & 0x7F;
  num = EPAdr(pEPD->bEndpointAddress);

  ep_cfg = ((uint32_t *) &(usb[port].hardware->ENDPTCTRL0))[lep];
  /* mask the attributes we are not-interested in */
  bmAttributes = pEPD->bmAttributes & USB_ENDPOINT_TYPE_MASK;
  /* set EP type */
  if (bmAttributes != USB_ENDPOINT_TYPE_ISOCHRONOUS)
  {
    /* init EP capabilities */
    usb[port].ep_QH[num].cap = QH_MAXP(pEPD->wMaxPacketSize)
        | QH_IOS | QH_ZLT;
    /* The next DTD pointer is INVALID */
    usb[port].ep_TD[num].next_dTD = 0x01;
  }
  else
  {
    /* init EP capabilities */
    usb[port].ep_QH[num].cap = QH_MAXP(pEPD->wMaxPacketSize) | QH_ZLT | (1 << 30);
    /* The next DTD pointer is INVALID */
    usb[port].ep_QH[num].next_dTD = usb[port].ep_TD[num].next_dTD = 0x01;
  }
  /* setup EP control register */
  if (pEPD->bEndpointAddress & 0x80)
  {
    ep_cfg &= ~0xFFFF0000;
    ep_cfg |= EPCTRL_TX_TYPE(bmAttributes)
        | EPCTRL_TXR;
  }
  else
  {
    ep_cfg &= ~0xFFFF;
    ep_cfg |= EPCTRL_RX_TYPE(bmAttributes)
        | EPCTRL_RXR;
  }
  ((uint32_t *) &(usb[port].hardware->ENDPTCTRL0))[lep] = ep_cfg;
  return;
}

/******************************************************************************/
/** @brief		Enable the USB endpoint
    @param[in]	EPNum	Endpoint number and direction
    					7	Direction (0 - out; 1-in)
    					3:0	Endpoint number
*******************************************************************************/
static void EnableEP(uint8_t const port, uint32_t const EPNum)
{
  uint32_t lep, bitpos;

  lep = EPNum & 0x0F;

  if (EPNum & 0x80)
  {
    ((uint32_t *) &(usb[port].hardware->ENDPTCTRL0))[lep] |= EPCTRL_TXE;
#if USB_POLLING
    /* enable NAK interrupt for IN transfers - useful in polling mode only*/
    bitpos = USB_EP_BITPOS(EPNum);
    usb[port].hardware->ENDPTNAKEN |= (1 << bitpos);
#endif
  }
  else
  {
    ((uint32_t *) &(usb[port].hardware->ENDPTCTRL0))[lep] |= EPCTRL_RXE;
    /* enable NAK interrupt */
    bitpos = USB_EP_BITPOS(EPNum);
    usb[port].hardware->ENDPTNAKEN |= (1 << bitpos);
  }
}

/******************************************************************************/
/** @brief		Disable the USB endpoint
    @param[in]	EPNum	Endpoint number and direction
    					7	Direction (0 - out; 1-in)
    					3:0	Endpoint number
*******************************************************************************/
static void DisableEP(uint8_t const port, uint32_t const EPNum)
{
  uint32_t lep, bitpos;

  lep = EPNum & 0x0F;
  if (EPNum & 0x80)
  {
#if USB_POLLING
    /* disable NAK interrupt */
    bitpos = USB_EP_BITPOS(EPNum);
    usb[port].hardware->ENDPTNAKEN |= ~(1 << bitpos);
#endif
    ((uint32_t *) &(usb[port].hardware->ENDPTCTRL0))[lep] &= ~EPCTRL_TXE;
  }
  else
  {
    /* disable NAK interrupt */
    bitpos = USB_EP_BITPOS(EPNum);
    usb[port].hardware->ENDPTNAKEN &= ~(1 << bitpos);
    ((uint32_t *) &(usb[port].hardware->ENDPTCTRL0))[lep] &= ~EPCTRL_RXE;
  }
}

/******************************************************************************/
/** @brief		Reset the USB endpoint
    @param[in]	EPNum	Endpoint number and direction
    					7	Direction (0 - out; 1-in)
    					3:0	Endpoint number
*******************************************************************************/
void USB_ResetEP(uint8_t const port, uint32_t const EPNum)
{
  uint32_t bit_pos = USB_EP_BITPOS(EPNum);
  uint32_t lep     = EPNum & 0x0F;
  uint32_t ep      = EPAdr(EPNum);

  /* flush EP buffers */
  do
  {
    usb[port].hardware->ENDPTFLUSH = (1 << bit_pos);
    while (usb[port].hardware->ENDPTFLUSH & (1 << bit_pos))
      asm volatile("nop");
  } while ((usb[port].hardware->ENDPTSTAT & (1 << bit_pos)) != 0);

  /* reset data toggles */
  if (EPNum & 0x80)
  {
    ((uint32_t *) &(usb[port].hardware->ENDPTCTRL0))[lep] |= EPCTRL_TXR;
  }
  else
  {
    ((uint32_t *) &(usb[port].hardware->ENDPTCTRL0))[lep] |= EPCTRL_RXR;
  }

  /* clear any open transfer descriptors */
  // Needed to clear stale descriptors for this which are still scanned for pending transfers
  // Only non-control EPs (data endpoints) will be cleared
  if (ep >= 2)
    ClearDTD(port, ep);
}

/******************************************************************************/
/** @brief		Get Configuration USB request
    @return		TRUE - success; FALSE - error
*******************************************************************************/
uint32_t USB_ReqGetConfiguration(uint8_t const port)
{

  switch (usb[port].SetupPacket.bmRequestType.BM.Recipient)
  {
    case REQUEST_TO_DEVICE:
      usb[port].EP0Data.pData = &usb[port].Configuration;
      break;
    default:
      return (FALSE);
  }
  return (TRUE);
}

/******************************************************************************/
/** @brief		Set Configuration USB request
    @return		TRUE - success; FALSE - error
*******************************************************************************/
uint32_t USB_ReqSetConfiguration(uint8_t const port)
{
  USB_COMMON_DESCRIPTOR *pD;
  uint32_t               alt = 0;
  uint32_t               n, m;
  uint32_t               new_addr;
  switch (usb[port].SetupPacket.bmRequestType.BM.Recipient)
  {
    case REQUEST_TO_DEVICE:

      if (usb[port].SetupPacket.wValue.WB.L)
      {
        if (usb[port].DevStatusFS2HS == FALSE)
        {
          pD = (USB_COMMON_DESCRIPTOR *) usb[port].FSConfigDescriptor;
        }
        else
        {
          pD = (USB_COMMON_DESCRIPTOR *) usb[port].HSConfigDescriptor;
        }
        while (pD->bLength)
        {
          switch (pD->bDescriptorType)
          {
            case USB_CONFIGURATION_DESCRIPTOR_TYPE:
              if (((USB_CONFIGURATION_DESCRIPTOR *) pD)->bConfigurationValue == usb[port].SetupPacket.wValue.WB.L)
              {
                usb[port].Configuration = usb[port].SetupPacket.wValue.WB.L;
                usb[port].NumInterfaces = ((USB_CONFIGURATION_DESCRIPTOR *) pD)->bNumInterfaces;
                for (n = 0; n < USB_IF_NUM; n++)
                {
                  usb[port].AltSetting[n] = 0;
                }
                for (n = 1; n < USB_EP_NUM; n++)
                {
                  if (usb[port].EndPointMask & (1 << n))
                  {
                    DisableEP(port, n);
                  }
                  if (usb[port].EndPointMask & ((1 << 16) << n))
                  {
                    DisableEP(port, n | 0x80);
                  }
                }
                usb[port].EndPointMask  = 0x00010001;
                usb[port].EndPointHalt  = 0x00000000;
                usb[port].EndPointStall = 0x00000000;
                Configure(TRUE);
                if (((USB_CONFIGURATION_DESCRIPTOR *) pD)->bmAttributes & USB_CONFIG_POWERED_MASK)
                {
                  usb[port].DeviceStatus |= USB_GETSTATUS_SELF_POWERED;
                }
                else
                {
                  usb[port].DeviceStatus &= ~USB_GETSTATUS_SELF_POWERED;
                }
              }
              else
              {
                new_addr = (uint32_t) pD + ((USB_CONFIGURATION_DESCRIPTOR *) pD)->wTotalLength;
                pD       = (USB_COMMON_DESCRIPTOR *) new_addr;
                continue;
              }
              break;
            case USB_INTERFACE_DESCRIPTOR_TYPE:
              alt = ((USB_INTERFACE_DESCRIPTOR *) pD)->bAlternateSetting;
              break;
            case USB_ENDPOINT_DESCRIPTOR_TYPE:
              if (alt == 0)
              {
                n = ((USB_ENDPOINT_DESCRIPTOR *) pD)->bEndpointAddress & 0x8F;
                m = (n & 0x80) ? ((1 << 16) << (n & 0x0F)) : (1 << n);
                usb[port].EndPointMask |= m;
                ConfigEP(port, (USB_ENDPOINT_DESCRIPTOR *) pD);
                EnableEP(port, n);
                USB_ResetEP(port, n);
              }
              break;
          }
          new_addr = (uint32_t) pD + pD->bLength;
          pD       = (USB_COMMON_DESCRIPTOR *) new_addr;
        }
      }
      else
      {
        usb[port].Configuration = 0;
        for (n = 1; n < USB_EP_NUM; n++)
        {
          if (usb[port].EndPointMask & (1 << n))
          {
            DisableEP(port, n);
          }
          if (usb[port].EndPointMask & ((1 << 16) << n))
          {
            DisableEP(port, n | 0x80);
          }
        }
        usb[port].EndPointMask  = 0x00010001;
        usb[port].EndPointHalt  = 0x00000000;
        usb[port].EndPointStall = 0x00000000;
        Configure(FALSE);
      }

      if (usb[port].Configuration != usb[port].SetupPacket.wValue.WB.L)
      {
        return (FALSE);
      }
      break;
    default:
      return (FALSE);
  }
  return (TRUE);
}

/******************************************************************************/
/** @brief		Get Interface USB request
    @return		TRUE - success; FALSE - error
*******************************************************************************/
uint32_t USB_ReqGetInterface(uint8_t const port)
{

  switch (usb[port].SetupPacket.bmRequestType.BM.Recipient)
  {
    case REQUEST_TO_INTERFACE:
      if ((usb[port].Configuration != 0) && (usb[port].SetupPacket.wIndex.WB.L < usb[port].NumInterfaces))
      {
        usb[port].EP0Data.pData = usb[port].AltSetting + usb[port].SetupPacket.wIndex.WB.L;
      }
      else
      {
        return (FALSE);
      }
      break;
    default:
      return (FALSE);
  }
  return (TRUE);
}

/******************************************************************************/
/** @brief		Set Interface USB request
    @return		TRUE - success; FALSE - error
*******************************************************************************/
uint32_t USB_ReqSetInterface(uint8_t const port)
{
  USB_COMMON_DESCRIPTOR *pD;
  uint32_t               ifn = 0, alt = 0, old = 0, msk = 0;
  uint32_t               n, m;
  uint32_t               set, new_addr;

  switch (usb[port].SetupPacket.bmRequestType.BM.Recipient)
  {
    case REQUEST_TO_INTERFACE:
      if (usb[port].Configuration == 0)
        return (FALSE);
      set = FALSE;
      if (usb[port].DevStatusFS2HS == FALSE)
      {
        pD = (USB_COMMON_DESCRIPTOR *) usb[port].FSConfigDescriptor;
      }
      else
      {
        pD = (USB_COMMON_DESCRIPTOR *) usb[port].HSConfigDescriptor;
      }
      while (pD->bLength)
      {
        switch (pD->bDescriptorType)
        {
          case USB_CONFIGURATION_DESCRIPTOR_TYPE:
            if (((USB_CONFIGURATION_DESCRIPTOR *) pD)->bConfigurationValue != usb[port].Configuration)
            {
              new_addr = (uint32_t) pD + ((USB_CONFIGURATION_DESCRIPTOR *) pD)->wTotalLength;
              pD       = (USB_COMMON_DESCRIPTOR *) new_addr;
              continue;
            }
            break;
          case USB_INTERFACE_DESCRIPTOR_TYPE:
            ifn = ((USB_INTERFACE_DESCRIPTOR *) pD)->bInterfaceNumber;
            alt = ((USB_INTERFACE_DESCRIPTOR *) pD)->bAlternateSetting;
            msk = 0;
            if ((ifn == usb[port].SetupPacket.wIndex.WB.L) && (alt == usb[port].SetupPacket.wValue.WB.L))
            {
              set                       = TRUE;
              old                       = usb[port].AltSetting[ifn];
              usb[port].AltSetting[ifn] = (uint8_t) alt;
            }
            break;
          case USB_ENDPOINT_DESCRIPTOR_TYPE:
            if (ifn == usb[port].SetupPacket.wIndex.WB.L)
            {
              n = ((USB_ENDPOINT_DESCRIPTOR *) pD)->bEndpointAddress & 0x8F;
              m = (n & 0x80) ? ((1 << 16) << (n & 0x0F)) : (1 << n);
              if (alt == usb[port].SetupPacket.wValue.WB.L)
              {
                usb[port].EndPointMask |= m;
                usb[port].EndPointHalt &= ~m;
                ConfigEP(port, (USB_ENDPOINT_DESCRIPTOR *) pD);
                EnableEP(port, n);
                USB_ResetEP(port, n);
                msk |= m;
              }
              else if ((alt == old) && ((msk & m) == 0))
              {
                usb[port].EndPointMask &= ~m;
                usb[port].EndPointHalt &= ~m;
                DisableEP(port, n);
              }
            }
            break;
        }
        new_addr = (uint32_t) pD + pD->bLength;
        pD       = (USB_COMMON_DESCRIPTOR *) new_addr;
      }
      break;
    default:
      return (FALSE);
  }

  return (set);
}

/******************************************************************************/
/** @brief		Endpoint 0 Callback
    @param[in]	event	Event that triggered the interrupt
*******************************************************************************/
static void EndPoint0(uint8_t const port, uint32_t const event)
{
  switch (event)
  {
    case USB_EVT_SETUP:
      SetupStage(port);
      DirCtrlEP(usb[port].SetupPacket.bmRequestType.BM.Dir);
      usb[port].EP0Data.Count = usb[port].SetupPacket.wLength; /* Number of bytes to transfer */
      switch (usb[port].SetupPacket.bmRequestType.BM.Type)
      {

        case REQUEST_STANDARD:
          switch (usb[port].SetupPacket.bRequest)
          {
            case USB_REQUEST_GET_STATUS:
              if (!USB_ReqGetStatus(port))
              {
                goto stall_i;
              }
              DataInStage(port);
              break;

            case USB_REQUEST_CLEAR_FEATURE:
              if (ReqSetClrFeature(port, 0))
              {
                goto stall_i;
              }
              StatusInStage(port);
#if USB_FEATURE_EVENT
              USB_Feature_Event(port);
#endif
              break;

            case USB_REQUEST_SET_FEATURE:
              if (!ReqSetClrFeature(port, 1))
              {
                goto stall_i;
              }
              StatusInStage(port);
#if USB_FEATURE_EVENT
              USB_Feature_Event(port);
#endif
              break;

            case USB_REQUEST_SET_ADDRESS:
              if (!USB_ReqSetAddress(port))
              {
                goto stall_i;
              }
              StatusInStage(port);
              break;

            case USB_REQUEST_GET_DESCRIPTOR:
              if (!USB_ReqGetDescriptor(port))
              {
                goto stall_i;
              }
              DataInStage(port);
              break;

            case USB_REQUEST_SET_DESCRIPTOR:
              /*stall_o:*/ SetStallEP(port, 0x00); /* not supported */
              usb[port].EP0Data.Count = 0;
              break;

            case USB_REQUEST_GET_CONFIGURATION:
              if (USB_ReqGetConfiguration(port))
              {
                goto stall_i;
              }
              DataInStage(port);
              break;

            case USB_REQUEST_SET_CONFIGURATION:
              if (!USB_ReqSetConfiguration(port))
              {
                goto stall_i;
              }
              StatusInStage(port);
#if USB_CONFIGURE_EVENT
              USB_Configure_Event(port);
#endif
              break;

            case USB_REQUEST_GET_INTERFACE:
              if (!USB_ReqGetInterface(port))
              {
                goto stall_i;
              }
              DataInStage(port);
              break;

            case USB_REQUEST_SET_INTERFACE:
              if (!USB_ReqSetInterface(port))
              {
                goto stall_i;
              }
              StatusInStage(port);
              if (usb[port].Interface_Event)
                usb[port].Interface_Event(port, &usb[port].SetupPacket);
              break;

            default:
              goto stall_i;
          }
          break; /* end case REQUEST_STANDARD */

        case REQUEST_CLASS:
          if (usb[port].Class_Specific_Request)
          {
            if (usb[port].Class_Specific_Request(port, &usb[port].SetupPacket, &usb[port].EP0Data, event))
              goto stall_i;
          }
          break; /* end case REQUEST_CLASS */
        default:
        stall_i:
          SetStallEP(port, 0x80);
          usb[port].EP0Data.Count = 0;
          break;
      }
      break; /* end case USB_EVT_SETUP */

    case USB_EVT_OUT_NAK:
      if (usb[port].SetupPacket.bmRequestType.BM.Dir == 0)
      {
        USB_ReadReqEP(port, 0x00, usb[port].EP0Data.pData, usb[port].EP0Data.Count);
      }
      else
      {
        /* might be zero length pkt */
        USB_ReadReqEP(port, 0x00, usb[port].EP0Data.pData, 0);
      }
      break;
    case USB_EVT_OUT:
      if (usb[port].SetupPacket.bmRequestType.BM.Dir == REQUEST_HOST_TO_DEVICE)
      {
        if (usb[port].EP0Data.Count)
        {                     /* still data to receive ? */
          DataOutStage(port); /* receive data */
          if (usb[port].EP0Data.Count == 0)
          { /* data complete ? */
            switch (usb[port].SetupPacket.bmRequestType.BM.Type)
            {

              case REQUEST_STANDARD:
                goto stall_i;
                break; /* not supported */
              case REQUEST_CLASS:
                if (usb[port].Class_Specific_Request)
                {
                  if (usb[port].Class_Specific_Request(port, &usb[port].SetupPacket, &usb[port].EP0Data, event))
                    goto stall_i;
                }
                break;
              default:
                goto stall_i;
            }
          }
        }
      }
      else
      {
        StatusOutStage(port); /* receive Acknowledge */
      }
      break; /* end case USB_EVT_OUT */

    case USB_EVT_IN:
      if (usb[port].SetupPacket.bmRequestType.BM.Dir == REQUEST_DEVICE_TO_HOST)
      {
        DataInStage(port); /* send data */
      }
      else
      {
        if (usb[port].DeviceAddress & 0x80)
        {
          usb[port].DeviceAddress &= 0x7F;
          SetAddress(port, usb[port].DeviceAddress);
        }
      }
      break; /* end case USB_EVT_IN */

    case USB_EVT_OUT_STALL:
      ClrStallEP(port, 0x00);
      break;

    case USB_EVT_IN_STALL:
      ClrStallEP(port, 0x80);
      break;
  }
}

static inline void SetError(uint8_t const port)
{
  usb[port].error = 1;
}

#define STATUS_BITS (0xC0)
#define CLEAR_MASK  (0x7FFF00C0)

static inline void Handler(uint8_t const port)
{
  uint32_t disr, val, n;

  do  // repeat checking IRQ sources as several might have piled up before AND during the execution of the handler
  {
    disr                         = usb[port].hardware->USBSTS_D;  // Device Interrupt Status
    usb[port].hardware->USBSTS_D = disr;                          // writing it clear interrupt flags
    if (disr == 0)                                                // no more pending IRQs ?
      return;

    // handling sorted by priority

    /* handle setup status interrupts */
    val = usb[port].hardware->ENDPTSETUPSTAT;
    /* Only EP0 will have setup packets so call EP0 handler */
    if (val)
    {
      usb[port].activity = 1;
      /* Clear the endpoint complete CTRL OUT & IN when */
      /* a Setup is received */
      usb[port].hardware->ENDPTCOMPLETE = 0x00010001;
      /* enable NAK interrupts */
      usb[port].hardware->ENDPTNAKEN |= 0x00010001;

      usb[port].P_EPCallback[0](port, USB_EVT_SETUP);
    }

    /* handle completion interrupts */
    val = usb[port].hardware->ENDPTCOMPLETE;
    if (val)
    {
      usb[port].activity           = 1;
      usb[port].hardware->ENDPTNAK = val;

      /* EP 0 - OUT */
      if (val & 1)
      {
        usb[port].hardware->ENDPTCOMPLETE = 1;
        if (usb[port].ep_TD[0].total_bytes & STATUS_BITS)
          SetError(port);
        usb[port].P_EPCallback[0](port, USB_EVT_OUT);
      }
      /* EP 0 - IN */
      if (val & (1 << 16))
      {
        usb[port].ep_TD[1].total_bytes &= CLEAR_MASK;  // isolate byte count
        if (usb[port].ep_TD[1].total_bytes != 0)
          SetError(port);
        usb[port].hardware->ENDPTCOMPLETE = (1 << 16);
        usb[port].P_EPCallback[0](port, USB_EVT_IN);
      }
      /* EP 1 - OUT */
      if (val & 2)
      {
        usb[port].hardware->ENDPTCOMPLETE = 2;
        if (usb[port].ep_TD[2].total_bytes & STATUS_BITS)
          SetError(port);
        usb[port].P_EPCallback[1](port, USB_EVT_OUT);
      }
      /* EP 1 - IN */
      if (val & (1 << 17))
      {
        usb[port].ep_TD[3].total_bytes &= CLEAR_MASK;  // isolate byte count
        usb[port].hardware->ENDPTCOMPLETE = (1 << 17);
        if (usb[port].ep_TD[3].total_bytes != 0)
          SetError(port);
        usb[port].P_EPCallback[1](port, USB_EVT_IN);
      }
      /* EP 2 - IN */
      if (val & (1 << 18))
      {
        usb[port].ep_TD[5].total_bytes &= CLEAR_MASK;  // isolate byte count
        usb[port].hardware->ENDPTCOMPLETE = (1 << 18);
        if (usb[port].ep_TD[5].total_bytes != 0)
          SetError(port);
        usb[port].P_EPCallback[2](port, USB_EVT_IN);
      }
    }

    /* handle NAK interrupts */
    if (disr & USBSTS_NAKI)
    {
      val = usb[port].hardware->ENDPTNAK;
      val &= usb[port].hardware->ENDPTNAKEN;
      if (val)
      {
        usb[port].activity           = 1;
        usb[port].hardware->ENDPTNAK = val;
        for (n = 0; n < EP_NUM_MAX / 2; n++)
        {
          if (val & (1 << n))
            usb[port].P_EPCallback[n](port, USB_EVT_OUT_NAK);
          if (val & (1 << (n + 16)))
            usb[port].P_EPCallback[n](port, USB_EVT_IN_NAK);
        }
      }
    }

    /* Start of Frame Interrupt */
    if (disr & USBSTS_SRI)
    {
      if (usb[port].SOF_Event)
        usb[port].SOF_Event(port);
    }

    /* Device Status Interrupt (Reset, Connect change, Suspend/Resume) */
    if (disr & USBSTS_URI) /* Reset */
    {
      Reset(port);
      USB_ResetCore(port);
      return;
    }

    if (disr & USBSTS_SLI) /* Suspend */
    {
      usb[port].activity              = 1;
      usb[port].connectionEstablished = 0;
    }

    if (disr & USBSTS_PCI) /* Resume */
    {
      usb[port].activity              = 1;
      usb[port].connectionEstablished = 1;
      /* check if device is operating in HS mode or full speed */
      usb[port].DevStatusFS2HS = ((port == 0) && (usb[port].hardware->PORTSC1_D & (1 << 9)));  // only port USB0 can be high-speed
    }

    /* Error Interrupt */
    if (disr & USBSTS_UEI)
    {
      usb[port].activity = 1;
      SetError(port);
    }

    /* System Error Interrupt */
    if (disr & USBSTS_SEI)
    {
      usb[port].activity = 1;
      SetError(port);
    }

    if (usb[port].activity)
      ;  // indicate general activity
  } while (1);
}

/******************************************************************************/
/** @brief		Program the dTD descriptor
    @param[in]	Edpt	Endpoint number in the endpoint queue
    @param[in]	ptrBuff	Pointer to data buffer
    @param[in]	TsfSize	Size of the transfer buffer
*******************************************************************************/
void USB_ProgDTD(uint8_t const port, uint32_t Edpt, uint32_t ptrBuff, uint32_t TsfSize)
{
  DTD_T *pDTD;

  pDTD = (DTD_T *) &usb[port].ep_TD[Edpt];

  /* Zero out the device transfer descriptors */
  memset((void *) pDTD, 0, sizeof(DTD_T));
  /* The next DTD pointer is INVALID */
  pDTD->next_dTD = 0x01;

  /* Length */
  pDTD->total_bytes = ((TsfSize & 0x7fff) << 16);
  pDTD->total_bytes |= TD_IOC;
  pDTD->total_bytes |= 0x80;

  pDTD->buffer0 = ptrBuff;
  pDTD->buffer1 = (ptrBuff + 0x1000) & 0xfffff000;
  pDTD->buffer2 = (ptrBuff + 0x2000) & 0xfffff000;
  pDTD->buffer3 = (ptrBuff + 0x3000) & 0xfffff000;
  pDTD->buffer4 = (ptrBuff + 0x4000) & 0xfffff000;

  usb[port].ep_QH[Edpt].next_dTD = (uint32_t)(&usb[port].ep_TD[Edpt]);
  usb[port].ep_QH[Edpt].total_bytes &= (~0xC0);
}

static void ClearDTD(uint8_t const port, uint32_t Edpt)
{
  DTD_T *pDTD;

  pDTD = (DTD_T *) &usb[port].ep_TD[Edpt];

  /* Zero out the device transfer descriptors */
  memset((void *) pDTD, 0, sizeof(DTD_T));
  /* The next DTD pointer is INVALID */
  pDTD->next_dTD = 0x01;
}

/******************************************************************************/
/** @brief		Write USB endpoint data
    @param[in]	EPNum	Endpoint number and direction
    					7	Direction (0 - out; 1-in)
    					3:0	Endpoint number
    @param[in]	pData	Pointer to data buffer
    @param[in]	cnt		Number of bytes to write
    @return		Number of bytes written
*******************************************************************************/
uint32_t USB_WriteEP(uint8_t const port, uint32_t EPNum, uint8_t *pData, uint32_t cnt)
{
  uint32_t n = USB_EP_BITPOS(EPNum);

  USB_ProgDTD(port, EPAdr(EPNum), (uint32_t) pData, cnt);
  /* prime the endpoint for transmit */
  usb[port].hardware->ENDPTPRIME |= (1 << n);

  return (cnt);
}

/******************************************************************************/
/** @brief		Read USB endpoint data
    @param[in]	EPNum	Endpoint number and direction
    					7	Direction (0 - out; 1-in)
    					3:0	Endpoint number
    @return		Number of bytes read
*******************************************************************************/
uint32_t USB_ReadEP(uint8_t const port, uint32_t EPNum)
{
  uint32_t cnt, n;
  DTD_T *  pDTD;

  n    = EPAdr(EPNum);
  pDTD = (DTD_T *) &(usb[port].ep_TD[n]);

  /* return the total bytes read */
  cnt = (pDTD->total_bytes >> 16) & 0x7FFF;
  cnt = usb[port].ep_read_len[EPNum & 0x0F] - cnt;
  return (cnt);
}

/******************************************************************************/
/** @brief		Enqueue read request
    @param[in]	EPNum	Endpoint number and direction
    					7	Direction (0 - out; 1-in)
    					3:0	Endpoint number
    @param[in]	pData	Pointer to data buffer
    @param[in]	len		Length of the data buffer
    @return		Length of the data buffer
*******************************************************************************/
uint32_t USB_ReadReqEP(uint8_t const port, uint32_t EPNum, uint8_t *pData, uint32_t len)
{
  uint32_t num = EPAdr(EPNum);
  uint32_t n   = USB_EP_BITPOS(EPNum);

  USB_ProgDTD(port, num, (uint32_t) pData, len);
  usb[port].ep_read_len[EPNum & 0x0F] = len;
  /* prime the endpoint for read */
  usb[port].hardware->ENDPTPRIME |= (1 << n);
  return len;
}

/******************************************************************************/
/** @brief		Gets the number of bytes left to be sent
    @param[in]	endbuff	End of buffer address
    @param[in]	ep		Endpoint number and direction
    					7	Direction (0 - out; 1-in)
    					3:0	Endpoint number
    @return		        number of bytes left to be sent
*******************************************************************************/
int32_t USB_Core_BytesToSend(uint8_t const port, uint32_t ep)
{
  return usb[port].ep_TD[EPAdr(ep)].total_bytes >> 16;
}

void USB_Core_Device_Descriptor_Set(uint8_t const port, const uint8_t *ddesc)
{
  usb[port].DeviceDescriptor = ddesc;
}

void USB_Core_Device_FS_Descriptor_Set(uint8_t const port, const uint8_t *fsdesc)
{
  usb[port].FSConfigDescriptor        = fsdesc;
  usb[port].FSOtherSpeedConfiguration = fsdesc;
}

void USB_Core_Device_HS_Descriptor_Set(uint8_t const port, const uint8_t *hsdesc)
{
  usb[port].HSConfigDescriptor        = hsdesc;
  usb[port].HSOtherSpeedConfiguration = hsdesc;
}

void USB_Core_Device_String_Descriptor_Set(uint8_t const port, const uint8_t *strdesc)
{
  usb[port].StringDescriptor = strdesc;
}

void USB_Core_Device_Device_Quali_Descriptor_Set(uint8_t const port, const uint8_t *dqdesc)
{
  usb[port].DeviceQualifier = dqdesc;
}

void USB_Core_Interface_Event_Handler_Set(uint8_t const port, InterfaceEventHandler ievh)
{
  usb[port].Interface_Event = ievh;
}

void USB_Core_Class_Request_Handler_Set(uint8_t const port, ClassRequestHandler csrqh)
{
  usb[port].Class_Specific_Request = csrqh;
}

void USB_Core_SOF_Event_Handler_Set(uint8_t const port, SOFHandler sofh)
{
  usb[port].SOF_Event = sofh;
}

/******************************************************************************/
/** @brief		USB Interrupt Service Routine
*******************************************************************************/
void USB0_IRQHandler(void)
{
  Handler(0);
}
void USB1_IRQHandler(void)
{
  Handler(1);
}
