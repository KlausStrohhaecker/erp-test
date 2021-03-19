/******************************************************************************/
/** @file		nl_usb_descmidi.h
    @date		2013-01-23
    @brief    	Descriptors header for the USB-MIDI driver
    @example
    @ingroup  	USB_MIDI
    @author		Nemanja Nikodijevic 2013-01-23
*******************************************************************************/
#pragma once

#include "sys/nl_version.h"

/** MIDI device descriptor fields
 * @{
 */
/** bcdUSB 2.0 */
#define BCDUSB_2_0 0x0200

#ifdef SEPARATE_USB_DEVICE_IDS  // use different IDs and strings for HS and FS port, for debug/test
#warning Compiling for different USB IDs and strings for HS and FS port, for debug/test

/** Product Id */
// Host id's the device by this number, no matter wrt FS or HS port used.
// Therefore, use different IDs for FS and FS for debugging enumeration or for tests
// with both ports attached to the same computer so that ports can be identified.
// Different numbers will also create different ID strings in descmidi.c to help this identification.
// Gotcha: Windows will not display any updated string for a device as long as the number
// does not change as well. Remove the (stale) entries in the Device Manager, but to be sure
// make individual HS and FS numbers both different from the shared number, like 2 and 3 !!
// Any future NLL MIDI Products also must not have the same number(s).
#define PRODUCT_ID_HS 0x0002
#define PRODUCT_ID_FS 0x0003
/** Vendor Id */
#define VENDOR_ID 0x4E4C  // unofficial, "NL"

#else                         // production setting

/** Product Id */
// see https://github.com/obdev/v-usb/blob/master/usbdrv/USB-IDs-for-free.txt
#define PRODUCT_ID_HS 0x05E4  // PID for "textual identification"
#define PRODUCT_ID_FS 0x05E4  // PID for "textual identification"
/** Vendor Id */
#define VENDOR_ID     0x16C0  // VID for class-compliant MIDI devices

#endif

/** bcdDevice */
#define BCD_DEVICE ((SW_VERSION_MAJOR << 8) | (SW_VERSION_MINOR_H << 4) | SW_VERSION_MINOR_L)
/** @} */

/** Size of the configuration block */
#define CONFIG_BLOCK_SIZE 0x0065
/** Size of standard Audio endpoint descriptor */
#define USB_AUDIO_ENDPOINT_DESC_SIZE 0x09

/** Audio interface subclasses
 * @{
 */
/** Audio Control subclass */
#define USB_SUBCLASS_AUDIO_CONTROL 0x01
/** MIDI Streaming subclass */
#define USB_SUBCLASS_MIDI_STREAMING 0x03
/** @} */

/** MIDI Streaming class-specific interface descriptor subtypes
 * @{
 */
/** MS_HEADER */
#define USB_MIDI_HEADER_SUBTYPE 0x01
/** MIDI_IN_JACK */
#define USB_MIDI_IN_JACK_SUBTYPE 0x02
/** MIDI_OUT_JACK */
#define USB_MIDI_OUT_JACK_SUBTYPE 0x03
/** @} */

/** Class-specific descriptor types
 * @{
 */
/** Class-specific interface descriptor */
#define USB_CS_INTERFACE_DESC_TYPE 0x24
/** Class-specific endpoint descriptor */
#define USB_CS_ENDPOINT_DESC_TYPE 0x25
/** Class-specific MIDI Streaming general endpoint descriptor subtype */
#define USB_MIDI_EP_GENERAL_SUBTYPE 0x01
/** @} */

/** Audio device class specification that the device complies to */
#define BCDADC_1_0 0x0100

/** Size of class-specific descriptors
 * @{
 */
/** Size of CS Audio Control header interface descriptor */
#define USB_CS_AUDIO_CONTROL_SIZE 0x09
/** Total size of CS Audio Control interface descriptors */
#define USB_CS_AUDIO_CONTROL_TOTAL 0x0009
/** Size of CS MIDI Streaming header interface descriptor */
#define USB_CS_MIDI_STREAMING_SIZE 0x07
/** Total size of CS MIDI Streaming interface descriptors */
#define USB_CS_MIDI_STREAMING_TOTAL 0x0041
/** Size of MIDI IN jack descriptor */
#define USB_MIDI_IN_JACK_SIZE 0x06
/** Size of MIDI OUT jack descriptor */
#define USB_MIDI_OUT_JACK_SIZE 0x09
/** Size of MIDI Streaming endpoint descriptor */
#define USB_MIDI_ENDPOINT_DESC_SIZE 0x05
/** @} */

/** USB MIDI jack types
 * @{
 */
/** Embedded jack */
#define USB_MIDI_JACK_EMBEDDED 0x01
/** External jack */
#define USB_MIDI_JACK_EXTERNAL 0x02
/** @} */

/** Bulk transfer maximum packet size
 * @{
 */
/** Full-speed */
#define USB_FS_BULK_SIZE (64)  // can only be 8, 16, 32 or 64
/** Hi-speed */
#define USB_HS_BULK_SIZE (512)  // can only be 512
/** @} */

/** MIDI endpoint numbers
 * @{
 */
/** Bulk OUT endpoint - 0x01 */
#define MIDI_EP_OUT 0x01
/** Bulk IN endpoint  - 0x02 */
#define MIDI_EP_IN 0x02
/** @} */

extern const uint8_t USB0_MIDI_DeviceDescriptor[];
extern const uint8_t USB1_MIDI_DeviceDescriptor[];
extern const uint8_t USB_MIDI_FSConfigDescriptor[];
extern const uint8_t USB_MIDI_HSConfigDescriptor[];
extern const uint8_t USB0_MIDI_StringDescriptor[];
extern const uint8_t USB1_MIDI_StringDescriptor[];
extern const uint8_t USB_MIDI_DeviceQualifier[];
