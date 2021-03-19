/******************************************************************************/
/** @file		nl_version.h
    @brief		define firmware version and place string in image
*******************************************************************************/
#pragma once

// version history :
//  1.01 : initial (with separate USB IDs/names for HS and FS port)

// clang-format off
// just set up the X.YZ style of version number vertically in below
#define SW_VERSION_MAJOR    1
// delimiter                .
#define SW_VERSION_MINOR_H  0
#define SW_VERSION_MINOR_L  1
// clang-format on

#define STR_IMPL_(x) #x            // stringify argument
#define STR(x)       STR_IMPL_(x)  // indirection to expand argument macros

#define SW_VERSION      \
  STR(SW_VERSION_MAJOR) \
  "." STR(SW_VERSION_MINOR_H) STR(SW_VERSION_MINOR_L)

// !! SW_VERSION in below string must follow directly after the "VERSION: " part
// as the firmware version scanner in the 'mk-sysex' tool depends on this!
// Also, one can use  grep -oP '(?<=VERSION:).*' | sed 's: ::g'  to extract version number from image
static const char VERSION_STRING[] = "\n\nNLL ERP-Test (via MIDI Host-to-Host Bridge), LPC4337, FIRMWARE VERSION: " SW_VERSION " \n\n\0\0\0";
