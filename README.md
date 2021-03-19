# midibridge
 hardware + firmware + tools for a ERP test unit based on the USB host-to-host MIDI bridge
 
## Project Organization
* firmware/src contains two projects:
  * the microcontroller main firmware application. Required packages: cmake, gcc, gcc-arm-none-eabi. CMake switches : 
      * `cmake -DEVAL_BOARD=On|Off path/to/source-dir` --> Force GPIOs for LEDs etc as wired on the evalution board. When set to 'Off' auto-detection will be used.
      * `cmake -DSEPARATE_USB_DEVICE_IDS=On|Off path/to/source-dir` --> use different USB device IDs and strings for HS and FS port, for debug/test.
  * a uC firmware component 'in-app-flasher' to flash this firmware into the uC. The image of this flasher is uploaded into RAM via USB in form of a MIDI SysEx message and then executed. The image of the main firmware is contained (statically linked) within the in-app-flasher and hence the executed code can flash the new firmware.
* tools/mk-sysex is a helper tool mainly for use at build-time to create the proper SysEx message from the binary image of the in-app-flasher.
Those helper tools are compiled for the platform the build system is running on (no docker encapsulation), and as long as that is a x86 Linux, any x86 linux-based system can run these standalone as well.

Toolchain setups are provided for two platforms:
* automated build of all components with CMake
* manual build of the main application with LPCXpresso under Windows. This is currently required to build the artifact needed for flashing the *blank* uC chip at production time with a hardware JTAG programmer. A proper LPXxpresso project is needed, both to create the \*.axf file first and then to flash the artifact anytime later.


### Inner dependencies
Whenever there is a change in any of the header parameters of the SysEx message (see doc/SysEx_DeviceInterface_V1.0.odt) in a new firmware revision, it is important to use the mk-sysex tool from the *current* firmware revision installed in the product to create the SysEx. Otherwise the device will reject the SysEx message.
Currently, the way to accomplish this in the build is to trick the build into using another, the older, *mk-sysex* than the one it just compliled:
- make a clean build (``make clean; make``) of the current firmware of the product and save the *mk-sysex* executable somewhere.
- switch to the new firmware and make a clean build.
- copy the saved *mk-sysex* into the build output dir replacing the new one, and run ``touch tools/mk-sysex/mk-sysex)`` to update its time-stamp.
- run *make* again, which now will detect that the *mk-sysex* executable has changed and thus will re-invoke the creation of the SysEx file for the update with the correct version of *mk_sysex*
