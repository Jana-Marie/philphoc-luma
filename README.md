# philphoc-luma

Philphoc-luma is a ZigBee (e.g. Philips Hue) to addressable LED-strip (e.g. WS2812, SK6812) bridge. It is based on an E75-2G4M20S and a STM32G030.

![](ppl.jpeg)
![](front.png)

## Firmware

There are two firmwares to be flashed, one for the STM32 and one for the E75-2G4M20S.

### ESP32

This is a simple GCC and Make based project, refer to the makefile and use your favourite flasher.

### E75

Please refer to the awesome works of PeeVeeOne to flash Light_ColorLight_JN5168_RGB onto the E75-2G4M20S module.

## License

Copyright Jana Marie Hemsing 2023. This source describes Open Hardware and is licensed under the CERN- OHL-S v2.

You may redistribute and modify this source and make products using it under the terms of the CERN-OHL-S v2 (LICENSE).

This source is distributed WITHOUT ANY EXPRESS OR IMPLIED WARRANTY, INCLUDING OF MERCHANTABILITY, SATISFACTORY QUALITY AND FITNESS FOR A PARTICULAR PURPOSE. Please see the CERN-OHL-S v2 for applicable conditions.

Source location: https://github.com/Jana-Marie/philphoc-luma As per CERN-OHL-S v2 section 4, should You produce hardware based on this source, You must where practicable maintain the Source Location visible on the external case of philphoc-luma or other products you make using this source