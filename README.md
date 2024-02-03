# tiny-lamparray
This project is an implementation of [USB HID LampArray](https://learn.microsoft.com/en-us/windows-hardware/design/component-guidelines/dynamic-lighting-devices) for ATtiny88 (MH-ET Live Tiny88). The firmware controls an addressable RGBW strip of type SK6812, forwarding the control of the strip to a supported USB host through a standardized protocol.

![Lamp Array Effect](https://github.com/MatejKocourek/tiny-lamparray/raw/main/img/LampArrayEffect.gif)

# Features
The project implements everything in the [USB Lamp Array standard](https://www.usb.org/sites/default/files/hutrr84_-_lighting_and_illumination_page.pdf), making it possible for the host to create both simple ambient lighting and complex addressable effects.

As the strip contains a white LED, the project converts colorspaces to use both RGB and W channels. This allows for better representation of colors, resulting in higher CRI.

# Host support
At the moment of creating the project, supported hosts are:
- Windows 11 Build 23466 (Preview) and later
- Xbox GDK March 2023 Update 1 and later

Windows can control the LED device natively from the settings:

![Settings example](https://learn.microsoft.com/en-us/windows/uwp/devices-sensors/images/lighting/settings-dynamic-lighting-effects.png)

Software running on Windows can utilize the API for displaying various effect. Microsoft provides [this guide](https://learn.microsoft.com/en-us/windows/uwp/devices-sensors/lighting-dynamic-lamparray) for developers.

Microsoft also released a demo app that creates ambient lighting from the most prominent color on screen. It works great with this project. https://github.com/microsoft/Dynamic-Lighting-AutoRGB


# Issues
ATtiny88 is not a very capable device, its price (about 1$) being the sole advantage of it. There are several limitations that users should be aware of:

- The RAM of only 512B severely limits the number of LEDs the device can drive to about 60 (tested with 30). That can be one or two meters with the most popular strips. It should theoretically be possible to overcome this by decreasing the bit-depth of the color, but several new issues arise by doing this. If you want to drive more LEDs, it would be better idea to invest in a better platform.
- The device does not support USB natively, making use of V-USB library to simulate a low-speed device through software. The library consumes considerable amount of resources and requires absolutely precise timings. This alone would not be a problem, if the SK6812 strip also did not require the same. When sending data to the strip, the device must disable interrupts for the whole duration of the transfer. Should there be any USB event in this time, the device either freezes or disconnects from the computer permanently (until physically reconnected). The project uses some hacks to wait for a time when the PC should be quiet before updating the LED strip, and it works (and is stable-ish). However, it is not guaranteed that this will work for everyone and forever, since Microsoft can change the driver and the whole thing can just stop working or become unstable.

If you are just deciding which platform to choose, I recommend to go with something else. Microsoft released similar implementation for USB HID LampArray as part of their [ArduinoHidForWindows](https://github.com/microsoft/ArduinoHidForWindows) project. It supports Arduinos with native USB controllers.

# Documentation
I tried to explain the functionality of the project using comments in the code. I also explained how the HID report structures work (to my understanding).

# Schematic

Connect the middle pin of SK6812 to a pin on the MH-Tiny device (pin 8 is used in the code). It is recommended to use your own source of power if there is more than 30 LEDS connected.

# Compiling and installing

First, install [PlatformIO](http://platformio.org/) open source ecosystem for IoT development compatible with **Arduino** code and its command line tools (Windows, MacOS and Linux). Also, you may need to install [git](http://git-scm.com/) in your system. 

Note: with `platformIO` you don't need the Arduino IDE and install libraries, this will do it for you.

Compiling and installing:
``` bash
pio run --target upload
```

Note: you need connect your device after each compiling to upload the new firmware or reset it. More info [here](http://digistump.com/wiki/digispark/tutorials/connectingpro).

# References 

I extensively used the mentioned Microsoft Arduino HID project to implement the HID standard. Apart from that, the code for SK6812 was reworked from this source: https://github.com/Electry/Arduino_SK6812. The function for colorspace conversion utilizes a formula from this paper: [Color conversion from RGB to RGB+White while preserving hue and saturation](https://library.imaging.org/admin/apis/public/api/ist/website/downloadArticle/cic/10/1/art00054).

# License
This project is distributed with GPL license, see LICENSE file for more informations.