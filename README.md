# mac-qt-kvasercan-plugin
Qt Serial Bus plugin for the KvaserCAN devices using the KvaserCAN-Library for MacOS

This is a Qt Serial Bus plugin for KvaserCAN devices on MacOS.

The plugin uses the KvaserCAN user-space driver from [MacCAN by UV software](https://mac-can.github.io/drivers/KvaserCAN/).  It therefore supports
all devices supported by driver. See [here](https://mac-can.github.io/drivers/KvaserCAN/#features) for the supported features by device.  Since it is based
on MacCAN it is only supported on MacOS.

It has only been tested with a Leaf HS v2 device on MacOS Catalina (13.4.1) but should be supported on any device that both supports Qt and the MacCAN driver.

It has also only been tested with Qt 6.5.1

To build:
```
  git clone https://github.com/svcguymac-qt-kvasercan-plugin
  git submodule init
  cmake --build .
```
There currently isn't an install because I don't know how to do that :)

After it successfully builds, copy the .dylib file to your `plugins/canbus` folder in your Qt installation.</p>

## Acknowledgements
This is based ***very heavily*** from the PeakCAN plugin provided by Qt.  I essentially took it and ported it to the KvaserCAN library.

Thanks to the author of the PeakCAN plugin, Denis Shienkov and Qt

Thanks to Uwe Vogt of UV software for the driver
