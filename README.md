# ble-usb-advertiser

Firmware that turns an **M5Stack AtomS3U** (ESP32-S3) into a USB **BLE advertising stick** for
[BrickController2](https://github.com/J0EK3R/brickcontroller2) (BC2) on Windows.

Advertising-based toys such as MouldKing, CaDA, JieStar and PowerBox are controlled by BLE advertising.
On Windows, the Bluetooth stack updates advertising data much more slowly than Android. With the stick,
BC2 sends the data over USB and the stick puts it on air within milliseconds, so the toys react as fast as
with the Android app.

## Features

- **Fast advertising:** a payload change of a running set is on air within milliseconds.
- **Several devices at once:** up to 5 simultaneous advertising sets, each with its own data, interval
  and transmit power.
- **Plug and play:** no driver is needed on Windows 10/11, and BC2 finds the stick automatically.
- **Scanner and GATT:** the stick also scans and connects to BLE devices (LEGO hubs, MouldKing DIY, ...), several
  at the same time, so BC2 can use GATT devices without a Windows Bluetooth adapter (firmware 0.2.0).
- **Safety:** everything stops when BC2 disappears (keep-alive timeout) or when the button on the stick
  is pressed: advertising sets, scan and GATT links.
- **Status LED:** shows booting, waiting for a host, connected, advertising, timeout and error.
- **Open protocol:** a small, documented serial protocol, plus a Python test tool.

## Flashing

Download the `.bin` file from the [latest release](https://github.com/J0EK3R/ble-usb-advertiser/releases/latest)
and flash it with a web flasher in Chrome or Edge at address `0x0`. No toolchain is needed. Step by step
instructions: [docs/flashing.md](docs/flashing.md).

## Documentation

| Document | Content |
|----------|---------|
| [docs/flashing.md](docs/flashing.md) | Flashing with the browser, troubleshooting |
| [docs/usage.md](docs/usage.md) | Use with BC2, LED states, button, host test tool |
| [docs/protocol.md](docs/protocol.md) | Serial protocol between host and stick |
| [docs/hardware.md](docs/hardware.md) | Board notes (AtomS3U pins, reset, download mode) |
| [docs/development.md](docs/development.md) | Code layout, toolchain, build, tests, CI, releases |
| [docs/verification.md](docs/verification.md) | Manual hardware test checklist |
