# Flashing the firmware

## With the browser (recommended, no toolchain needed)

This works on Windows, Linux and macOS with **Chrome or Edge**. Firefox and Safari do not support
WebSerial, so they cannot be used.

1. **Download the firmware image.**
   - Open the [latest release](https://github.com/J0EK3R/ble-usb-advertiser/releases/latest).
   - Download **`bc2-usb-advertiser-<version>-merged.bin`** from its assets. No GitHub login is needed.
   - The file is a single image with the bootloader, the partition table and the app.
   - Development builds of `main` are available from the
     [CI runs](https://github.com/J0EK3R/ble-usb-advertiser/actions/workflows/ci.yml?query=branch%3Amain+is%3Asuccess)
     as artifact `bc2-usb-advertiser-firmware`. Downloading them needs a GitHub login, and they expire after
     90 days. Use `bc2-usb-advertiser-merged.bin` from the zip.
2. **Plug in the stick.**
   - On Windows it shows up as one "USB Serial Device (COMx)" with hardware id `VID_303A&PID_1001&MI_00`.
   - A second entry, "USB JTAG/serial debug unit", is the debug interface and not a COM port.
3. **Flash the image with one of these web flashers.**
   - **ESP Launchpad** (by Espressif): <https://espressif.github.io/esp-launchpad/>
     1. Select the **DIY** tab.
     2. Click **Connect** and choose the stick's port (`303A:1001`).
     3. Add the downloaded `.bin` file with flash address **`0x0`**.
     4. Click **Program**.
   - **ESPHome Web**: <https://web.esphome.io/>
     1. Click **Connect** and choose the port.
     2. Click **Install** and pick the downloaded `.bin` file. It is written to address `0x0` automatically,
        and it does not have to be an ESPHome firmware.
   - The address must be **`0x0`**, because the merged image starts with the bootloader. With any other
     address the stick does not boot until it is flashed again correctly.
   - A successful run ends with `Wrote ... bytes ... at 0x0` and `Leaving...`. The message
     "Not changing the image" is only information.
4. **Unplug the stick and plug it in again.**
   - The LED first shows white (boot) and then **blinks blue once per second**. The firmware is running and
     waits for a host.
   - The meaning of all LED states is in [usage.md](usage.md#led-states).
5. Optional: check the stick with the host tool, see [usage.md](usage.md#host-test-tool).

## Troubleshooting

- **Connect fails or no port shows up:**
  - Hold the stick's button while plugging it in. This starts the ROM download mode. Then connect again.
  - In download mode Windows may assign a different COM number.
  - A stick with factory firmware usually connects without this step.
- **Flashing aborts:** choose a lower baud rate, for example 115200, before you click Connect.
- **Two COM ports with the same `VID_303A&PID_1001`:**
  - Another ESP32-S3/C3 board is attached, or, in a VM, the stick is passed through twice.
  - Unplug the stick to see which port is its own.

## With ESP-IDF

See [development.md](development.md#build-and-flash).
