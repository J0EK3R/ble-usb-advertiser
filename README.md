# ble-usb-advertiser

Firmware for an **M5Stack AtomS3U** (ESP32-S3) that works as a USB "BLE advertising stick" for
BrickController2 (BC2) on Windows. The host sends advertising sets (manufacturer id + raw payload) over
a USB serial port using a small binary protocol. The stick puts them on air as **legacy advertising
PDUs**, can change the payload of a running set within a few milliseconds and stops everything when the
host disappears (keep-alive timeout) or the button is pressed.

- Protocol contract (for BC2 task 008): [`docs/protocol.md`](docs/protocol.md)
- Hardware notes: [`docs/hardware.md`](docs/hardware.md)
- Manual hardware checklist (not yet executed, no stick was available): [`docs/verification.md`](docs/verification.md)

## Quick start: flash the firmware with the browser (no toolchain needed)

Works on Windows, Linux and macOS with **Chrome or Edge**. The browser needs WebSerial, which
Firefox and Safari do not provide.

1. **Download the firmware image.**
   - Open the [latest release](https://github.com/J0EK3R/ble-usb-advertiser/releases/latest) and
     download **`bc2-usb-advertiser-<version>-merged.bin`** from its assets. This needs no GitHub login.
   - The file is a single image with the bootloader, the partition table and the app.
   - Development builds of `main` are available from the
     [CI runs](https://github.com/J0EK3R/ble-usb-advertiser/actions/workflows/ci.yml?query=branch%3Amain+is%3Asuccess)
     as artifact `bc2-usb-advertiser-firmware`. You need a GitHub login to download them, and they expire
     after 90 days. Use `bc2-usb-advertiser-merged.bin` from the zip.
2. **Plug in the stick.** On Windows it shows up as one "USB Serial Device (COMx)" with hardware id
   `VID_303A&PID_1001&MI_00`. A second entry "USB JTAG/serial debug unit" is the debug interface and
   not a COM port.
3. **Flash it with one of these web flashers:**
   - **ESP Launchpad** (by Espressif): <https://espressif.github.io/esp-launchpad/>
     1. Select the **DIY** tab.
     2. Click **Connect** and choose the stick's port (`303A:1001`).
     3. Add the downloaded `.bin` file with flash address **`0x0`**.
     4. Click **Program**.
   - **ESPHome Web**: <https://web.esphome.io/>
     1. Click **Connect** and choose the port.
     2. Click **Install** and pick the downloaded `.bin` file. It is written to address `0x0`
        automatically. The file does not have to be an ESPHome firmware.
   - The address must be **`0x0`**, because the merged image starts with the bootloader. Any other
     address leaves the stick unbootable until it is flashed again correctly.
   - A successful run ends with `Wrote 500832 bytes ... at 0x0` (the size depends on the version)
     and `Leaving...`. The message "Not changing the image" is only information.
4. **Unplug the stick and plug it in again.** The LED first shows white (boot) and then **blinks blue
   once per second**: the firmware runs and waits for a host. See [LED states](#led-states).
5. **Check the stick with the host tool.** This needs Python 3. Download
   [`tools/bc2adv.py`](tools/bc2adv.py) first.
   ```
   pip install pyserial
   python bc2adv.py ports                     # lists candidate ports
   python bc2adv.py --port COM9 hello         # LED turns green, prints version and max sets
   python bc2adv.py --port COM9 hold --data 0102030405060708 --seconds 10   # LED pulses cyan
   ```
   While `hold` runs, a BLE scanner such as nRF Connect shows manufacturer data `0xFFF0` with the
   payload `0102030405060708`.

**Troubleshooting**

- **Connect fails, or no port shows up:** hold the stick's button while plugging it in. This starts
  the ROM download mode. Then connect again. In download mode Windows may assign a different COM number.
  A stick with factory firmware usually connects without this step.
- **Flashing aborts:** choose a lower baud rate, for example 115200, before you click Connect.
- **Two COM ports with the same `VID_303A&PID_1001`:** another ESP32-S3/C3 board is attached. In a VM,
  the stick may also be passed through twice. Unplug the stick to see which port is its own.

## Features

- Up to 5 simultaneous advertising sets (reported in `HELLO_RSP`), each with its own interval, TX power,
  PDU type and manufacturer data; in-place updates of running sets.
- Safety: all sets stop on keep-alive timeout, button press, `HELLO`; hard reset if a stop fails.
- RGB LED shows the state, the button is an emergency stop.
- Host test tool `tools/bc2adv.py`; unit tests run on Linux without hardware.

## Layout

```
main/                       app_main: wiring, tasks, button
components/bc2_protocol/    pure C: CRC, framer, codec, AD data builder   (host tested)
components/bc2_core/        pure C: session state machine, safety rules   (host tested with fakes)
components/bc2_ble/         NimBLE extended advertising adapter (legacy PDUs)
components/bc2_usb/         USB Serial/JTAG byte transport
components/bc2_led/         WS2812 pattern output
components/bc2_board/       pin definitions
test/host/                  CMake/CTest unit tests (no ESP-IDF needed)
tools/                      Python test tool and its unit tests
```

## Toolchain (pinned: ESP-IDF v5.5.6)

```
sudo apt install git wget flex bison gperf python3 python3-venv cmake ninja-build ccache libffi-dev libssl-dev dfu-util libusb-1.0-0
mkdir -p ~/esp && cd ~/esp
git clone --depth 1 --recursive --shallow-submodules -b v5.5.6 https://github.com/espressif/esp-idf.git
cd esp-idf && ./install.sh esp32s3
# without sudo: cmake and ninja can also come from ESP-IDF itself:
#   python3 tools/idf_tools.py install cmake ninja
. ~/esp/esp-idf/export.sh
```

The component `espressif/led_strip` is downloaded from the ESP component registry on the first build
(`dependencies.lock` pins it).

## Build and flash

```
. ~/esp/esp-idf/export.sh
idf.py set-target esp32s3      # first time only
idf.py build
idf.py -p /dev/ttyACM0 flash   # COMx on Windows
```

If the port is not found, hold the stick's button while plugging it in (ROM download mode).
Last measured build: application binary 0x6a450 bytes (about 42 % of the 1 MB app partition).
The CI workflow additionally creates a single flashable image
(`esptool.py --chip esp32s3 merge_bin -o bc2-usb-advertiser-merged.bin @flash_args` in `build/`).

## Tests

```
# C unit tests (protocol, framer, codec, session logic with a fake BLE adapter)
cmake -S test/host -B build-host -DCMAKE_BUILD_TYPE=Debug
cmake --build build-host
ctest --test-dir build-host --output-on-failure

# Python framing tests (same golden vectors as the C tests)
python3 -m unittest discover -s tools/tests -v
```

## Host tool

```
pip install -r tools/requirements.txt
python tools/bc2adv.py ports                       # list candidate ports (VID 303A / PID 1001) and probe them
python tools/bc2adv.py hello                       # capabilities
python tools/bc2adv.py hold --set 0 --mfr 0xFFF0 --data 0102030405060708 --seconds 30
python tools/bc2adv.py hold --set 0 --data 0102 --pdu scan --no-flags --interval 160
python tools/bc2adv.py shell                       # interactive: start/update/stop/stopall/status
python tools/bc2adv.py bench --set 0 --rate 50 --seconds 10
python tools/bc2adv.py fuzz
```

`--port COMx` overrides auto-detection, `-v` dumps all frames, `--no-stop` (hold) leaves the shutdown to
the keep-alive timeout. The tool opens the port with DTR/RTS de-asserted.

## LED states

Priority: ERROR > BUTTON_STOP > TIMEOUT > ADVERTISING > CONNECTED > NO_HOST > BOOT.
Brightness is limited by `BC2ADV_LED_BRIGHTNESS` (menuconfig, default 20 of 255).

| State | Condition | Pattern |
|-------|-----------|---------|
| BOOT | Startup, BLE not synced | white, solid |
| NO_HOST | Powered, no session | blue, 1 Hz blink |
| CONNECTED | Session, no running set | green, solid |
| ADVERTISING | At least one set running | cyan pulse, short white flash on every update |
| ERROR | BLE error / no resources / corrupt frames in the last 2 s | red, 4 Hz blink |
| TIMEOUT | Keep-alive timeout happened (until the next HELLO) | orange, solid |
| BUTTON_STOP | Button pressed | red for 300 ms, then CONNECTED |

A short button press stops all sets and keeps the session (`EVT_BUTTON`). A long press has no function
in version 1.

## Troubleshooting

- **No COM port / port busy**: hold the button while plugging in for download mode and flash again.
  Close other programs using the port.
- **Chip resets when the port is opened**: the terminal program toggles DTR/RTS. Disable both (the tool
  and the documented BC2 settings do) - see `docs/protocol.md` section 9.
- **Garbage right after plugging in**: the ROM bootloader prints text; the tool and the parser skip it.
- **Windows without driver**: Windows 10/11 ship a CDC driver for the USB Serial/JTAG device; if the
  port shows up as "unknown device" install the latest Windows updates.
- **Stick goes quiet by itself**: the host did not send a valid frame within `keepalive_ms`
  (LED orange). Send `PING` at least every `keepalive_ms / 4`.
- **Build fails with a missing component**: run the build with network access once so the
  `led_strip` managed component can be downloaded.

## Scope

Advertising only. GATT messages (stick as GATT central/proxy) are a possible later task; a message TYPE
range is reserved for them in the protocol but nothing is implemented.
