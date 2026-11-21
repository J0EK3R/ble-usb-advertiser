# Development

## Layout

```
main/                       app_main: wiring, tasks, button
components/bc2_protocol/    pure C: CRC, framer, codec, AD data builder, GATT codecs (host tested)
components/bc2_core/        pure C: session state machine, safety rules, GATT logic (bc2_gatt.c)  (host tested with fakes)
components/bc2_txq/         pure C: TX frame queue with drop classes      (host tested)
components/bc2_scanfilter/  pure C: scan report filter, address type cache (host tested)
components/bc2_ble/         NimBLE adapters: extended advertising (bc2_ble.c), scanner + GATT central (bc2_gatt_nimble.c)
components/bc2_usb/         USB Serial/JTAG byte transport with TX task and queue
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

The first build downloads the component `espressif/led_strip` from the ESP component registry.
`dependencies.lock` pins its version.

## Build and flash

```
. ~/esp/esp-idf/export.sh
idf.py set-target esp32s3      # first time only
idf.py build
idf.py -p /dev/ttyACM0 flash   # COMx on Windows
```

- If the port is not found, hold the stick's button while plugging it in (ROM download mode).
- Last measured build (firmware 0.2.0): application binary 0x713a0 bytes, 44 % of the 1 MB app partition; static DRAM 108 KB.
- Kconfig (`idf.py menuconfig`, menu "BC2 USB Advertiser"): `BC2ADV_ENABLE_GATT` (scanner + GATT, default on) and
  `BC2ADV_MAX_CONNS` (default 4, announced in HELLO_RSP). `sdkconfig.defaults` only applies to a fresh `sdkconfig`: delete
  `sdkconfig` (it is not committed) after changing it.
- The CI workflow also creates a single flashable image:
  `esptool.py --chip esp32s3 merge_bin -o bc2-usb-advertiser-merged.bin @flash_args`, run in `build/`.
- The build fails with a missing component if the `led_strip` managed component cannot be downloaded. Run
  the build once with network access.

## Tests

```
# C unit tests (protocol and GATT codecs, framer, TX queue, scan filter, session and GATT logic with fake adapters)
cmake -S test/host -B build-host -DCMAKE_BUILD_TYPE=Debug
cmake --build build-host
ctest --test-dir build-host --output-on-failure

# Python tests (same golden vectors as the C tests; GATT session logic against a scripted fake link)
python3 -m unittest discover -s tools/tests -v
```

## CI

`.github/workflows/ci.yml` runs the host tests and the firmware build on every push and pull request. It
uploads the binaries, including the merged image, as artifact `bc2-usb-advertiser-firmware`.

## Releases

A release is a tag `vX.Y.Z` that matches `BC2_FW_MAJOR/MINOR/PATCH` in
`components/bc2_protocol/include/bc2_protocol.h`. Each release carries these assets:

- `bc2-usb-advertiser-vX.Y.Z-merged.bin`, taken from the CI artifact of the tagged commit
- `bc2adv.py`
- `SHA256SUMS.txt`

## Design notes: GATT central

- `bc2_gatt` (pure C) is driven by requests (`bc2_gatt_handle_request`), adapter events (`bc2_gatt_pump`) and time
  (`bc2_gatt_tick`); the radio is reached only through `bc2_gatt_ops_t`. `test/host/fake_ble.c` implements a scriptable
  fake adapter, so every state machine path is tested without hardware. Tests that depend on the number of links run
  for `max_conns` 1, 4, 9 and 16.
- Threads: USB RX task and core task call into the core under one mutex; the NimBLE host task only queues events
  (`bc2_gatt_event_t`, 128 slots, droppable events are refused when the queue is half full) and wakes the core task; the
  TX task alone blocks on USB writes. The core never calls the adapter from the host task and the adapter never calls the
  core.
- Safety: a safety stop (keep-alive timeout, HELLO, button) runs the failsafe writes, terminates all links, stops the scan
  and the advertising sets; links that survive are terminated again after 1 s and the chip is reset after 2 s.
- To add a GATT operation: protocol constants and codec (`bc2_gatt_protocol.h`, `bc2_gatt_codec.c`), golden vector
  (`tools/bc2adv.py` + both test suites), core handler (`bc2_gatt.c`), adapter op (`bc2_gatt_nimble.c`), docs.

## Event queue overflow

The adapter queue (`EVENT_QUEUE_LEN` 128 in `bc2_gatt_nimble.c`) carries state events from the NimBLE host task
to the core task. One discovery can post about 50 events in a burst. If the core task is delayed long enough
to fill the queue, a state event is lost, `s_state_lost` is raised and the core runs a safety stop of all links
(safe, but harsh). Remedies if it ever shows up: raise the length further (about 80 bytes per slot) or pace
the discovery posting.

At init the adapter logs a warning when `CONFIG_BT_CTRL_BLE_MAX_ACT - advertising sets - 1 scan < MAX_LINKS`;
CONNECT then fails with `NO_RESOURCES` once the controller budget is used up.
