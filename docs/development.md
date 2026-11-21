# Development

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
- Last measured build: application binary 0x6a450 bytes, about 42 % of the 1 MB app partition.
- The CI workflow also creates a single flashable image:
  `esptool.py --chip esp32s3 merge_bin -o bc2-usb-advertiser-merged.bin @flash_args`, run in `build/`.
- The build fails with a missing component if the `led_strip` managed component cannot be downloaded. Run
  the build once with network access.

## Tests

```
# C unit tests (protocol, framer, codec, session logic with a fake BLE adapter)
cmake -S test/host -B build-host -DCMAKE_BUILD_TYPE=Debug
cmake --build build-host
ctest --test-dir build-host --output-on-failure

# Python framing tests (same golden vectors as the C tests)
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
