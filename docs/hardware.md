# Hardware notes: M5Stack AtomS3U

Target board: M5Stack AtomS3U (ESP32-S3FN8, 8 MB flash, no PSRAM, USB-A plug). **Not available during
development**: everything below is UNVERIFIED (pins were taken from memory, not from the schematic) and must be
checked against the schematic when the
board arrives (see `verification.md`).

| Item | Value | Where in the code |
|------|-------|-------------------|
| SoC | ESP32-S3FN8, 8 MB flash | `sdkconfig.defaults` (`CONFIG_ESPTOOLPY_FLASHSIZE_8MB`) |
| RGB LED | one WS2812 on GPIO35 | `components/bc2_board/include/board.h` |
| Button | GPIO41, active low | `board.h`, internal pull-up enabled |
| USB D-/D+ | GPIO19 / GPIO20 (native USB Serial/JTAG) | fixed in silicon |

## USB identity

The native USB Serial/JTAG peripheral enumerates with the fixed identity **VID 0x303A, PID 0x1001**
("USB JTAG/serial debug unit"). It cannot carry a custom product string and is shared by all ESP32-S3/C3
development boards. The firmware is therefore identified by the `HELLO_RSP` magic `BC2ADV`
(`docs/protocol.md` section 9). On Windows 10/11 the port appears as a COM port without driver
installation. A TinyUSB CDC backend with its own product string was considered and rejected by the
product owner for version 1.

## Reset and download mode

- Hold the button while plugging in the stick to enter the ROM download mode (UNVERIFIED, not yet
  tested on the board; needed if flashing from
  `idf.py flash` cannot find the port, for example after a crash loop).
- The host must keep DTR and RTS de-asserted. The USB Serial/JTAG peripheral interprets certain DTR/RTS
  sequences as "reset" or "enter bootloader" (this is how esptool works).
- The ROM bootloader may print boot text on the USB port right after reset. The firmware itself prints
  nothing (console and logs disabled); the protocol parser skips such garbage.

## Radio

- BLE 5 controller with extended advertising instances. The firmware uses 5 instances (1 + `CONFIG_BT_NIMBLE_MAX_EXT_ADV_INSTANCES = 4`)
  but only with legacy PDUs (`legacy_pdu = 1`): 31 byte payload, channels 37/38/39, 1M PHY.
- Every set uses its own static random address derived from the chip MAC.
- Default PDU type ADV_IND (connectable + scannable) as Android does. A connection attempt is terminated
  immediately; the set keeps advertising.
