# Hardware notes: M5Stack AtomS3U

Target board: M5Stack AtomS3U (ESP32-S3FN8, 8 MB flash, no PSRAM, USB-A plug). The LED pin (GPIO35), the flashing
and the BLE boot were confirmed on a real stick (`tasks/007-usb-advertiser-fw/hw-results.md` of the team repository); the
other entries marked UNVERIFIED below still have to be checked against the schematic (see `verification.md`).

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

## Radio: scanner and GATT central (firmware 0.2.0)

- The controller runs scan, up to 5 advertising sets and the connection events itself (time slicing). One
  connect attempt can run at a time (NimBLE has a single initiator): the firmware queues CONNECT requests and
  works through them round robin (protocol section 14).
- Scan defaults 100 ms interval / 50 ms window, connection interval 15..30 ms, slave latency 0, supervision timeout 4 s,
  ATT MTU requested by the stick 64 (a peer without MTU support keeps 23).
- Only legacy advertising PDUs are reported (no BC2 device uses extended advertising).

### Maximum number of simultaneous connections (UNVERIFIED, derived)

| Item | Value | Source |
|------|-------|--------|
| Controller activities | `CONFIG_BT_CTRL_BLE_MAX_ACT` = 10 (maximum of the Kconfig range) | `components/bt/controller/esp32c3/Kconfig.in` of ESP-IDF 5.5.6 (shared by the ESP32-S3), `sdkconfig.defaults` |
| Used by advertising | 5 (one per advertising set; the 0.1 firmware ran 5 sets with the default of 6 on the real stick) | `hw-results.md`, `CONFIG_BT_NIMBLE_MAX_EXT_ADV_INSTANCES` = 4 + 1 |
| Used by the scan | 1 | assumption; verified by spike step S4 |
| Left for links | **4** (a connect attempt becomes the link, so it does not add an activity) | 10 - 5 - 1 |
| NimBLE connection slots | `CONFIG_BT_NIMBLE_MAX_CONNECTIONS` = 4 | `sdkconfig.defaults` |
| Announced `max_conns` | **4** = min(`BC2ADV_MAX_CONNS` 4, NimBLE slots 4) | `main/Kconfig.projbuild`, `components/bc2_ble/bc2_gatt_nimble.c` |
| Memory | static DRAM 108 KB of 342 KB used, each controller activity about 0.8 KB, ACL / mbuf pools in `sdkconfig.defaults` | `idf.py size`; the heap is reported by `GATT_STATUS_RSP` (`min_free_heap`) |

The number 4 is the **ceiling on paper**. The hardware spike (`verification.md` S1-S5, `bc2adv connspike`)
measures the real stable maximum; enter the result in the table below and, if it differs, in `sdkconfig.defaults`.
The host reads `max_conns` from HELLO_RSP and never assumes a value.

### Measured limits (fill in after the spike)

| Item | Result |
|------|--------|
| Stable maximum with 5 sets + scan (10 min) | |
| First failing count and failure mode | |
| Scan and connect attempt overlap, or scan paused | |
| Minimum free heap with the maximum links | |
| Write rate per link (write without response) | |
| ACK latency WRITE_CMD / WRITE, notification latency | |

### Hub behaviour on disconnect (fill in with verification G10)

BC2 itself sends no "stop" write before it disconnects. Whether a hub stops its motors when the link drops is not
documented in the BC2 code; it is measured per hub:

| Hub | Stops by itself when the link drops? After how long? | Failsafe write needed (SET_FAILSAFE) |
|-----|------------------------------------------------------|--------------------------------------|
| LEGO DUPLO train hub | | `08 00 81 00 11 51 00 00` to the hub characteristic if it keeps running |
| MouldKing DIY | | 18 byte neutral frame `CC AA BB 01 80 80 80 80 80 80 80 80 80 80 80 80 66 33` |
| LEGO WeDo 2.0 | | two writes to `1565`: `01 01 01 00` and `02 01 01 00` |

### Future: dynamic link budget

Not implemented. The controller activities (`CONFIG_BT_CTRL_BLE_MAX_ACT` 10) are one pool shared by advertising
sets, the scan and the links, while `max_conns` is a static 4. A later task (after the spike results) may build
NimBLE with more connection slots, enforce `used links + active sets + scan <= 10` at run time (`NO_RESOURCES`
otherwise) and report the free budget to the host. The product owner decides.
