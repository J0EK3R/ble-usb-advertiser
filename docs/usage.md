# Using the stick

## With BrickController2

1. Use a BC2 Windows build that contains the USB advertiser. Currently that is branch
   `feature/usbadvertising` of [J0EK3R/brickcontroller2](https://github.com/J0EK3R/brickcontroller2).
2. Plug in the stick.
3. On the settings page, set "USB advertiser" to **Auto**. The status line shows the stick and its COM port.
4. Advertising devices such as MouldKing, CaDA, JieStar and PowerBox now use the stick. Without a stick,
   BC2 falls back to Windows Bluetooth.
5. With firmware 0.2.0 or newer (protocol 1.1) the stick can also scan and connect to GATT devices (LEGO hubs,
   MouldKing DIY, SBrick, BuWizz, ...), once BC2 supports it (task 010). The firmware announces how many devices it
   can hold at the same time (`max_conns`, shown by `bc2adv.py hello`).

## LED states

Priority: ERROR > BUTTON_STOP > TIMEOUT > ADVERTISING ("active") > CONNECTED > NO_HOST > BOOT.
Brightness is limited by `BC2ADV_LED_BRIGHTNESS` (menuconfig, default 20 of 255).

| State | Condition | Pattern |
|-------|-----------|---------|
| BOOT | Startup, BLE not synced | white, solid |
| NO_HOST | Powered, no session | blue, 1 Hz blink |
| CONNECTED | Session, no running set | green, solid |
| ADVERTISING (active) | At least one set running, or the scan runs, or a GATT link / connect attempt exists | cyan pulse, short white flash on every update |
| ERROR | BLE error, no resources, or corrupt frames in the last 2 s | red, 4 Hz blink |
| TIMEOUT | Keep-alive timeout happened (until the next HELLO) | orange, solid |
| BUTTON_STOP | Button pressed | red for 300 ms, then CONNECTED |

## Button

A short press stops all sets, drops all GATT links and stops the scan, and keeps the session (`EVT_BUTTON`). A
long press has no function in version 1.

## Host test tool

`tools/bc2adv.py` needs Python 3 and pyserial. It is also attached to every release.

```
pip install -r tools/requirements.txt              # or: pip install pyserial
python tools/bc2adv.py ports                       # list candidate ports (VID 303A / PID 1001) and probe them
python tools/bc2adv.py hello                       # capabilities; the LED turns green
python tools/bc2adv.py hold --set 0 --mfr 0xFFF0 --data 0102030405060708 --seconds 30
python tools/bc2adv.py hold --set 0 --data 0102 --pdu scan --no-flags --interval 160
python tools/bc2adv.py shell                       # interactive: start/update/stop/stopall/status
python tools/bc2adv.py bench --set 0 --rate 50 --seconds 10
python tools/bc2adv.py fuzz
```

GATT commands (firmware 0.2.0, protocol 1.1). Addresses come from `scan`; handles come from `discover`. One-shot
commands connect, do their job and disconnect again (a new HELLO ends every link, so links never survive between two
tool runs; use `gatt-shell` for a longer session):

```
python tools/bc2adv.py scan --seconds 8                            # merged ADV + scan response data per device
python tools/bc2adv.py scan --name Hub --mfr 0x0397 --duplicates   # filters, report every PDU
python tools/bc2adv.py connect AA:BB:CC:DD:EE:FF --discover --notify 0x12 --hold 30
python tools/bc2adv.py connect AA:BB:CC:DD:EE:FF --background --hold 120   # wait for a hub that is switched on later
python tools/bc2adv.py discover AA:BB:CC:DD:EE:FF                  # service / characteristic tree with CCCD handles
python tools/bc2adv.py read AA:BB:CC:DD:EE:FF 0x12
python tools/bc2adv.py write AA:BB:CC:DD:EE:FF 0x10 0800810011510000          # with response
python tools/bc2adv.py write-cmd AA:BB:CC:DD:EE:FF 0x2A 01020304              # without response
python tools/bc2adv.py notify AA:BB:CC:DD:EE:FF 0x12 --seconds 10             # subscribe (CCCD handle) and count
python tools/bc2adv.py gatt-bench AA:BB:CC:DD:EE:FF 0x2A --size 18 --rate 100 --seconds 10   # add --response for writes with response
python tools/bc2adv.py gatt-shell                                  # several links, failsafe, status (type "help")
python tools/bc2adv.py gatt-shell --script "scan 3; connect 0 AA:BB:CC:DD:EE:FF; discover 0; wait 5"
python tools/bc2adv.py connspike AA:..:01=0x10,0x11 AA:..:02=0x10    # maximum connection spike, see verification.md
```

In `gatt-shell` the commands are `scan [seconds] [passive]`, `connect <conn> <addr> [bg]`, `disconnect <conn>`,
`discover <conn>`, `read <conn> <handle>`, `write <conn> <handle> <hex>`, `write-cmd ...`, `notify <conn> <cccd>
[off|indicate]`, `failsafe <conn> <slot> <handle> <hex>` (up to 4 "safe state" writes per link that the stick executes
before it drops the link on timeout, button or HELLO; empty hex clears the slot), `gatt-status`, `wait <s>`.
Connection ids are chosen by the user (0..15); events (notifications, disconnects, dropped counters) are printed as
they arrive. All GATT requests are sent once and never retransmitted.

- `--port COMx` overrides the auto-detection.
- `-v` dumps all frames.
- `--no-stop` (for `hold`) leaves the shutdown to the keep-alive timeout.
- The tool opens the port with DTR/RTS de-asserted.

While `hold` runs, a BLE scanner such as nRF Connect shows the manufacturer data, for example `0xFFF0`
with the payload `0102030405060708`.

## Troubleshooting

- **No COM port, or the port is busy:**
  - Close other programs that use the port.
  - Hold the button while plugging in to start the download mode, then flash again.
- **The chip resets when the port is opened:** the terminal program toggles DTR/RTS. Disable both. The
  tool and BC2 already do this. See [protocol.md](protocol.md) section 9.
- **Garbage right after plugging in:** the ROM bootloader prints text. The tool and the parser skip it.
- **Windows shows no driver:**
  - Windows 10/11 ship a CDC driver for the USB Serial/JTAG device.
  - If the port shows up as "unknown device", install the latest Windows updates.
- **A hub does not appear in `scan`:** use an active scan (default) because LEGO hubs put their name into the scan
  response; check that the hub is switched on and not connected to a phone. `scan --duplicates` shows every PDU.
- **`connect` fails with reason 0xF0:** the hub was not reachable within the timeout (default 10 s). Switch it on or use
  `--background`. Reason 0xF2 means the stick dropped the link itself (keep-alive timeout, button, HELLO).
- **`NO_RESOURCES` on connect:** more connections than `max_conns` (see `hello`). Disconnect one first.
- **The stick goes quiet by itself (LED orange):**
  - The host did not send a valid frame within `keepalive_ms`.
  - Send a `PING` at least every `keepalive_ms / 4`.
