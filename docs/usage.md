# Using the stick

## With BrickController2

1. Use a BC2 Windows build that contains the USB advertiser. Currently that is branch
   `feature/usbadvertising` of [J0EK3R/brickcontroller2](https://github.com/J0EK3R/brickcontroller2).
2. Plug in the stick.
3. On the settings page, set "USB advertiser" to **Auto**. The status line shows the stick and its COM port.
4. Advertising devices such as MouldKing, CaDA, JieStar and PowerBox now use the stick. Without a stick,
   BC2 falls back to Windows Bluetooth.

## LED states

Priority: ERROR > BUTTON_STOP > TIMEOUT > ADVERTISING > CONNECTED > NO_HOST > BOOT.
Brightness is limited by `BC2ADV_LED_BRIGHTNESS` (menuconfig, default 20 of 255).

| State | Condition | Pattern |
|-------|-----------|---------|
| BOOT | Startup, BLE not synced | white, solid |
| NO_HOST | Powered, no session | blue, 1 Hz blink |
| CONNECTED | Session, no running set | green, solid |
| ADVERTISING | At least one set running | cyan pulse, short white flash on every update |
| ERROR | BLE error, no resources, or corrupt frames in the last 2 s | red, 4 Hz blink |
| TIMEOUT | Keep-alive timeout happened (until the next HELLO) | orange, solid |
| BUTTON_STOP | Button pressed | red for 300 ms, then CONNECTED |

## Button

A short press stops all sets and keeps the session (`EVT_BUTTON`). A long press has no function in
version 1.

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
- **The stick goes quiet by itself (LED orange):**
  - The host did not send a valid frame within `keepalive_ms`.
  - Send a `PING` at least every `keepalive_ms / 4`.
