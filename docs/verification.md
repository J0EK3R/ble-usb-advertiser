# Manual hardware verification (needs the AtomS3U)

> **HARDWARE VERIFICATION PENDING:** the GATT central (firmware 0.2.0) has only been verified by host tests and
> a build. No spike (S1-S5) or G1-G12 step has run on a real AtomS3U; the real `max_conns` is unknown.

Part 1 covers advertising (protocol 1.0), part 2 the scanner and GATT central (protocol 1.1, firmware
0.2.0). The results of the steps that were executed are collected in `tasks/007-usb-advertiser-fw/hw-results.md`
of the team repository. The firmware 0.2.0 steps (part 2) were written without hardware at hand and are **not yet
executed**: run them and fill in the result column. Tool commands are described in
[usage.md](usage.md#host-test-tool); `python tools/bc2adv.py` is called `bc2adv` below. Needs a phone with nRF Connect (or any
BLE scanner showing raw advertising data) and, for item 10, a real toy.

| # | Needs hardware | Precondition | Action | Expected result | Result |
|---|----------------|--------------|--------|-----------------|--------|
| 1 | yes | Firmware built | `idf.py -p <port> flash` (hold the button while plugging in if the port is missing) | LED white flash, then blue blinking (1 Hz). On Windows 10/11 a COM port with VID 303A / PID 1001 appears without driver | |
| 2 | yes | 1 done | `bc2adv ports`, `bc2adv hello` | The stick is listed with the BC2 probe result; hello shows magic, proto 1.1, fw 0.2.0, `max_sets` (expect 5, record the real value), `max_conns` (expect 4), MAC. LED green (session) while connected | |
| 3 | yes | 2 done | `bc2adv hold --set 0 --mfr 0xFFF0 --data 0102030405060708 --seconds 30` | nRF Connect shows manufacturer data 0xFFF0 with the bytes, flags 0x06, legacy ADV_IND (connectable); LED cyan pulse; address is a static random address | |
| 4 | yes | 3 done | `bc2adv bench --set 0 --rate 50 --seconds 10` | Round trip < 20 ms (host), device `processing_time_us` < 5000. Payload changes visible in the scanner without lag. Record min/avg/max and whether the stop/set/start fallback was needed (internal counter `s_fallback_updates`, check with a debugger or temporary build) | |
| 5 | yes | 2 done | Use `bc2adv shell`: start sets 0..max-1 with different mfr ids/payloads, update one of them | All sets visible in the scanner at the same time; updating one does not disturb the others. Record how many were seen | |
| 6 | yes | 3 done | Repeat 3 with `--pdu scan` and `--pdu nonconn`; test with the real toys (MouldKing, CaDA, JieStar, PowerBox) | Record which PDU type each toy accepts (decides the default for task 008) | |
| 7a | yes | A set is running | Kill the tool with Ctrl-C twice / use `hold --no-stop` | Advertising stops after `keepalive_ms` (<= 1.2 s); LED orange | |
| 7b | yes | A set is running | Unplug USB | Advertising stops (power gone) | |
| 7c | yes | A set is running | Close the port without STOP_ALL (`hold --no-stop`) | Stops after the timeout | |
| 7d | yes | A set is running | Press the button | Stops immediately, LED red flash, then green; session stays (`bc2adv shell`: `status` answers) | |
| 8 | yes | 2 done | `bc2adv fuzz` | Valid PINGs between garbage/truncated/corrupt frames are still answered; `parse_errors` in PONG rises | |
| 9 | yes | Windows 11 PC | Repeat 2-4 on Windows 11 with the tool (pyserial) | COM port works; DTR/RTS handling does not reset the chip | |
| 10 | yes | Real toy, telegram from BC2 logs | `bc2adv hold` with the known telegram (24 byte Android layout) | The model reacts; it stops when the tool is killed | |
| 11 | yes | 3 sets | Soak 30 min with 3 sets and 20 updates/s each (adapt `bench` or script `shell`) | No resets (uptime in PONG keeps growing), `parse_errors` stays 0. Also exercises the NimBLE adapter races (restart_instance vs. op_stop, update fallback), which have no automated test | |
| 12 | yes | - | Check pins in `board.h` against the board: LED on GPIO35, button on GPIO41 | LED colours as in usage.md, button press recognised | |
| 13 | yes | - | Record whether 5 instances really exist (HELLO `max_sets`) and whether `CONFIG_BT_NIMBLE_MAX_EXT_ADV_INSTANCES = 4` builds/boots on the ESP32-S3 | `max_sets` = 5 | |

Open questions of part 1 that only hardware can answer: do the toys need a specific PDU type or address
behaviour; does the controller accept data changes on a running legacy instance (else the fallback path
is used and the latency rises); is the 20 ms interval with 5 simultaneous sets scheduled without gaps.

# Part 2: scanner and GATT central (firmware 0.2.0, protocol 1.1)

Test devices (decided by the product owner): **LEGO DUPLO train hub**, **MouldKing DIY** hub, **LEGO WeDo 2.0** hub
(all GATT) and the **MouldKing MK4.0** (advertising). A phone with nRF Connect (or a second ESP32 running a minimal
NimBLE peripheral) serves as an extra peripheral for protocol level checks and for link counts above 3.
Handles come from `discover`; the examples use `<H>` for a handle. Hub facts used below (read from the BC2 code):

| Hub | Recognised by | Service / characteristic | Write |
|-----|---------------|--------------------------|-------|
| DUPLO train hub | manufacturer data id `0x0397`, `data[3] == 0x20`, name from the scan response | `00001623-...` / `00001624-1212-efde-1623-785feabcd123` (one characteristic, notify + write) | with response, 8 bytes `08 00 81 00 11 51 00 00` (motor off) |
| MouldKing DIY | manufacturer data id `0xAC33` | `0000ae3a-...` / `0000ae3b-...` | no response, 18 bytes, 100/s while changing |
| WeDo 2.0 | 128 bit service `00001523-1212-efde-1523-785feabcd123` + name | control service `00004f0e-...`: output `...1565`, input `...1563`, sensor `...1560` (notify); Device Information `180A` / `2A26` | with response, 4 bytes per motor port (`01 01 01 00` port 1, `02 01 01 00` port 2) |

## S: connection spike (decides `max_conns`)

The limit could not be measured while the firmware was written. The firmware therefore ships with the value that the
controller limits allow on paper and the spike below confirms or corrects it. Derivation: the ESP32-S3 controller
shares `CONFIG_BT_CTRL_BLE_MAX_ACT` (maximum 10) between advertising sets, the scan and the links; 5 sets (each uses one
activity) + 1 scan leave **4 links** (`BC2ADV_MAX_CONNS` = 4, `CONFIG_BT_NIMBLE_MAX_CONNECTIONS` = 4). Memory is not the
limit: static DRAM use is 108 KB of 342 KB and each activity costs about 0.8 KB in the controller.

| # | Precondition | Action | Expected result | Result |
|---|--------------|--------|-----------------|--------|
| S1 | Firmware 0.2.0 flashed; the 3 hubs + extra peripherals (phones with nRF Connect as GATT server, or a second ESP32) switched on and connectable | `bc2adv connspike AA:..:01=<write handle>,<cccd> AA:..:02=<h> AA:..:03=<h> AA:..:04=<h>` (peer list in the order they should be added; the write handle is a write-no-response characteristic, the CCCD handle is optional) | The tool starts 5 advertising sets (20 ms), an active scan, then connects one more peer at a time. After every new link it drives all links with 100 writes/s each (write without response, `--rate`, `--size`) and watches them for `--settle` seconds (default 30). It prints `RESULT: highest stable link count = N` and a line per link count. Needs `hello` first to get the addresses (`bc2adv scan`) | |
| S2 | S1 reached the configured maximum | Same command with `--hold 600` (10 minutes) | No supervision timeout, no `BUSY`/errors, the free heap shown per step stays high (`min_free_heap` > 20 KB) | |
| S3 | S2 passed | Try one more link: rebuild with `BC2ADV_MAX_CONNS` and `CONFIG_BT_NIMBLE_MAX_CONNECTIONS` raised by one (`idf.py menuconfig`), connect N+1 peers | Records the first failing count and its failure mode (connect error, supervision timeout, writes `BUSY`). If N+1 is stable, keep raising | |
| S4 | S1 | Repeat S1 with the scan stopped after the links are up (`gatt-shell`: `scan 3` before, no scan during) | Whether the scan costs a link (document it). Also whether a connect attempt works while the scan runs: the tool's `connect` during `scan` must succeed; otherwise the stick pauses the scan during the attempt (automatic fallback in the adapter) | |
| S5 | One hub without MTU support (WeDo 2.0) | `bc2adv connect <wedo addr> --discover` | `connected: mtu 23` within 3 s; no connect failure | |

Result handling: set `BC2ADV_MAX_CONNS` (and `CONFIG_BT_NIMBLE_MAX_CONNECTIONS`, `CONFIG_BT_CTRL_BLE_MAX_ACT`) in
`sdkconfig.defaults` to the measured stable maximum and record numbers in `hardware.md` (section "Measured
limits"). The host never hard-codes the value: it reads `max_conns` from HELLO_RSP.

## G: GATT checks

| # | Precondition | Action | Expected result | Result |
|---|--------------|--------|-----------------|--------|
| G1 | Hubs switched on | `bc2adv scan --seconds 8` (active scan); also with `--duplicates` and `--refresh 500` | DUPLO hub: manufacturer id `0x0397` and a **name** (from the scan response), WeDo 2.0: service `00001523-...` and a name, MK DIY: manufacturer id `0xAC33`. Unchanged advertisers are not repeated more often than every second; in a crowded place `dropped reports` is counted | |
| G2 | G1 | `bc2adv connect <addr> --discover --hold 300` for each hub; DUPLO also with `--background` (hub switched on **after** the command; cancel by `gatt-shell`: `connect 0 <addr> bg`, `disconnect 0`); `--addr-type auto` (default) | Every hub connects; WeDo 2.0 reports `mtu 23` (no MTU support expected) in under 3 s; the DUPLO background connect succeeds when the hub is switched on later; the links stay up for 5 minutes idle | |
| G3 | G2 | `bc2adv discover <addr>` per hub, compare with nRF Connect | DUPLO `1623`/`1624` (CCCD found), MK DIY `ae3a`/`ae3b`, WeDo 2.0 `4f0e` with `1560` (notify, CCCD found), `1563`, `1565` and `180A` with `2A26`. Handles identical to nRF Connect | |
| G4 | G3 | DUPLO: `gatt-shell`: `connect 0 <addr>`, `write 0 <H> 0800810011510000` (motor off), then a drive frame; hub property request write + `read 0 <H>`. MK DIY: `gatt-bench <addr> <H> --size 18 --rate 100 --seconds 10`. WeDo 2.0: `write 0 <1565> 01010100` / `02010100`, `read 0 <2A26>`, input format 11 byte write. ATT error: `write 0 <2A26> 00` | Motors react; with-response writes are acknowledged after the peer's response; READ returns the value; the write to the read-only `2A26` answers `ATT_ERROR` (extra = ATT code) | |
| G5 | G3 | DUPLO: `gatt-shell`: `notify 0 <1624 CCCD>` right after connecting; WeDo 2.0: `notify 0 <1560 CCCD>`; load case with nRF Connect / ESP32 peer at 100 Hz | The DUPLO hub's attached-IO notifications arrive **within 1 s** of SUBSCRIBE (the ACK comes first, the first notification right behind it); WeDo voltage notifications arrive; order is kept, no loss at 100 Hz per link (`dropped` stays 0) | |
| G6 | S1 done | The three hubs + extra peripherals up to `max_conns`, 5 advertising sets (`bc2adv shell`), scan running, hubs driven at their BC2 rates | Stable for 10 minutes; `RESULT` of `connspike` equals `max_conns` of HELLO_RSP; the next CONNECT answers `NO_RESOURCES` | |
| G7 | G2 | MK4.0 advertising (`bc2adv hold --set 0 ...` with a real MK4 telegram) while DUPLO, MK DIY and WeDo 2.0 links are active and the scan runs | The MK4.0 keeps reacting without extra delay; the links stay up. This is the key concurrency case | |
| G8 | G2 with 2 links | Kill the tool (`gatt-shell`, Ctrl-C twice / kill the process) | All links are dropped within about 1.2 s (`keepalive_ms` + 0.2 s), the advertising sets are silent; observe each hub afterwards (see G10) | |
| G9 | G2 with 2 links + scan | Press the button | Links, scan and sets stop at once, LED red then green, the session stays (`gatt-shell`: `status` answers); connecting again works | |
| G10 | G4 | (a) **Without** failsafe: drive each hub, then kill the tool; note whether the motor stops by itself and after how long. (b) **With** failsafe: DUPLO `failsafe 0 0 <H> 0800810011510000`; MK DIY `failsafe 0 0 <H> CCAABB018080808080808080808080806633` (the 18 byte neutral frame); WeDo 2.0 `failsafe 0 0 <1565> 01010100` and `failsafe 0 1 <1565> 02010100`; then trigger keep-alive timeout, button and HELLO (`hello` from a second tool run), and a plain `disconnect 0` | (a) fill the table "hub behaviour on disconnect" in `hardware.md`; (b) the motor stops at/just before the disconnect on timeout, button and HELLO, but **not** on a normal `disconnect` | |
| G11 | G2 | Unplug and replug the stick while links are up; run `hello`, reconnect | Clean state after HELLO (`gatt-status` shows no links, no scan); hubs can be reconnected | |
| G12 | G2 | `gatt-bench <addr> <H> --response` (with response) and without, for each hub; also with 1 and with `max_conns` links, 5 advertising sets running | Numbers recorded in `hardware.md`: ACK latency of WRITE_CMD (target < 5 ms) and WRITE (about 1-2 connection intervals), notification latency, writes per second per hub | |

Reminder: the hubs must be put into a safe state before the checks (wheels free, nothing connected to the motors) because
motors may keep running after the link drops (G10).
