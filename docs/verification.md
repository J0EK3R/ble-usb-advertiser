# Manual hardware verification (needs the AtomS3U)

No stick was available while the firmware was written, so these steps were **not yet executed**.
Run them when the hardware arrives and fill in the result column. Tool commands are described in
[usage.md](usage.md#host-test-tool); `python tools/bc2adv.py` is called `bc2adv` below. Needs a phone with nRF Connect (or any
BLE scanner showing raw advertising data) and, for item 10, a real toy.

| # | Needs hardware | Precondition | Action | Expected result | Result |
|---|----------------|--------------|--------|-----------------|--------|
| 1 | yes | Firmware built | `idf.py -p <port> flash` (hold the button while plugging in if the port is missing) | LED white flash, then blue blinking (1 Hz). On Windows 10/11 a COM port with VID 303A / PID 1001 appears without driver | |
| 2 | yes | 1 done | `bc2adv ports`, `bc2adv hello` | The stick is listed with the BC2 probe result; hello shows magic, proto 1.0, `max_sets` (expect 5, record the real value), MAC. LED green (session) while connected | |
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

Open questions that only hardware can answer: do the toys need a specific PDU type or address
behaviour; does the controller accept data changes on a running legacy instance (else the fallback path
is used and the latency rises); is the 20 ms interval with 5 simultaneous sets scheduled without gaps.
