# Host regression tests

Run from the repository root with a C++17 compiler that supports
AddressSanitizer and UndefinedBehaviorSanitizer (GCC on Linux or WSL):

```sh
sh tests/run.sh
```

Temporary binaries are removed on exit. The runner stops on the first failed
assertion or sanitizer error. Leak detection is off; address and undefined
behaviour checks are on. No radio hardware or network is used.

| Test program | Real code | Replaced | Coverage |
| --- | --- | --- | --- |
| `test_transactions.cpp` | Orchestrator state machine, DLMS client | ESPHome runtime, radio (`FakeRadio`), RF framing/encryption | Repeated cycles with batch sizes 10/5/1; priming on every cycle; invoke-id per GET incl. wrap; stale same-size reply rejected and the same GET re-sent; polls then re-send on "not ready"; bounded retry budget (a 0x5B cannot reset it); no skip and no publication on transaction failure; counters saved after a failed cycle; close accepts 0x40 or a repeat of the same reply, never stores it twice, bounded deadline; mid-read 0x53 completed with mode-6 and resumed at the same sensor; re-keys bounded per cycle; 0x53 at wake; re-delivered 0x43 at wake paused then recovered; `invoke_id_check: false`; timer wrap |
| `test_rf_integration.cpp` | Orchestrator, DLMS client, RF CRC/AES-GCM | ESPHome runtime, radio (`FakeRadio`) | Two complete encrypted 16-sensor cycles, numeric/text publication, stale responses, authenticated duplicate at close, CRC/address/MIC failures, replay and non-mutating peek |
| `test_dlms_parser.cpp` | DLMS client | Logging | Typed values, access-error/null/compound slot alignment, clock, float, all truncations, length/count/invoke-id rejection, 20,000 deterministic mutations |

`fake_radio.h` implements `RadioHal` in memory: nothing is received through it
(tests place a complete frame in the component's RX buffer and step the state
machine) and every transmitted frame is recorded.

`rf_vectors.h` holds synthetic frames generated independently with Python's
`cryptography` AES-GCM and a bitwise CRC. The key is `00..0F`; the addresses are
made up. The vectors are committed, so Python is not needed to run the tests.
To regenerate them:

```sh
python -m pip install cryptography
python tests/generate_rf_vectors.py
git diff --exit-code -- tests/rf_vectors.h
```

`build-check.yaml` is a compile-only ESP32 + CMT2300A fixture with 15 numeric
sensors, a Clock text sensor and a re-pair button. It has dummy Wi-Fi
credentials and a dummy meter serial: **do not flash it**.

```sh
esphome config tests/build-check.yaml
esphome compile tests/build-check.yaml
```

What these tests do not cover: radio timing and real reception, flash/NVS
wear and power loss, Home Assistant, and how a given meter firmware behaves.
The behaviour modelled here was measured on a Nartis D101 with the Linux
library that shares this protocol (invoke-id echo, priming per cycle, mid-read
0x53 + mode-6, close answered by a repeat of the data, re-delivered 0x43 at the
wake about 30 s after a read).

The harness, the RF vectors and most transaction scenarios come from
denis-isaev's `fix/dlms-response-recovery` branch, adapted to the `RadioHal`
interface.
