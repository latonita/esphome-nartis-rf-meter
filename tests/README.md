# Host regression tests and ESP32 build

Run from the repository root with a C++17 compiler supporting AddressSanitizer
and UndefinedBehaviorSanitizer:

```sh
sh tests/run.sh
```

The temporary binaries are removed on exit. The runner stops on any failed
assertion or sanitizer error. Leak detection is disabled; address and undefined
behavior checks are enabled. No radio hardware or network is used by host tests.

| Test program | Real code | Replaced boundary | Coverage |
| --- | --- | --- | --- |
| `test_transactions.cpp` | Orchestrator, DLMS | ESPHome runtime, CMT2300A, RF encryption/framing | Repeated cycles with batch sizes 10/5/1; delayed responses; IDs including wrap; no-data/timeout retries; bounded failure and closure; no partial publication on transaction failure; priming and timer wrap |
| `test_rf_integration.cpp` | Orchestrator, DLMS, RF CRC/AES-GCM | ESPHome runtime, physical CMT2300A | Two complete encrypted 16-sensor cycles, numeric/text publication, stale responses, authenticated duplicate closure, CRC/address/MIC failures, replay and non-mutating peek |
| `test_dlms_parser.cpp` | DLMS | Logging | Typed values, access-error/null/compound slot alignment, clock, float, all sample truncations, length/count/ID rejection, 20,000 deterministic mutations |

`rf_vectors.h` contains synthetic frames generated independently with Python's
`cryptography` AES-GCM implementation and a bitwise CRC implementation. The test
key is `00..0F`; addresses are fabricated. The vectors are committed, so Python
packages are not required to run the C++ suite. To regenerate them:

```sh
python -m pip install cryptography
python tests/generate_rf_vectors.py
git diff --exit-code -- tests/rf_vectors.h
```

Compile the component with ESPHome as well:

```sh
python -m venv .venv
.venv/bin/pip install esphome==2026.10.2
.venv/bin/esphome config tests/build-check.yaml
.venv/bin/esphome compile tests/build-check.yaml
```

The fixture has 15 numeric sensors, one Clock text sensor and a re-pair button;
it uses local component sources, ESP32 `esp32dev` and ESP-IDF. It deliberately
contains dummy Wi-Fi credentials and meter serial: **do not flash this config**.

## Validation performed for this change

Host tests passed with GCC, AddressSanitizer and UndefinedBehaviorSanitizer:
17 transaction scenarios, 3 RF/AES integration scenarios, and the parser suite
including 20,000 mutations. Two regressions were first observed failing: priming
was not reset on the next poll, and complete unrelated frames could bypass the
closing wait's timeout. Both pass after the corresponding fixes.

The complete ESP32 build passed with ESPHome 2026.10.2, ESP-IDF 5.5.6 and the
Xtensa GCC 14.2 toolchain. `git diff --check` and fixture regeneration were also
checked. The host compiler reports an existing signedness warning in
`rf_data.cpp`'s encrypted RX size comparison; it is outside the modified paths.

The execution environment needed a temporary build-tool workaround:
`idf_component_manager/prepare_components/cmake_pid.py` raised
`psutil.NoSuchProcess` while discovering its own PID. During compilation only,
that exception used the module's documented `os.getppid()` fallback. The file
was restored afterward. No component code or firmware behavior depends on this
workaround; it is not a required change on a normal ESPHome installation.

These tests do not exercise radio electrical timing, actual RF reception,
flash/NVS endurance or power-loss behavior, Home Assistant integration, or the
meter firmware's response to changing invoke IDs. Hardware acceptance steps and
the investigation are in [the report](../docs/d101-response-recovery.md).
