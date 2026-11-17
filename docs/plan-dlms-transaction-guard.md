# Plan: port the read-flow fixes into the ESPHome component

Goal: bring what was fixed and proven in the Linux library
(`nartis-rf433-v1-linux`, branch `fix/dlms-invoke-id-correlation`, see its
`docs/protocol-flow.md`) into `components/nartis_rf_meter`, together with the
findings from denis-isaev's fork (branch `fix/dlms-response-recovery`, commit
`a7f6226`).

Base: branch `cc1101` (has PR #6 CC1101 backend and PR #7 DLMS misalignment
fix). Work branch: `fix/dlms-transaction-guard`.

## Facts this plan relies on

- The meter echoes the DLMS invoke-id (bench, 2026-11-17: `sent=2 recv=2`,
  `sent=3 recv=3`, and every request in 20+ reads since).
- Priming must run on every wake/read cycle, not once per boot. Without it the
  answer to the first GET of a session can arrive on the next GET.
- After a 0x53 the meter keeps re-sending it until the CIU sends mode-6.
- After a failed read the frame counter must still be persisted, or the next
  read replays counters and the meter starts a re-key.
- About 30 s after a read the meter answers the wake beacon with the last
  0x43 of the previous read (not at 60 s). Pausing 4.5 s and re-beaconing
  fixes it: in the library 30 s reads went from 2/5 to 7/7.
- The meter answers the closing beacon with a repeat of the same data reply,
  not with 0x40. The close step must accept that repeat (same invoke-id and
  count, counters not moved) and never publish it twice.

## Current state of the component (branch cc1101)

| Item | Now | Change needed |
| --- | --- | --- |
| Invoke-id | Hardcoded `0xC1` (dlms_client.cpp:80, :120) | Per-GET 4-bit id |
| Priming | `session_primed_` persists across `update()` (nartis_rf_meter.cpp:86) | Prime every cycle |
| Late reply | PR #7: skip batch + `discard_next_response_` (positional) | Replace with invoke-id match + retry same GET |
| Parser | Accepts with-list only, count checked by caller | Strict: invoke-id, count, IEC length, no trailing bytes |
| 0x53 | Handled in the pair flow with mode-6 | Verify the wake/GET states; add re-key + mode-6 + restart if missing |
| Close beacon | 4.5 s delay (`READ_CLOSE_BEACON_DELAY_MS_`) | Keep |
| 0x43 at wake | Not handled specially | Pause and re-beacon, bounded |
| Counter save after failure | Reported as present | Verify |

## Steps

1. **DLMS client.** Add an `invoke_id` parameter to both request builders
   (`0xC0 | (id & 0x0F)`). Port the strict parser from the library / denis:
   expected invoke-id, expected count, IEC declared length, all results parsed,
   no trailing bytes, normal vs with-list form. Return the response invoke-id so
   it can be logged.
2. **Invoke-id per GET.** `next_get_invoke_id_` advances on each logical GET
   (priming and every batch) and wraps at 16. Re-sends keep the same id and
   APDU. Log `GET invoke: sent=N recv=M match=yes|NO` at DEBUG for every reply.
3. **Config option** `invoke_id_check` (default `true`, since this meter
   echoes). Off = log only, for meters that do not echo the id.
4. **Priming every cycle.** Reset the priming state at the start of each
   read cycle. Keep pairing, key, counters and the NVS layout as they are.
5. **Reject and re-send.** A data reply that fails the strict checks is
   dropped (values cleared) and the same GET is re-sent after 1 s, inside one
   shared retry budget per cycle (denis uses 6). An intermediate 0x5B does not
   reset the budget. When the budget runs out the cycle ends without publishing
   the batch, instead of skipping it and continuing.
6. **Remove the positional discard.** Drop `discard_next_response_` and the
   skip-batch path from PR #7 once step 5 works. Keep the per-OBIS learned
   type/length check as a second line of defence.
7. **Mid-read 0x53.** In wake, priming and GET states: install the key (check
   the serial prefix), reset counters, wait 5629 ms, send mode-6, expect
   0x5B/0x40 (2 attempts), save to NVS, restart the cycle from the wake with a
   new priming GET. At most 2 re-keys per cycle.
8. **0x43 at wake.** Do not count it as a failed wake: log it, wait (match
   denis's `wake_retry_wait_` timing), re-beacon, bounded. Never publish its
   values.
9. **Counter persistence.** Confirm counters are saved to NVS after failed
   cycles; add it if not.
10. **Publishing.** Log `Read cycle complete: N/M` or `incomplete: N/M`.
    A transaction failure publishes nothing from that cycle; a slot-level
    DLMS error (access error, null, unsupported type) leaves only that sensor
    unpublished.
11. **Tests.** Take denis's off-device harness (`tests/`: ESPHome stubs, radio
    stub, `test_transactions.cpp`, `test_rf_integration.cpp`,
    `test_dlms_parser.cpp`, RF vectors) and adapt it to the cc1101 branch and
    the RadioHal interface. Add cases for: mid-read 0x53 + mode-6, 0x43 at wake,
    counter save after failure, invoke-id mismatch then recovery.
12. **Docs.** README: invoke-id option, priming, read spacing guidance, and
    measuring `freq_offset` per CC1101 module (the Pi module needed +14282 Hz;
    at the wrong offset long frames such as 0x53 are lost).

Port by hand rather than merging denis's branch: it is based on a commit from
before PR #6 and PR #7 and touches the same state machine.

## Validation on the bench (device 3, ESP8266 + CC1101)

1. Flash the new build (OTA `--device 192.168.0.9`); pause device 4 meanwhile.
   Measure and set `freq_offset` for this module first.
2. Pair if the stored session is no longer accepted.
3. `update_interval` 120 s for 3 cycles: expect `complete: N/N`, every invoke
   line `match=yes`.
4. 60 s for 5 cycles, then 30 s for 5 cycles. Pass: no wrong value published
   (voltage plausible, energy never decreasing), any stale reply shows
   `match=NO` followed by a successful re-send.
5. 24 h soak at 120 s with DEBUG logs: count complete vs incomplete cycles,
   re-keys and wake retries.
6. Compare with the library results: 120 s 3/3, 60 s 5/5, 30 s 2/5 before the
   wake fix.

## Upstream

- Open a PR from `fix/dlms-transaction-guard` to `main` after `cc1101` is
  merged (or stack it on the cc1101 PR).
- Credit denis-isaev's analysis (per-cycle priming, invoke-id correlation) in
  the PR description, and report the bench result back to him: the meter
  echoes the invoke-id, so his approach works on this meter.

## Open decisions

- `invoke_id_check` default on or off.
- Whether to also publish valid values from a cycle that later fails (denis:
  publish nothing on transaction failure).
- Whether to keep the PR #7 learned type check after the invoke-id guard is in.
