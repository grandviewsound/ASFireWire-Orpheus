# Codex Handoff — 2026-04-19

## Update — 2026-04-22 (async ACK / timeout rewrite)

This section supersedes the old timeout theory for the current attach-time CMP failure. The short version is:

- The remaining attach failure in fix 69 was **not** best explained by "PCR locks just need a longer timeout."
- Apple reverse engineering now shows the more important problem was our **ACK/state-machine shape** for compare-swap / lock transactions.
- The rewrite below fixes that shape first, keeps the Apple-backed AR completion model for locks, and deliberately does **not** rush into a fake "125 ms Apple clone" before the retry engine actually matches Apple's command re-exec behavior.

### 0. Why the rewrite happened

Fix 69 proved the sample-rate / format-discovery path is working:

- `console log fix 69.txt:1739` — `AVCDiscovery: Cached raw 48k format blocks playback=19 capture=17`
- `console log fix 69.txt:1762` — `BeBoBProtocol ... source=discovered-raw`
- `console log fix 69.txt:1861` / `1876` — driver publishes 6 sample rates / 6 stream formats

But attach still failed when bringing up the input PCR:

- `console log fix 69.txt:1956` — `[Async] OnTimeout: tLabel=37 state=AwaitingAR ackCode=0x0 retries=0`
- `console log fix 69.txt:1958` — `Lock PCR 0xF0000984 failed: status=timeout(1)`
- `console log fix 69.txt:1959` — `CMP ConnectIPCR failed`
- `console log fix 69.txt:1967` — `OnARResponse: No transaction for key`
- `console log fix 69.txt:2034` — later `StartStreaming fallback — pipeline not live`

That pattern matters:

- the transaction timed out while already in `AwaitingAR`
- its stored `ackCode` was still `0x0`
- then the AR arrived **after** timeout, but there was no matching transaction left to complete

So the key question became: did Apple treat lock/CAS as "ignore AT completion entirely and wait only for AR", or does Apple still process `gotAck()` first and only *complete* on the packet path?

### 1. Apple findings that changed the diagnosis

#### 1.1 `IOFireWireFamily` — timeout and retry model

From `IOFireWireFamily.i64`:

- `IOFWAsyncCommand::initAll(...)` sets `fTimeout = 125000`
- `IOFWCommand::startExecution()` calls `updateTimer()` before `execute()`
- `IOFWCommand::updateTimer()` inserts the command into the timeout queue using `fTimeout`
- `IOFWAsyncCommand::complete(...)` retries on timeout by calling `startExecution()` again

Interpretation:

- Apple uses a **short per-attempt timeout with retries**, not one long monolithic deadline.
- But this alone did **not** prove we should simply replace our local timeout with `125 ms`.

#### 1.2 `IOFWCompareAndSwapCommand` — lock/CAS really completes on packet path

Also from `IOFireWireFamily.i64`:

- `IOFWCompareAndSwapCommand::execute(...)` is the real async lock path
- `IOFWCompareAndSwapCommand::gotPacket(...)` completes the compare-swap command

This confirmed one important boundary:

- compare-swap / PCR locks should remain **AR-completing**
- we should **not** rewrite them into AT-completing transactions

#### 1.3 `IOFWAsyncCommand::gotAck(...)` — our ACK mapping was wrong

The decisive finding came from `IOFWAsyncCommand::gotAck(...)`:

- Apple stores the ACK code first
- for `ackCode == 1`, Apple drives `gotPacket(0, 0, 0)` — effectively "ack_complete"
- for `ackCode == 2`, Apple returns without completion — effectively "ack_pending"
- `ackCode == 4/5/6` are the busy family
- other ACKs fall into the immediate-failure mapping

Cross-checks:

- `IOFWAsyncStreamCommand::gotAck(...)` also treats `ackCode == 1` as success
- `IOFWAsyncPHYCommand::gotAck(...)` does the same

So the real Apple-backed ACK interpretation is:

- `0x1 = ack_complete`
- `0x2 = ack_pending`
- `0x4/0x5/0x6 = ack_busy_X/A/B`

Our local code had effectively been treating:

- `0x0` as complete
- `0x1` as pending

That mismatch made the timeout logic and lock state machine much less Apple-like than the raw timeout constant itself.

#### 1.4 `AppleFWOHCI` — timeout poke exists, but not for our PCR path

From `AppleFWOHCI.i64`:

- `AppleFWOHCI::handleAsyncTimeout(...)` injects a detached side-band async lock-style poke
- it only does so when:
  - `addrHi == 0xFFFF`
  - `addrLo <= 0xF00007FF`
- the helper uses the normal async lock transmit path (`AppleFWOHCI::asyncLock(...)` → `AppleFWOHCI_AsyncTransmitRequest::asyncLock(...)`)
- the poke is detached from the original command object

Important consequence:

- Apple's timeout poke **does not directly apply** to our PCR lock addresses like `0xF0000984`
- so the earlier local `1000 ms` PCR special case was not actually justified by this AppleFWOHCI evidence

### 2. Revised root cause

The best current explanation for fix 69 is:

1. Lock / compare-swap transactions were still modeled as "complete on AR"
2. but our implementation treated that too bluntly and effectively skipped real AT / `gotAck` processing for them
3. therefore their stored ACK stayed at the default `0x0`
4. timeout recovery logic saw `AwaitingAR + ackCode=0x0`, timed the transaction out, and removed it
5. the AR then arrived late and hit `No transaction for key`

So the state-machine bug was more fundamental than the timeout length.

### 3. Code changes made in this rewrite

#### 3.1 `ASFWDriver/Async/Core/CompletionStrategy.hpp`

Reason:

- Apple's AR-completing transactions still process `gotAck()`.
- "Complete on AR" does **not** mean "ignore AT completion."

Changes:

- corrected strategy comments to Apple-backed ACK values:
  - write quadlet `ack_complete = 0x1`
  - write block `ack_pending = 0x2`
- changed `ProcessesATCompletion(...)` so `CompleteOnAR` participates in AT completion processing

#### 3.2 `ASFWDriver/Async/Core/Transaction.hpp`

Reason:

- document the new intended behavior clearly so later work does not regress back to "skip AT for all CompleteOnAR".

Changes:

- updated `skipATCompletion_` comment to:
  - read-only fast path only
  - locks still process `gotAck`

#### 3.3 `ASFWDriver/Async/Track/Tracking.hpp`

Reason:

- reads can still bypass the AT handler safely
- locks cannot, because they need the real ACK code stored before waiting for AR

Changes:

- `TxMetadata` now carries `timeoutMs`
- transaction registration now uses:
  - explicit per-transaction timeout when provided
  - otherwise `500 ms` default
- only **read operations** now set `skipATCompletion`
- `OnTxPosted(...)` only fast-paths to `AwaitingAR` for those read-only bypass cases

#### 3.4 `ASFWDriver/Async/Track/TransactionCompletionHandler.hpp`

Reason:

- this is where the actual Apple-shape mismatch lived

Changes:

- corrected ACK documentation and constants:
  - `kAckComplete = 0x1`
  - `kAckPending  = 0x2`
  - `kAckBusyX/A/B = 0x4/0x5/0x6`
- hardware timeout path now treats `ack_pending (0x2)` as "AT succeeded, still awaiting AR"
- `needsARData` now keys off `RequiresARResponse(...)`, not the old narrow check
- `OnATCompletion(...)` logic now matches Apple evidence much more closely:
  - `0x2` → `AwaitingAR`
  - `0x1` → complete immediately only if no AR data is required
  - `0x4/0x5/0x6` → stay alive and extend deadline
  - `0x0`, `0x3`, `0x7`, `0xD`, `0xE`, `0xF`, and unexpected ACKs → fail
- removed the old speculative "tardy / unknown ACK should wait for AR" behavior
- timeout recovery now extends deadlines using the transaction's own timeout budget instead of ad-hoc `200 ms` / `250 ms` constants

Important nuance:

- this is still an **Apple-style approximation**
- Apple re-executes the same command object on timeout
- our current code keeps the transaction alive in place
- so I intentionally did **not** force the timeout base down to Apple's `125000` until the retry engine itself is more faithful

#### 3.5 `ASFWDriver/Async/FireWireBusImpl.cpp`

No new diff in this pass, but verified:

- the temporary `1000 ms` PCR special-case path is no longer part of the current implementation
- that is the correct direction given the AppleFWOHCI findings above

### 4. What did **not** get rewritten yet

This was a deliberate scope decision.

I did **not** yet:

- implement Apple-style same-command replay on timeout
- lower the base async timeout to Apple's `125000`
- inject an AppleFWOHCI-like detached timeout poke for PCR locks

Reasons:

- the proven bug was the wrong ACK/state-machine model
- the AppleFWOHCI poke range does not cover our PCR address
- a fake `125 ms` timeout without Apple-like re-exec semantics would be a risky half-clone

### 5. Build result

Built successfully on 2026-04-22:

```bash
xcodebuild -project 'ASFW.xcodeproj' \
  -scheme ASFWDriver \
  -configuration Debug \
  -derivedDataPath /tmp/ASFWDriverDerived \
  CODE_SIGNING_ALLOWED=NO build
```

Result:

- `BUILD SUCCEEDED`

Note:

- Xcode still emitted its usual local simulator / provisioning noise, but the driver target itself compiled cleanly.

### 6. What the next hardware test should prove

The next run is mainly a state-machine validation, not a format-discovery validation.

Expected improvements:

- the attach-time lock transaction should no longer sit in `AwaitingAR` with `ackCode=0x0`
- if the device returns `ack_pending`, we should preserve that and keep the transaction alive appropriately
- the late AR should complete the original transaction instead of hitting `No transaction for key`
- ideally:
  - no `Lock PCR 0xF0000984 failed: status=timeout`
  - no `CMP ConnectIPCR failed`
  - no `StartStreaming fallback — pipeline not live`

If it still fails, the most important log lines to check are:

- `OnATCompletion` for the lock transaction's actual ACK code
- whether it transitions to `ATCompleted` / `AwaitingAR` with `ackCode=0x2`
- whether timeout now happens with a nonzero Apple-shaped ACK code
- whether the AR still arrives after removal or now completes successfully

### 7. Best current interpretation

As of this rewrite, the most honest summary is:

- sample-rate / format discovery is no longer the blocker
- the current blocker was the async transaction state machine for lock/CAS
- the driver had become less Apple-like by:
  - misreading ACK values
  - skipping AT processing for locks
- this rewrite fixes that first
- the remaining deeper Apple-clone work, if needed, is:
  - true same-command timeout replay
  - timeout-queue behavior closer to `IOFWAsyncCommand`
  - MIDI publication / attach-time auto-live follow-through once CMP lock stability is confirmed

---

Kevin is away; this doc is the working context for Codex to continue from. It captures what changed since Friday (Apr 17), where fix 67 stands after today's hardware test, and what to tackle next.

Branch: `orpheus-dev`. Last pushed commit: `0c1771e`. Fix 67 work is **uncommitted** on disk (see §3).

---

## 1. Commit `0c1771e` — "Fixes 60-66: descriptor reader, plug counts, FCP diagnostics, AR WAKE" (Apr 19)

Single squash covering fixes 60–66. 61 files, +3447 / -607.

| Fix | What it does | Where |
|-----|--------------|-------|
| **60** | `ReadDescriptorChunked` dataStart 9→10. Off-by-one was corrupting every descriptor payload parse. | `Protocols/AVC/AVCUnit.cpp` |
| **61** | Multi-chunk reader now reads declared descriptor length from payload `[0..1]` instead of per-chunk `data_length`. Fixes 132-byte truncation of Music Status Descriptor (actual 2453B). | `Protocols/AVC/AVCUnit.cpp` |
| **62** | `AudioSubunit::LoadFromDiscovery` + `MusicSubunit::LoadFromDiscovery` now call `SetPlugCounts` on base class so UI shows Audio 11/6 and Music 8/9 instead of 0/0. | `Protocols/AVC/Audio/AudioSubunit.cpp`, `Protocols/AVC/Music/MusicSubunit.cpp` |
| **63** | FCP-timeout diagnostic: `FCPTransport::OnCommandTimeout` dumps ctype/subunit/opcode/operands so we can tell which exact cmd is stalling. | `Protocols/AVC/FCPTransport.cpp` |
| **64** | FCP response-drop diagnostic pass — unconditional V1 logs in response router + ValidateResponse drop paths (matcher, parser, post-timeout). | `Protocols/AVC/FCPResponseRouter.hpp`, `FCPTransport.cpp` |
| **65** | AR tCode-drop diagnostic — V0 byte dump on drop + ring state on every `Dequeue`. Localized Phase 5 timeout to parser dropping a buffer at tCode=0xF. | `Async/Rx/ARPacketParser.cpp` |
| **66** | **Structural:** WAKE bit write on AR auto-recycle. `FilledBufferInfo` gained `autoRecycledPrev`; `ARContextBase::Dequeue` now writes `kContextControlWakeBit` under IOLock when set. Without this, controller pauses at buffer boundary. | `Async/Contexts/ARContextBase.hpp`, `Shared/Rings/BufferRing.cpp/.hpp`, `Shared/SharedDataModels.hpp` |

Also in this commit (not numbered fixes, but material):
- `Protocols/AVC/AppleDiscoverySequence.cpp/.hpp` (+1032 lines) — byte-level Apple discovery replay harness.
- `Protocols/AVC/Util/AVCExtendedStreamFormatCommand.hpp` (+173) — typed 0xBF/0xC0 ExtStreamFormat builder.
- `Protocols/Audio/BeBoB/BeBoBProtocol.cpp` (+395/-…) — substantial refactor.
- Tests: +`FCPTransportResponseMatchingTests.cpp`, +44 lines in `BufferRingDMATests.cpp`.

Build status at time of commit: Xcode clean; ctest 447/452 passing (5 pre-existing failures in SelfID enumerator + ConfigROM CRC, unrelated).

---

## 2. Fix 66 hardware test (Apr 19 08:41) — WAKE engaged, Phase 5/11 still stall

WAKE fix fired 2× as designed. Still stalled. Dropped AR buffer showed next-sequential FCP response at **offset 12** inside the buffer → packets straddle the AR buffer boundary, auto-recycle lost the 4-byte Q0, parser saw tCode=0xF and dropped. WAKE was necessary but not sufficient — real bug is AR descriptor sizing / Z / straddle handling.

That finding is what fix 67 attempts to address.

---

## 3. Fix 67 — "AR straddle-scratch" (uncommitted on disk)

### What it does
- Adds Apple-shape **8 KB per-ring straddle scratch** buffer, attached to each AR ring at provision time.
- On auto-recycle, `BufferRing::Dequeue` now copies `[old_tail | new_head]` into scratch **before** resetting the old descriptor, then returns the scratch pointer as the `FilledBufferInfo` so the parser sees a logically contiguous range.
- Removes fix 59's `0xFFFFFFFF` padding-skip in `ARPacketParser` — theory was that once straddles are preserved, no legitimate packets look like padding.

### Files touched (uncommitted)
```
ASFWDriver/Async/Engine/ContextManager.cpp   +17   (allocates arReqScratch / arRspScratch in State PIMPL)
ASFWDriver/Async/Rx/ARPacketParser.cpp       +1 / -38   (removes padding-skip)
ASFWDriver/Shared/Rings/BufferRing.cpp       +97 / -13  (straddle-scratch path + AttachScratch)
ASFWDriver/Shared/Rings/BufferRing.hpp       +14       (scratch_ span + AttachScratch API)
tests/BufferRingDMATests.cpp                 +121      (6 new straddle-scratch tests — all green)
tests/AsyncPacketSerDesLinuxCompatTests.cpp  ±41       (padding-skip removal follow-through)
```

### Hardware test (14:03–14:06) — **DRIVER CRASHED 20+ TIMES**
Log: `logs/console log fix 67.txt` (31358 lines). Crash report: `/Library/Logs/DiagnosticReports/com.kevinpeters.ASFW.ASFWDriver-2026-04-19-140400.ips`.

- Pattern: driver alive ~8s, crashes, relaunches, repeats.
- First straddle (28B: `old_tail=4 new_head=24`) **succeeds**.
- Second straddle (40B: `old_tail=20 new_head=20`) **crashes**, inside the scratch copy block.
- Exception: `EXC_BAD_ACCESS / SIGBUS / EXC_ARM_DA_ALIGN`.
- Faulting frame: `_platform_memmove + 420` ← `BufferRing::Dequeue` ← `ARContextBase::Dequeue` ← `RxPath::ProcessARInterrupts`.
- Register `x[9]=4140` = `last_dequeued_bytes_` at crash.

### Root cause (confirmed)
`std::memcpy` compiles to `_platform_memmove`, which on arm64 uses NEON 16-byte vector instructions (`LDP Q0,Q1,[src]`). AR buffers are mapped `kIOMemoryMapCacheModeInhibit` (Device memory type). Device memory **rejects unaligned vector access** — and refuses all NEON loads regardless on some arm64 variants. ESR: "Data Abort byte read Alignment fault".

- First call: `copyOld=4` → memcpy scalar tail path → OK.
- Second call: `copyOld=20` from offset 4140 (4-aligned, not 16-aligned) → memcpy vector path → fault.

Same class of restriction that `DMAMemoryManager.cpp:305-307` already acknowledges for the write side:
```cpp
// Cache-inhibited mappings reject dc zva; use plain stores via volatile pointer
auto* volatilePtr = reinterpret_cast<volatile uint8_t*>(slabVirt_);
for (size_t i = 0; i < cappedLength; ++i) { volatilePtr[i] = 0; }
```

### Fix for the alignment bug — **APPLIED, not yet hw-tested**
`BufferRing.cpp:162-181` — replaced both `std::memcpy` calls with a `volatile uint8_t*` byte-by-byte loop that mirrors the write-side pattern. This forces scalar loads; no NEON; no alignment fault on cache-inhibited memory.

**To validate:**
1. `./build.sh --test-only --test-filter BufferRing` — all 6 straddle tests should still pass (they don't exercise the cache-inhibited mapping, so they'll pass regardless — but sanity).
2. Full `./build.sh` → install dext → reproduce the attach. Expect: second straddle (40B) no longer crashes; Phase 5/11 either complete, or fail in a new way (see §4).

---

## 4. What Codex should do next (priority order)

### P0 — Confirm the alignment fix works on hardware
The scalar-loop fix in `BufferRing.cpp` is applied but untested on real DMA. Kevin will run the test when back; Codex, don't commit fix 67 until that test passes. If the crash recurs at the same site, check whether the compiler still auto-vectorized the byte loop — if so, add `#pragma clang loop vectorize(disable)` or use `uint32_t` quadlet reads (all AR offsets are quadlet-aligned).

### P1 — Verify fix 59's padding-skip removal is actually safe
Fix 67 removed the `ARPacketParser` padding-skip on the theory that only lost-straddle quadlets ever looked like `0xFFFFFFFF`. Because the driver crashed before the parser ran post-straddle, that theory is **unverified**. Once the alignment fix lands and the straddle path delivers bytes to the parser, check the next hw log:
- No `drop at tCode=0xF` messages = theory confirmed, removal safe.
- If drops return → restore the padding-skip OR make it narrower (only skip quadlets that are a whole-buffer trailer, not mid-stream).

### P2 — Address secondary concerns after straddle is green
From MEMORY.md pending items (still valid):
- **Sample-rate enumeration**: only 1 of 6 rates reaches the CoreAudio nub. Suspect Phase 9 format-enum path.
- **MIDI plug count = 0**: expected 1 each side. Wiring issue between MusicSubunit parse and nub publication.
- **Phase 9 1/32 format-enum timeout**: observed once pre-fix-67; classify transient vs state-dependent once straddle stops masking.

### P3 — Attach-time `BringUpPipeline` dispatch
Fix 58 moved bring-up to attach; fix 63 test showed play-time fallback still firing. Root-cause why `OnAudioConfigurationReady` isn't triggering attach-time bring-up. Not blocking silence, but important for the "click on attach, not play" lifecycle.

---

## 5. Repo state snapshot (for cold-start orientation)

```
Branch: orpheus-dev (ahead of origin by 0 — last pushed 0c1771e)
Uncommitted: fix 67 changes in 7 files (see §3). Do not commit until P0 is validated.
Build: Xcode clean as of 0c1771e; fix 67 edits compiled clean locally, scalar-loop edit just applied.
Hardware: Orpheus attached. Old macOS-11 laptop available as ground truth.
```

Canonical references inside the project:
- `CLAUDE.md` — architecture overview, build commands, critical rules.
- `docs/linux/` — authoritative for OHCI descriptor layout.
- `docs/IOFireWireFamily/` — Apple's original kext source.
- Apple byte-level discovery sequence: see `Protocols/AVC/AppleDiscoverySequence.cpp` (fresh in 0c1771e).

---

## 6. Things to NOT do
- Don't re-introduce `std::memcpy` against DMA-mapped (cache-inhibited) buffers. Use the scalar volatile loop.
- Don't drop WAKE writes on the auto-recycle path (fix 66 is structural, not diagnostic).
- Don't tighten FCP timeouts yet — fix 57's 6s/8s values are obsolete but removing them before the straddle bug is fully fixed will mask it again. Leave them until P0+P1 are green.
- Don't commit the `.xcuserstate` diff; that's user-local.
