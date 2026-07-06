# Investigation: yaAGC debuggability for the test harness (#49 item #5)

**Status**: INVESTIGATION FINDINGS for issue #49 open item #5 ("No single-step
debug capability"). Read-only source study of the VirtualAGC `yaAGC` tree; no code
changes. §5 gives the recommendation.

**Goal**: item #5 asks whether we can *step through* the failing WAITLIST/RESTARTS
chain (and similar) instead of reverse-engineering it from `COREDUMP` snapshots — and
specifically whether "yaAGC GDB/MI integration" is a path to that.

---

## 1. Headline: GDB/MI already exists — that was never the blocker

The #49 re-opening signal ("yaAGC GDB/MI integration becomes available") is **stale**.
yaAGC already ships:

- A built-in **gdb-style debugger** (on by default; the tests pass `--nodebug` to
  disable it). Commands are case-insensitive and gdb-like: `BREAK`, `WATCH`, `LOG`,
  `STEP`/`S`, `NEXT`/`N`, `CONT`/`RUN`, `BACKTRACES`, `INTERRUPTS`, `COREDUMP`,
  memory/register examine, etc.
- A **GDB/MI interface**: `agc_gdbmi.c` / `agc_gdbmi.h`, plus `--fullname`
  ("Output information used by emacs-GDB interface") and DDD GUI-frontend glue
  (`ddd-*.tsv`).
- **Non-interactive scripting**: `--command=FILE` executes debugger commands from a
  file (`FromFiles[]`, `agc_debugger.c:565-574`), so no interactive stdin is needed.
- Non-blocking stdin (`nbfgets`) so the prompt doesn't hard-block the process.

So "make MI available" is not the task. The real obstacle is architectural.

## 2. The real obstacle: a debugger halt freezes the socket-driven harness

yaAGC is single-threaded. The execution loop (`agc_simulator.c:299` `SimExecute`):

```c
while (Simulator.CycleCount < Simulator.DesiredCycles) {
    if (Simulator.Options->debug && DbgExecute()) continue;  // halted → skip engine
    SimExecuteEngine();                                        // agc_engine(): one instr
    SimSetCycleCount(SIM_CYCLECOUNT_INC);
}
```

And the **socket servicing lives inside `agc_engine()`**:

- `agc_engine.c:1943` → `ChannelRoutine(State)` (outgoing: AGC → peripherals)
- `agc_engine.c:1951` → `ChannelInput(State)` (incoming: peripherals → AGC)

Therefore, when a breakpoint/watchpoint halts the CPU (`DbgExecute()` returns true →
`continue`), **`agc_engine()` is never called**, so:

- **Incoming socket writes are never processed** — the harness's DSKY/PIPA/CDU
  connections back up and the DSKY interaction stalls. *This is exactly why our
  keyboard-path (`VBPROC`/`RECALTST`) breakpoints "broke the DSKY socket."*
- **Core dumps stop** (the outer `SimManageTime`/`SimManageCoreDump` isn't reached
  while the inner loop spins on `continue`).

There is also a **speed** problem: with the debugger enabled, `DbgExecute()` runs
every cycle, and real-time pacing means the ~20–30 s of AGC time to pass GAMDIFSW
takes real wall-clock — slow to reach the interesting states.

### A useful nuance (points at the fix)

While *waiting at the prompt*, the debugger's `rfgets` loop **does** call
`ChannelRoutine` (`agc_debugger.c:153`) — but **only the outgoing half**. It does
**not** call `ChannelInput`. So during a halt the DSKY *displays* still update, but
incoming keystrokes are not consumed (the CPU is stopped anyway). The asymmetry is
the lever for a minimal fix (§4 option B).

## 3. Watch/Log also halt

`WATCH` is a watchpoint: `DbgMonitorBreakpoints` sets `BreakFlag`
(`agc_debugger.c:696-714`, "Hit watchpoint …"), which halts via `DbgExecute`. So
watchpoints freeze the socket the same way breakpoints do. Whether a pure
**non-halting `LOG`** mode exists (record-and-continue) is unconfirmed and worth a
5-minute check — if it does, it is the cheapest passive tracer (§4 option C).

## 4. Options to actually make it usable

### A. Scripted breakpoint → auto-print → `CONT` (no yaAGC change)
Drive `--command=FILE` with a (conditional) breakpoint plus state-print commands and
an immediate `CONT`, so each hit freezes only momentarily. Viable for **capturing
state at a chosen PC** (e.g. the WAITLIST overflow site, the P62.1 gate) without
interactive input. **Risk to prototype:** whether the DSKY TCP socket survives the
brief freeze at each hit, and whether the run still reaches the point in acceptable
wall-clock. Lowest effort; best first experiment. Note: to *reach* the breakpoint we
still must drive the AGC over sockets, so this works cleanly only when the breakpoint
fires **after** the DSKY driving is complete (inspection-only), or fires rarely.

### B. Minimal yaAGC patch: service `ChannelInput` during a halt
Add `ChannelInput(State)` next to the existing `ChannelRoutine(State)` in the
debugger's wait loop (`agc_debugger.c:153`) — and/or move both socket calls out of
`agc_engine()` into the outer `SimExecute` loop. Then the socket stays fully alive
while the CPU is halted, making **interactive single-stepping while the harness
drives** possible. This is the robust fix and is a *small, targeted* change. It is a
**yaAGC-side modification** (build our own `yaAGC`), which #49 already contemplates
("embedding yaAGC … with stricter control", "porting yaAGC's restart-handling").
Same class of change as item #3's likely resolution.

### C. Non-halting `LOG` trace (if it exists)
If yaAGC has a record-and-continue `LOG` mode, use it to stream WAITLIST/RESTARTS/
flagword changes to a file during a normal harness-driven run — no halt, no socket
freeze, a time-series instead of point snapshots. Confirm existence first (§3).

## 5. Recommendation

1. **First, prototype option A** — a `--command` script that breaks at the
   WAITLIST/RESTARTS site, prints the relevant erasable, and continues. This needs
   **no yaAGC change** and quickly tells us whether momentary freezes are tolerable
   for our purpose. Verify option C (non-halting `LOG`) in the same sitting.
2. **If interactive stepping-while-driving is required**, do **option B** — the
   `ChannelInput`-during-halt patch. It is the correct architectural fix and is
   small, but it means building a patched `yaAGC` (a yaAGC-side change, like #3).
3. Either way, the existing **`--command=FILE` scripting + GDB/MI** is the delivery
   vehicle; nothing new needs to be built there.

**Bottom line:** item #5 is not blocked by a missing MI — MI and command-scripting
already exist. It is blocked by the single-threaded halt freezing socket I/O. That is
addressable either by living within it (scripted dump-and-continue, option A) or by a
small yaAGC-side patch (option B). Recommend prototyping A before committing to B.
