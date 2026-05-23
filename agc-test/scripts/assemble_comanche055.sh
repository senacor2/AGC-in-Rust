#!/usr/bin/env bash
# Assemble the Comanche055 AGC source into a yaAGC-loadable rope binary
# plus a text symbol-table listing.
#
# Idempotent: re-runs yaYUL only when one of the .agc sources is newer
# than the assembled binary. Designed to be run once after a fresh clone
# (or after pulling a new yaYUL build); subsequent invocations are no-ops.
#
# Outputs (next to the source):
#   ~/virtualagc/Comanche055/MAIN.agc.bin     — 73 728-byte core rope
#   ~/virtualagc/Comanche055/MAIN.agc.symtab  — yaYUL binary symtab (11 MB)
#   ~/virtualagc/Comanche055/MAIN.agc.lst     — assembly listing + symbol
#                                               table in plain text
#
# Required:
#   VirtualAGC checkout at ~/virtualagc/ with yaYUL built natively
#   (~/virtualagc/yaYUL/yaYUL must be a runnable binary).
#
# CI does NOT need to run this script — it produces developer-only build
# artefacts used by the routine-level VAGC fixture-capture harness in
# `agc-test/src/bin/capture_*`. The committed JSON fixtures are what CI
# actually validates against.

set -euo pipefail

VAGC_ROOT="${VAGC_ROOT:-$HOME/dev/virtualagc}"
YAYUL_BIN="$VAGC_ROOT/yaYUL/yaYUL"
SRC_DIR="$VAGC_ROOT/Comanche055"
MAIN_SRC="$SRC_DIR/MAIN.agc"
ROPE_BIN="$SRC_DIR/MAIN.agc.bin"
SYMTAB_BIN="$SRC_DIR/MAIN.agc.symtab"
LISTING="$SRC_DIR/MAIN.agc.lst"

# ── Pre-flight checks ────────────────────────────────────────────────────────

if [[ ! -x "$YAYUL_BIN" ]]; then
    echo "error: yaYUL not built at $YAYUL_BIN" >&2
    echo "       clone https://github.com/virtualagc/virtualagc and run 'make' in yaYUL/" >&2
    exit 1
fi

if [[ ! -f "$MAIN_SRC" ]]; then
    echo "error: Comanche055 source missing at $MAIN_SRC" >&2
    exit 1
fi

# ── Idempotency check ────────────────────────────────────────────────────────
# Re-assemble only if the rope is missing, empty, or any .agc source in the
# Comanche055 directory is newer than the rope.

rebuild=0
if [[ ! -s "$ROPE_BIN" ]]; then
    rebuild=1
    echo "MAIN.agc.bin missing or empty — assembling."
else
    newest_src="$(find "$SRC_DIR" -name '*.agc' -newer "$ROPE_BIN" -print -quit 2>/dev/null || true)"
    if [[ -n "$newest_src" ]]; then
        rebuild=1
        echo "Source newer than rope: $newest_src — re-assembling."
    fi
fi

if [[ $rebuild -eq 0 ]]; then
    echo "Comanche055 rope up to date at $ROPE_BIN ($(wc -c <"$ROPE_BIN") bytes)."
    exit 0
fi

# ── Assemble ─────────────────────────────────────────────────────────────────

echo "Running yaYUL on $MAIN_SRC ..."

# yaYUL writes <inputfile>.bin next to the input. Run from the Comanche055
# directory so relative includes resolve correctly. The listing goes to
# stdout; capture it to MAIN.agc.lst for later symbol lookup.
(
    cd "$SRC_DIR"
    "$YAYUL_BIN" MAIN.agc >"$LISTING"
) || {
    # yaYUL prints fatal errors to stderr; the listing on disk will have any
    # warnings/info. Surface the tail of the listing on failure.
    echo "error: yaYUL exited non-zero. Tail of $LISTING:" >&2
    tail -20 "$LISTING" >&2
    exit 1
}

# ── Verify the outputs ───────────────────────────────────────────────────────

# Expected: 36864 words × 2 bytes = 73 728 bytes (full AGC rope).
expected_bytes=73728
actual_bytes="$(wc -c <"$ROPE_BIN")"
if [[ "$actual_bytes" -ne "$expected_bytes" ]]; then
    echo "error: rope is $actual_bytes bytes, expected $expected_bytes" >&2
    exit 1
fi

# Verify "0 fatal errors" appears in the listing (yaYUL still produces a .bin
# on warnings; we want to fail loud on fatals).
if ! grep -q '^Fatal errors:  0$' "$LISTING"; then
    echo "error: assembly had fatal errors. Tail of $LISTING:" >&2
    tail -40 "$LISTING" >&2
    exit 1
fi

# Symbol table sanity-check: a handful of entry-guidance landmarks must
# resolve. If yaYUL silently changed its output format the harness would
# fail later in confusing ways; catch it here.
for sym in HUNTEST UPCONTRL PREDICT3 ROLLC LEWD; do
    if ! grep -qE "[[:space:]]$sym[[:space:]]" "$LISTING"; then
        echo "warning: symbol $sym not found in listing (may indicate format change)" >&2
    fi
done

echo "Assembly OK:"
echo "  rope   : $ROPE_BIN ($actual_bytes bytes)"
echo "  symtab : $SYMTAB_BIN ($(wc -c <"$SYMTAB_BIN") bytes)"
echo "  listing: $LISTING ($(wc -l <"$LISTING") lines)"
