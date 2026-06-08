# AGC-in-Rust — Simulator (`agc-sim`)

Build `agc-sim`: a ratatui+crossterm TUI driving `agc-core` with Apollo DSKY keystrokes.

- Binary `agc_sim`, `--scenario {launch,burn,free}`, `F1`/`F2`/`F3` live switch, `q` quit.
- Three panels: DSKY (PROG/VERB/NOUN, R1–R3, lights), Mission State (MET, r/v, SMA/ECC, APO/PER, VG/TGO/ΣΔV), Mission Log.
- Verbs V06/V16/V34/V35/V37; nouns N00/N33/N36/N44/N62/N85 via `agc_core::services::pinball`.
- V37N40 auto-arms a 50 m/s prograde SPS burn; all 6 README scenarios must run.
- Constants locked to Comanche055; `SimHardware` headless-safe; TUI only in the binary; `no_std` core preserved.
- Unit tests per public interface + one test per scenario.
