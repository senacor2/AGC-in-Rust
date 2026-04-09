//! FRESH START and RESTART sequences.
//!
//! FRESH START: full re-initialisation (power-on or crew-initiated).
//!   Clears all state, establishes P00 idle job.
//!
//! RESTART: recovery after a watchdog, parity, or software restart.
//!   Preserves the navigation state vector, REFSMMAT, and MET.
//!   Re-dispatches active restart groups from their saved phase.
