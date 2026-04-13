//! AGC flight software services: alarm system, initialisation, navigation cycles,
//! and PINBALL verb/noun keyboard state machine.
//!
//! AGC source: Comanche055/ALARM_AND_ABORT.agc, FRESH_START_AND_RESTART.agc,
//!             Comanche055/SERVICER207.agc,
//!             Comanche055/PINBALL_GAME_BUTTONS_AND_LIGHTS.agc,
//!             Comanche055/PINBALL_NOUN_TABLES.agc.

pub mod alarm;
pub mod average_g;
pub mod display;
pub mod fresh_start;
pub mod noun_table;
pub mod pinball;
pub mod v_n;

pub use alarm::{clear_all, most_recent, raise, AlarmCode, AlarmSeverity, AlarmState};
