//! Verb definitions and dispatch table.
//!
//! Each verb is a function that operates on `AgcState` using the current noun.

/// Verb handler signature.
pub type VerbHandler = fn(&mut crate::AgcState, noun: u8);

/// A verb table entry.
pub struct VerbEntry {
    pub handler: VerbHandler,
    pub requires_noun: bool,
}
