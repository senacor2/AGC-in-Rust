pub mod p00;
pub mod p01_p02;
pub mod p06;
pub mod p11;
pub mod p15;
pub mod p20;
pub mod p21;
pub mod p22;
pub mod p23;
pub mod p30;
pub mod p31;
pub mod p32;
pub mod p31_p34;
pub mod p37;
pub mod p40_p41;
pub mod p47;
pub mod p51_p52;
pub mod p61_p67;

use crate::executive::job::JobPriority;
use crate::executive::restart::Phase;

/// Interface that every major mode (P-program) must implement.
pub trait MajorMode {
    /// The program number (e.g. 40 for P40).
    fn number(&self) -> u8;

    /// Called when the crew selects this program (V37).
    /// Should establish the program's executive job and return its priority.
    fn start(&self, state: &mut crate::AgcState) -> JobPriority;

    /// Called while this program is active and a verb/noun command arrives.
    fn handle_display_input(&self, state: &mut crate::AgcState, verb: u8, noun: u8);

    /// Called on restart to re-enter the program at the saved phase.
    fn restart_resume(&self, state: &mut crate::AgcState, phase: Phase);

    /// Called when the crew switches away from this program.
    fn terminate(&self, state: &mut crate::AgcState);
}

/// Function signature for a program initialiser in the dispatch table.
pub type ProgramInit = fn(&mut crate::AgcState) -> JobPriority;

/// Major mode dispatch table indexed by program number (0–99).
/// `None` entries are programs not implemented in Comanche055 / out of scope.
pub static PROGRAM_TABLE: [Option<ProgramInit>; 100] = {
    let mut t: [Option<ProgramInit>; 100] = [None; 100];
    t[0]  = Some(p00::init);
    t[1]  = Some(p01_p02::init_p01);
    t[2]  = Some(p01_p02::init_p02);
    t[6]  = Some(p06::init);
    t[11] = Some(p11::init);
    t[15] = Some(p15::init);
    t[20] = Some(p20::init_p20);
    t[21] = Some(p21::p21_init);
    t[22] = Some(p22::p22_init);
    t[23] = Some(p23::init_p23);
    t[30] = Some(p30::init);
    t[31] = Some(p31::init_p31);
    t[32] = Some(p32::init_p32);
    t[33] = Some(p31_p34::init_p33);
    t[34] = Some(p31_p34::init_p34);
    t[37] = Some(p37::init);
    t[40] = Some(p40_p41::init_p40);
    t[41] = Some(p40_p41::init_p41);
    t[47] = Some(p47::init);
    t[51] = Some(p51_p52::init_p51);
    t[52] = Some(p51_p52::init_p52);
    t[61] = Some(p61_p67::init_p61);
    t[62] = Some(p61_p67::init_p62);
    t[63] = Some(p61_p67::init_p63);
    t[64] = Some(p61_p67::init_p64);
    t[67] = Some(p61_p67::init_p67);
    t
};
