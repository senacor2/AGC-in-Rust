/// Sequential Events Control System interface — the pyrotechnic discrete
/// channel for parachute deployment and other one-shot CM/SM events.
///
/// In the real flight hardware the SECS receives a one-bit "fire" command
/// from the AGC over a channel-output discrete (channel 14 / bit-specific).
/// The flight software writes the bit; the SECS hardware latches it and
/// fires the pyro. There is no read-back path — the AGC trusts that the
/// command landed, and crew-visible confirmation comes from the FDAI /
/// caution-and-warning panel rather than back through the channel.
///
/// For the simulator, `deploy_drogue` is a one-shot signal that the host
/// graphics layer can display ("drogues out") and that integration tests
/// can assert against. The bare-metal implementation forwards over the
/// HAL link to the actual SECS pyro driver.
pub trait Secs {
    /// Latch the drogue-deploy pyro discrete. Idempotent: calling more
    /// than once is a no-op on hardware once the pyros have fired.
    fn deploy_drogue(&mut self);

    /// Latch the CM/SM separation pyro discrete. Fires the umbilical
    /// guillotines and the spring-loaded pusher that pushes the Service
    /// Module away from the Command Module at the start of the entry
    /// sequence (P62). Idempotent: hardware ignores re-fires once the
    /// pyros have already gone.
    fn fire_csm_separation(&mut self);
}
