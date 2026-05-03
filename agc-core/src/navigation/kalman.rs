//! Shared scalar Kalman measurement-update helper.
//!
//! This module contains the pure-state scalar Kalman update used by both
//! P20 (Rendezvous Navigation) and P22 (Orbital Navigation / Landmark Tracking).
//! Decoupling it from either program avoids a cross-program import dependency.
//!
//! AGC source: Comanche055/MEASUREMENT_INCORPORATION.agc
//! Spec: p20-spec.md §6.5–§6.10 (algorithm); p21_p22-spec.md §6.2 (P22 adaptation)

// ── Types ──────────────────────────────────────────────────────────────────────

/// Outcome of a single scalar Kalman measurement update.
///
/// Returned by `scalar_measurement_update` to communicate whether the mark
/// was accepted or rejected, and whether a W-matrix overflow occurred.
///
/// Used by both P20 and P22; the program-layer wrapper decides which
/// rectify function to call on `AcceptedWOverflow`.
///
/// Spec: p20-spec.md §6.5–§6.10; p21_p22-spec.md §6.2
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum UpdateOutcome {
    /// Mark accepted: `x` and `w` were updated.
    Accepted,
    /// Mark rejected by the 3-sigma gate: `x` and `w` unchanged.
    Rejected,
    /// Accepted, but the W-matrix lost positive-definiteness (a diagonal element
    /// went negative). The caller must call the appropriate rectify function.
    AcceptedWOverflow,
}

// ── Core algorithm ─────────────────────────────────────────────────────────────

/// Pure scalar Kalman measurement update.
///
/// Applies one scalar measurement to a 6-element state vector and its 6×6
/// covariance matrix. Used by both P20 (target state) and P22 (CSM state).
///
/// # Arguments
/// - `x`: 6-element state vector, layout `[pos[0..3], vel[0..3]]`.
///   Mutated on accept.
/// - `w`: 6×6 covariance matrix (symmetric, positive semi-definite).
///   Mutated on accept.
/// - `b`: 6-element measurement sensitivity row (Jacobian H row).
/// - `residual`: scalar measurement residual `z_observed - z_predicted`.
/// - `sigma_sq`: measurement noise variance for this mark type.
///
/// # Returns
/// - `Accepted`          — mark passed the gate; `x` and `w` updated.
/// - `Rejected`          — residual exceeded 3-sigma; `x` and `w` unchanged.
/// - `AcceptedWOverflow` — accepted, but a diagonal entry of `w` went negative.
///   The caller must call the appropriate rectify function.
///
/// # Algorithm
/// Steps 3–10 of the AGC scalar incorporation sequence:
/// 1. Compute `Wb = W * b` (6-vector).
/// 2. Compute innovation variance `S = b^T * Wb + sigma_sq`.
/// 3. Gate: if `|residual| > 3 * sqrt(|S|)`, return `Rejected`.
/// 4. Compute Kalman gain `k = Wb / S`.
/// 5. Update state: `x += k * residual`.
/// 6. Downdate covariance: `W[i][j] -= k[i] * k[j] * S` for all i, j.
/// 7. Positive-definiteness check: if any `W[i][i] < 0`, return `AcceptedWOverflow`.
///
/// Spec: p20-spec.md §6.5–§6.10
pub fn scalar_measurement_update(
    x: &mut [f64; 6],
    w: &mut [[f64; 6]; 6],
    b: [f64; 6],
    residual: f64,
    sigma_sq: f64,
) -> UpdateOutcome {
    // Step 1: Compute W * b (6-element vector).
    let mut wb = [0.0_f64; 6];
    for i in 0..6 {
        for j in 0..6 {
            wb[i] += w[i][j] * b[j];
        }
    }

    // Step 2: Innovation variance S = b^T * W * b + sigma_sq.
    let mut s = sigma_sq;
    for i in 0..6 {
        s += b[i] * wb[i];
    }

    // Step 3: 3-sigma reject gate.
    // Guard against non-finite S (NaN-safety; physically S >= sigma_sq > 0).
    let threshold = 3.0 * libm::sqrt(libm::fabs(s));
    if libm::fabs(residual) > threshold {
        return UpdateOutcome::Rejected;
    }

    // Step 4: Kalman gain k = (W * b) / S.
    let mut k = [0.0_f64; 6];
    for i in 0..6 {
        k[i] = wb[i] / s;
    }

    // Step 5: State update x_new = x_old + k * residual.
    for i in 0..6 {
        x[i] += k[i] * residual;
    }

    // Step 6: Covariance downdate W_new[i][j] = W_old[i][j] - k[i] * k[j] * S.
    for i in 0..6 {
        for j in 0..6 {
            w[i][j] -= k[i] * k[j] * s;
        }
    }

    // Step 7: Positive-definiteness check on diagonal.
    for (i, row) in w.iter().enumerate() {
        if row[i] < 0.0 {
            return UpdateOutcome::AcceptedWOverflow;
        }
    }

    UpdateOutcome::Accepted
}
