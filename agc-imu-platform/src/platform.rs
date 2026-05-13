use crate::quat::UnitQuaternion;
use crate::{CDU_PULSE_RAD, GYRO_PULSE_RAD, PIPA_SCALE};

/// Destructive PIPA read result: pulse counts on three platform-frame axes.
pub struct PipaCounts(pub [i16; 3]);

/// Emulates a stable inertial platform on top of strapdown 6-axis IMU samples.
///
/// The BMI088 (and similar strapdown sensors) delivers angular rates and
/// accelerations in the *body frame*. The AGC expects:
/// - CDU angles (gimbal attitude expressed as three Euler angles)
/// - PIPA counts (delta-V accumulated in the *platform frame*)
/// - gyro torque commands (slew the virtual platform)
///
/// This struct maintains the body→platform rotation quaternion and accumulates
/// delta-V in the platform frame between `read_pipa` calls.
pub struct PlatformEmulator {
    /// Body→platform rotation. IDENTITY = body axes aligned with virtual platform.
    pub attitude: UnitQuaternion,
    /// Fractional PIPA counts per axis, platform frame. Drained by `read_pipa`.
    pub pipa_accum: [f64; 3],
    /// When true, `tick` is a no-op: no integration, no PIPA accumulation.
    pub caged: bool,
    /// Gyro bias subtracted before attitude integration. rad/s.
    pub bias_gyro: [f64; 3],
    /// Accelerometer bias subtracted before PIPA accumulation. m/s².
    pub bias_accel: [f64; 3],
}

impl PlatformEmulator {
    /// Construct in caged state with identity attitude and zero biases.
    pub const fn caged() -> Self {
        Self {
            attitude: UnitQuaternion::IDENTITY,
            pipa_accum: [0.0; 3],
            caged: true,
            bias_gyro: [0.0; 3],
            bias_accel: [0.0; 3],
        }
    }

    pub fn uncage(&mut self, initial_attitude: UnitQuaternion) {
        self.caged = false;
        self.attitude = initial_attitude;
    }

    pub fn set_bias(&mut self, gyro: [f64; 3], accel: [f64; 3]) {
        self.bias_gyro = gyro;
        self.bias_accel = accel;
    }

    /// Integrate one IMU sample.
    ///
    /// When uncaged:
    /// 1. Subtract biases.
    /// 2. Integrate the gyro rates into `attitude` (body→platform quaternion).
    /// 3. Rotate the bias-corrected body-frame acceleration into the platform frame
    ///    and accumulate PIPA counts.
    ///
    /// # Platform-frame acceleration direction
    ///
    /// The BMI088 measures delta-V in the *body frame*. The PIPAs on the real AGC
    /// measure delta-V in the *platform frame*. To emulate that, we apply the
    /// body→platform rotation (`attitude`) to the body-frame acceleration vector:
    ///   accel_platform = attitude.rotate_vec(accel_body_corrected)
    /// This is correct because `attitude` maps body vectors into platform coordinates.
    pub fn tick(&mut self, gyro_body: [f64; 3], accel_body: [f64; 3], dt_s: f64) {
        if self.caged {
            return;
        }

        let gyro_corr = [
            gyro_body[0] - self.bias_gyro[0],
            gyro_body[1] - self.bias_gyro[1],
            gyro_body[2] - self.bias_gyro[2],
        ];
        let accel_corr = [
            accel_body[0] - self.bias_accel[0],
            accel_body[1] - self.bias_accel[1],
            accel_body[2] - self.bias_accel[2],
        ];

        self.attitude = self.attitude.integrate(gyro_corr, dt_s);

        // Body→platform: attitude.rotate_vec maps body-frame vectors to platform frame.
        let accel_platform = self.attitude.rotate_vec(accel_corr);

        self.pipa_accum[0] += accel_platform[0] * dt_s * PIPA_SCALE;
        self.pipa_accum[1] += accel_platform[1] * dt_s * PIPA_SCALE;
        self.pipa_accum[2] += accel_platform[2] * dt_s * PIPA_SCALE;
    }

    /// Destructive read: drain integer PIPA counts from the accumulator.
    ///
    /// Truncates toward zero, subtracts the integer portion, and clamps to i16.
    pub fn read_pipa(&mut self) -> PipaCounts {
        let drain = |acc: &mut f64| -> i16 {
            let counts = *acc as i64;
            let clamped = counts.max(i16::MIN as i64).min(i16::MAX as i64) as i16;
            *acc -= clamped as f64;
            clamped
        };
        PipaCounts([
            drain(&mut self.pipa_accum[0]),
            drain(&mut self.pipa_accum[1]),
            drain(&mut self.pipa_accum[2]),
        ])
    }

    /// Read gimbal CDU angles as wrapping i16 counts ([-180°, +180°)).
    ///
    /// CDU index mapping per specs/imu-control-spec.md §2.1:
    ///   index 0 = outer = roll
    ///   index 1 = inner = pitch
    ///   index 2 = middle = yaw
    pub fn read_cdu(&self) -> [i16; 3] {
        let euler = self.attitude.to_euler_zyx(); // [roll, pitch, yaw]
        let to_cdu = |rad: f64| -> i16 { (rad / CDU_PULSE_RAD) as i32 as i16 };
        [to_cdu(euler[0]), to_cdu(euler[1]), to_cdu(euler[2])]
    }

    /// Apply signed gyro torque pulses about the given body axis.
    ///
    /// Scale: GYRO_PULSE_RAD (B-15 rev) — distinct from CDU_PULSE_RAD (B-1 rev).
    /// axis ∈ {0, 1, 2}; values ≥ 3 are silently ignored (debug_assert guards tests).
    pub fn torque_gyro(&mut self, axis: usize, pulses: i16) {
        debug_assert!(axis < 3, "torque_gyro: axis {} out of range", axis);
        if axis >= 3 {
            return;
        }
        let mut unit_axis = [0.0f64; 3];
        unit_axis[axis] = 1.0;
        let angle = pulses as f64 * GYRO_PULSE_RAD;
        let delta = UnitQuaternion::from_axis_angle(unit_axis, angle);
        self.attitude = self.attitude * delta;
    }

    /// Apply coarse CDU drive commands (same gyro-pulse scale as `torque_gyro`).
    ///
    /// `commands` are signed pulse counts on the three CDU axes (outer/roll,
    /// inner/pitch, middle/yaw), using the B-15 gyro pulse scale, not CDU counts.
    pub fn coarse_align(&mut self, commands: [i16; 3]) {
        let rv = [
            commands[0] as f64 * GYRO_PULSE_RAD,
            commands[1] as f64 * GYRO_PULSE_RAD,
            commands[2] as f64 * GYRO_PULSE_RAD,
        ];
        let delta = UnitQuaternion::from_rotation_vector(rv);
        self.attitude = self.attitude * delta;
    }
}
