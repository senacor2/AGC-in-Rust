use core::ops::Mul;
use libm::{asin, atan2, cos, sin, sqrt};

/// Unit quaternion representing a rotation, stored as (w, x, y, z).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct UnitQuaternion {
    pub w: f64,
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl UnitQuaternion {
    pub const IDENTITY: Self = Self {
        w: 1.0,
        x: 0.0,
        y: 0.0,
        z: 0.0,
    };

    /// `axis` need not be pre-normalised; normalisation is applied inside.
    pub fn from_axis_angle(axis: [f64; 3], angle: f64) -> Self {
        let len = sqrt(axis[0] * axis[0] + axis[1] * axis[1] + axis[2] * axis[2]);
        if len < 1e-12 {
            return Self::IDENTITY;
        }
        let inv = 1.0 / len;
        let half = angle * 0.5;
        let s = sin(half);
        Self {
            w: cos(half),
            x: axis[0] * inv * s,
            y: axis[1] * inv * s,
            z: axis[2] * inv * s,
        }
    }

    /// Build from a rotation vector `rv = ω * dt`.
    ///
    /// Small-angle path (|rv| < 1e-8) avoids division by zero and is accurate
    /// to first order in |rv|.
    pub fn from_rotation_vector(rv: [f64; 3]) -> Self {
        let theta = sqrt(rv[0] * rv[0] + rv[1] * rv[1] + rv[2] * rv[2]);
        if theta < 1e-8 {
            // sin(θ/2)/θ → 0.5 as θ→0
            let h = 0.5;
            return Self::normalise(Self {
                w: 1.0,
                x: rv[0] * h,
                y: rv[1] * h,
                z: rv[2] * h,
            });
        }
        let half = theta * 0.5;
        let s = sin(half) / theta;
        Self {
            w: cos(half),
            x: rv[0] * s,
            y: rv[1] * s,
            z: rv[2] * s,
        }
    }

    /// ZYX Euler convention: yaw (Z) applied first, then pitch (Y), then roll (X).
    /// Equivalent to: q = q_x(roll) · q_y(pitch) · q_z(yaw).
    pub fn from_euler_zyx(roll: f64, pitch: f64, yaw: f64) -> Self {
        let qz = Self::from_axis_angle([0.0, 0.0, 1.0], yaw);
        let qy = Self::from_axis_angle([0.0, 1.0, 0.0], pitch);
        let qx = Self::from_axis_angle([1.0, 0.0, 0.0], roll);
        qx * qy * qz
    }

    /// Conjugate — equivalent to inverse for unit quaternions.
    pub fn inverse(self) -> Self {
        Self {
            w: self.w,
            x: -self.x,
            y: -self.y,
            z: -self.z,
        }
    }

    /// Divide by norm; returns IDENTITY if norm < 1e-12.
    pub fn normalise(self) -> Self {
        let n2 = self.w * self.w + self.x * self.x + self.y * self.y + self.z * self.z;
        if n2 < 1e-24 {
            return Self::IDENTITY;
        }
        let inv = 1.0 / sqrt(n2);
        Self {
            w: self.w * inv,
            x: self.x * inv,
            y: self.y * inv,
            z: self.z * inv,
        }
    }

    /// Rotate vector `v` by this quaternion using the optimised formula:
    ///   t = 2 * cross(q.xyz, v)
    ///   v' = v + q.w * t + cross(q.xyz, t)
    /// Avoids two full Hamilton products compared to the naive sandwich product.
    pub fn rotate_vec(self, v: [f64; 3]) -> [f64; 3] {
        let qx = self.x;
        let qy = self.y;
        let qz = self.z;
        let qw = self.w;

        // t = 2 * (q.xyz × v)
        let tx = 2.0 * (qy * v[2] - qz * v[1]);
        let ty = 2.0 * (qz * v[0] - qx * v[2]);
        let tz = 2.0 * (qx * v[1] - qy * v[0]);

        // v' = v + qw * t + (q.xyz × t)
        [
            v[0] + qw * tx + (qy * tz - qz * ty),
            v[1] + qw * ty + (qz * tx - qx * tz),
            v[2] + qw * tz + (qx * ty - qy * tx),
        ]
    }

    /// Quaternion that rotates unit vector `from` onto unit vector `to`.
    ///
    /// Returns `IDENTITY` when `from` and `to` are already aligned (dot > 1 − 1e-12).
    /// For the antiparallel case (`from` ≈ `−to`), returns a 180° rotation about
    /// the standard basis axis least aligned with `from`, so the result maps
    /// `from` to `to` within 1e-9.
    pub fn from_two_unit_vectors(from: [f64; 3], to: [f64; 3]) -> Self {
        let dot = from[0] * to[0] + from[1] * to[1] + from[2] * to[2];
        if dot > 1.0 - 1e-12 {
            return Self::IDENTITY;
        }
        if dot < -1.0 + 1e-12 {
            // Antiparallel: pick any axis perpendicular to `from`.
            let axis = if from[0].abs() < 0.9 {
                [1.0, 0.0, 0.0]
            } else {
                [0.0, 1.0, 0.0]
            };
            let cross = [
                from[1] * axis[2] - from[2] * axis[1],
                from[2] * axis[0] - from[0] * axis[2],
                from[0] * axis[1] - from[1] * axis[0],
            ];
            return Self::from_axis_angle(cross, core::f64::consts::PI);
        }
        let cross = [
            from[1] * to[2] - from[2] * to[1],
            from[2] * to[0] - from[0] * to[2],
            from[0] * to[1] - from[1] * to[0],
        ];
        let s = sqrt((1.0 + dot) * 2.0);
        Self {
            w: 0.5 * s,
            x: cross[0] / s,
            y: cross[1] / s,
            z: cross[2] / s,
        }
    }

    /// Body-rate integration: `q_new = (q ⊗ from_rotation_vector(ω * dt)).normalise()`.
    pub fn integrate(self, omega: [f64; 3], dt: f64) -> Self {
        let rv = [omega[0] * dt, omega[1] * dt, omega[2] * dt];
        (self * Self::from_rotation_vector(rv)).normalise()
    }

    /// Extract `[roll, pitch, yaw]` in ZYX convention.
    ///
    /// Convention: q = q_x(roll) · q_y(pitch) · q_z(yaw), so yaw is applied
    /// first (about Z), then pitch (about Y), then roll (about X).
    ///
    /// Singularity at pitch = ±π/2 (gimbal lock): yaw is fixed to 0 and the
    /// full rotation is absorbed into roll, which is well-defined and preserves
    /// the rotation faithfully.
    pub fn to_euler_zyx(self) -> [f64; 3] {
        // Pitch is extracted from R[0][2] = 2*(xz+wy) = sin(pitch).
        // This differs from some conventions that use 2*(wy-xz); the correct
        // sign follows from expanding q = qx·qy·qz into a rotation matrix.
        let sinp = (2.0 * (self.x * self.z + self.w * self.y)).clamp(-1.0, 1.0);
        let pitch = asin(sinp);

        // Detect gimbal lock (|sin(pitch)| ≈ 1).
        if (sinp.abs() - 1.0).abs() < 1e-9 {
            // At pitch = ±π/2 the (roll, yaw) pair is degenerate: only the
            // combined angle roll + sign(pitch)*yaw is observable. Set yaw = 0
            // and absorb everything into roll using the non-degenerate elements
            // R[1][0] = 2*(xy+wz) and R[2][0] = 2*(wy-xz).
            let roll = atan2(
                2.0 * (self.x * self.y + self.w * self.z),
                2.0 * (self.w * self.y - self.x * self.z),
            );
            return [roll, pitch, 0.0];
        }

        // roll from R[2][2]=1-2(x²+y²) and sign-corrected roll element 2*(wx-yz).
        let roll = atan2(
            2.0 * (self.w * self.x - self.y * self.z),
            1.0 - 2.0 * (self.x * self.x + self.y * self.y),
        );
        // yaw from R[0][0]=1-2(y²+z²) and R[0][1] element 2*(wz-xy).
        let yaw = atan2(
            2.0 * (self.w * self.z - self.x * self.y),
            1.0 - 2.0 * (self.y * self.y + self.z * self.z),
        );

        [roll, pitch, yaw]
    }
}

/// Hamilton product: `self ⊗ rhs`.
impl Mul for UnitQuaternion {
    type Output = Self;

    fn mul(self, rhs: Self) -> Self {
        Self {
            w: self.w * rhs.w - self.x * rhs.x - self.y * rhs.y - self.z * rhs.z,
            x: self.w * rhs.x + self.x * rhs.w + self.y * rhs.z - self.z * rhs.y,
            y: self.w * rhs.y - self.x * rhs.z + self.y * rhs.w + self.z * rhs.x,
            z: self.w * rhs.z + self.x * rhs.y - self.y * rhs.x + self.z * rhs.w,
        }
    }
}
