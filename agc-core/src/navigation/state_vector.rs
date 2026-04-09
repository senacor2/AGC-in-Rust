use crate::types::{Met, Vec3};

/// Coordinate frame in which a state vector is expressed.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Frame {
    /// Earth-centered inertial (used in Earth orbit and cislunar coast).
    EarthInertial,
    /// Moon-centered inertial (used near the Moon).
    MoonInertial,
    /// Stable-member (IMU platform) frame.
    StableMember,
}

/// Position and velocity of a vehicle at a given epoch.
#[derive(Clone, Copy, Debug)]
pub struct StateVector {
    /// Position in metres.
    pub position: Vec3,
    /// Velocity in metres per second.
    pub velocity: Vec3,
    /// Epoch at which this state is valid.
    pub epoch: Met,
    /// Coordinate frame.
    pub frame: Frame,
}

impl StateVector {
    /// A zeroed state vector in the Earth inertial frame at MET = 0.
    pub const ZERO: Self = Self {
        position: [0.0; 3],
        velocity: [0.0; 3],
        epoch: Met(0),
        frame: Frame::EarthInertial,
    };
}
