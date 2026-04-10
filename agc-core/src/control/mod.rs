pub mod attitude;
pub mod dap;
pub mod imu_control;
pub mod rcs_logic;
pub mod tvc;

pub use attitude::AttitudeError;
pub use dap::{dap_init, dap_step, dap_stop, DapMode, DapState, DAP_PERIOD_CS, DAP_PERIOD_S};
pub use imu_control::{
    apply_pipa_compensation,
    coarse_align_step,
    compute_gyro_drift,
    fine_align_torque,
    is_gimbal_lock_critical,
    is_gimbal_lock_warning,
    refsmmat_from_star_sightings,
    GyroCompensation,
    ImuAlignmentState,
    COARSE_ALIGN_THRESHOLD,
    COLLINEAR_EPSILON,
    FINE_ALIGN_THRESHOLD,
    GYRO_PULSE_RAD,
    T4RUPT_PERIOD_CS,
};
pub use rcs_logic::{
    build_cm_torque_table, build_sm_torque_table,
    compute_pulse_duration, fire_pulse,
    select_jets_cm, select_jets_sm,
    RcsConfig, CM_JET_TORQUES, SM_JET_TORQUES,
};
pub use tvc::{
    tvc_init, tvc_step, update_trim,
    TvcFilter, TvcFilterAxis, TvcState,
    GIMBAL_LIMIT_RAD, K_TRIM, SPS_GIMBAL_SCALE,
    TVC_A0, TVC_A1, TVC_B1,
};
