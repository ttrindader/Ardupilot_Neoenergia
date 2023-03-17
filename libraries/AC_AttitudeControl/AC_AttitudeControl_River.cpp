#include "AC_AttitudeControl_River.h"
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>

#include <GCS_MAVLink/GCS.h>


// table of user settable parameters
const AP_Param::GroupInfo AC_AttitudeControl_River::var_info[] = {
    // parameters from parent vehicle
    AP_NESTEDGROUPINFO(AC_AttitudeControl, 0),

    // @Param: RAT_RLL_P
    // @DisplayName: Roll axis rate controller P gain
    // @Description: Roll axis rate controller P gain.  Converts the difference between desired roll rate and actual roll rate into a motor speed output
    // @Range: 0.01 0.5
    // @Increment: 0.005
    // @User: Standard

    // @Param: RAT_RLL_I
    // @DisplayName: Roll axis rate controller I gain
    // @Description: Roll axis rate controller I gain.  Corrects long-term difference in desired roll rate vs actual roll rate
    // @Range: 0.01 2.0
    // @Increment: 0.01
    // @User: Standard

    // @Param: RAT_RLL_IMAX
    // @DisplayName: Roll axis rate controller I gain maximum
    // @Description: Roll axis rate controller I gain maximum.  Constrains the maximum motor output that the I gain will output
    // @Range: 0 1
    // @Increment: 0.01
    // @Units: %
    // @User: Standard

    // @Param: RAT_RLL_D
    // @DisplayName: Roll axis rate controller D gain
    // @Description: Roll axis rate controller D gain.  Compensates for short-term change in desired roll rate vs actual roll rate
    // @Range: 0.0 0.05
    // @Increment: 0.001
    // @User: Standard

    // @Param: RAT_RLL_FF
    // @DisplayName: Roll axis rate controller feed forward
    // @Description: Roll axis rate controller feed forward
    // @Range: 0 0.5
    // @Increment: 0.001
    // @User: Standard

    // @Param: RAT_RLL_FLTT
    // @DisplayName: Roll axis rate controller target frequency in Hz
    // @Description: Roll axis rate controller target frequency in Hz
    // @Range: 5 100
    // @Increment: 1
    // @Units: Hz
    // @User: Standard

    // @Param: RAT_RLL_FLTE
    // @DisplayName: Roll axis rate controller error frequency in Hz
    // @Description: Roll axis rate controller error frequency in Hz
    // @Range: 0 100
    // @Increment: 1
    // @Units: Hz
    // @User: Standard

    // @Param: RAT_RLL_FLTD
    // @DisplayName: Roll axis rate controller derivative frequency in Hz
    // @Description: Roll axis rate controller derivative frequency in Hz
    // @Range: 5 100
    // @Increment: 1
    // @Units: Hz
    // @User: Standard
    AP_SUBGROUPINFO(_pid_rate_roll, "RAT_RLL_", 1, AC_AttitudeControl_River, AC_PID),

    // @Param: RAT_PIT_P
    // @DisplayName: Pitch axis rate controller P gain
    // @Description: Pitch axis rate controller P gain.  Converts the difference between desired pitch rate and actual pitch rate into a motor speed output
    // @Range: 0.01 0.50
    // @Increment: 0.005
    // @User: Standard

    // @Param: RAT_PIT_I
    // @DisplayName: Pitch axis rate controller I gain
    // @Description: Pitch axis rate controller I gain.  Corrects long-term difference in desired pitch rate vs actual pitch rate
    // @Range: 0.01 2.0
    // @Increment: 0.01
    // @User: Standard

    // @Param: RAT_PIT_IMAX
    // @DisplayName: Pitch axis rate controller I gain maximum
    // @Description: Pitch axis rate controller I gain maximum.  Constrains the maximum motor output that the I gain will output
    // @Range: 0 1
    // @Increment: 0.01
    // @Units: %
    // @User: Standard

    // @Param: RAT_PIT_D
    // @DisplayName: Pitch axis rate controller D gain
    // @Description: Pitch axis rate controller D gain.  Compensates for short-term change in desired pitch rate vs actual pitch rate
    // @Range: 0.0 0.05
    // @Increment: 0.001
    // @User: Standard

    // @Param: RAT_PIT_FF
    // @DisplayName: Pitch axis rate controller feed forward
    // @Description: Pitch axis rate controller feed forward
    // @Range: 0 0.5
    // @Increment: 0.001
    // @User: Standard

    // @Param: RAT_PIT_FLTT
    // @DisplayName: Pitch axis rate controller target frequency in Hz
    // @Description: Pitch axis rate controller target frequency in Hz
    // @Range: 5 100
    // @Increment: 1
    // @Units: Hz
    // @User: Standard

    // @Param: RAT_PIT_FLTE
    // @DisplayName: Pitch axis rate controller error frequency in Hz
    // @Description: Pitch axis rate controller error frequency in Hz
    // @Range: 0 100
    // @Increment: 1
    // @Units: Hz
    // @User: Standard

    // @Param: RAT_PIT_FLTD
    // @DisplayName: Pitch axis rate controller derivative frequency in Hz
    // @Description: Pitch axis rate controller derivative frequency in Hz
    // @Range: 5 100
    // @Increment: 1
    // @Units: Hz
    // @User: Standard
    AP_SUBGROUPINFO(_pid_rate_pitch, "RAT_PIT_", 2, AC_AttitudeControl_River, AC_PID),

    // @Param: RAT_YAW_P
    // @DisplayName: Yaw axis rate controller P gain
    // @Description: Yaw axis rate controller P gain.  Converts the difference between desired yaw rate and actual yaw rate into a motor speed output
    // @Range: 0.10 2.50
    // @Increment: 0.005
    // @User: Standard

    // @Param: RAT_YAW_I
    // @DisplayName: Yaw axis rate controller I gain
    // @Description: Yaw axis rate controller I gain.  Corrects long-term difference in desired yaw rate vs actual yaw rate
    // @Range: 0.010 1.0
    // @Increment: 0.01
    // @User: Standard

    // @Param: RAT_YAW_IMAX
    // @DisplayName: Yaw axis rate controller I gain maximum
    // @Description: Yaw axis rate controller I gain maximum.  Constrains the maximum motor output that the I gain will output
    // @Range: 0 1
    // @Increment: 0.01
    // @Units: %
    // @User: Standard

    // @Param: RAT_YAW_D
    // @DisplayName: Yaw axis rate controller D gain
    // @Description: Yaw axis rate controller D gain.  Compensates for short-term change in desired yaw rate vs actual yaw rate
    // @Range: 0.000 0.02
    // @Increment: 0.001
    // @User: Standard

    // @Param: RAT_YAW_FF
    // @DisplayName: Yaw axis rate controller feed forward
    // @Description: Yaw axis rate controller feed forward
    // @Range: 0 0.5
    // @Increment: 0.001
    // @User: Standard

    // @Param: RAT_YAW_FLTT
    // @DisplayName: Yaw axis rate controller target frequency in Hz
    // @Description: Yaw axis rate controller target frequency in Hz
    // @Range: 1 50
    // @Increment: 1
    // @Units: Hz
    // @User: Standard

    // @Param: RAT_YAW_FLTE
    // @DisplayName: Yaw axis rate controller error frequency in Hz
    // @Description: Yaw axis rate controller error frequency in Hz
    // @Range: 0 20
    // @Increment: 1
    // @Units: Hz
    // @User: Standard

    // @Param: RAT_YAW_FLTD
    // @DisplayName: Yaw axis rate controller derivative frequency in Hz
    // @Description: Yaw axis rate controller derivative frequency in Hz
    // @Range: 5 50
    // @Increment: 1
    // @Units: Hz
    // @User: Standard
    AP_SUBGROUPINFO(_pid_rate_yaw, "RAT_YAW_", 3, AC_AttitudeControl_River, AC_PID),

    // @Param: THR_MIX_MIN
    // @DisplayName: Throttle Mix Minimum
    // @Description: Throttle vs attitude control prioritisation used when landing (higher values mean we prioritise attitude control over throttle)
    // @Range: 0.1 0.25
    // @User: Advanced
    AP_GROUPINFO("THR_MIX_MIN", 4, AC_AttitudeControl_River, _thr_mix_min, AC_ATTITUDE_CONTROL_MIN_DEFAULT),

    // @Param: THR_MIX_MAX
    // @DisplayName: Throttle Mix Maximum
    // @Description: Throttle vs attitude control prioritisation used during active flight (higher values mean we prioritise attitude control over throttle)
    // @Range: 0.5 0.9
    // @User: Advanced
    AP_GROUPINFO("THR_MIX_MAX", 5, AC_AttitudeControl_River, _thr_mix_max, AC_ATTITUDE_CONTROL_MAX_DEFAULT),

    // @Param: THR_MIX_MAN
    // @DisplayName: Throttle Mix Manual
    // @Description: Throttle vs attitude control prioritisation used during manual flight (higher values mean we prioritise attitude control over throttle)
    // @Range: 0.1 0.9
    // @User: Advanced
    AP_GROUPINFO("THR_MIX_MAN", 6, AC_AttitudeControl_River, _thr_mix_man, AC_ATTITUDE_CONTROL_MAN_DEFAULT),

    AP_GROUPEND
};

AC_AttitudeControl_River::AC_AttitudeControl_River(AP_AHRS_View &ahrs, const AP_Vehicle::MultiCopter &aparm, AP_MotorsRiver& motors, float dt) :
    AC_AttitudeControl(ahrs, aparm, motors, dt),
    _motors_multi(motors),
    _pid_rate_roll(AC_ATC_MULTI_RATE_RP_P, AC_ATC_MULTI_RATE_RP_I, AC_ATC_MULTI_RATE_RP_D, 0.0f, AC_ATC_MULTI_RATE_RP_IMAX, AC_ATC_MULTI_RATE_RP_FILT_HZ, 0.0f, AC_ATC_MULTI_RATE_RP_FILT_HZ, dt),
    _pid_rate_pitch(AC_ATC_MULTI_RATE_RP_P, AC_ATC_MULTI_RATE_RP_I, AC_ATC_MULTI_RATE_RP_D, 0.0f, AC_ATC_MULTI_RATE_RP_IMAX, AC_ATC_MULTI_RATE_RP_FILT_HZ, 0.0f, AC_ATC_MULTI_RATE_RP_FILT_HZ, dt),
    _pid_rate_yaw(AC_ATC_MULTI_RATE_YAW_P, AC_ATC_MULTI_RATE_YAW_I, AC_ATC_MULTI_RATE_YAW_D, 0.0f, AC_ATC_MULTI_RATE_YAW_IMAX, AC_ATC_MULTI_RATE_RP_FILT_HZ, AC_ATC_MULTI_RATE_YAW_FILT_HZ, 0.0f, dt)
{
    AP_Param::setup_object_defaults(this, var_info);
}

// Update Alt_Hold angle maximum
void AC_AttitudeControl_River::update_althold_lean_angle_max(float throttle_in)
{
    // calc maximum tilt angle based on throttle
    float thr_max = _motors_multi.get_throttle_thrust_max();
    // divide by zero check
    if (is_zero(thr_max)) {
        _althold_lean_angle_max = 0.0f;
        return;
    }
    float althold_lean_angle_max = acosf(constrain_float(throttle_in / (AC_ATTITUDE_CONTROL_ANGLE_LIMIT_THROTTLE_MAX * thr_max), 0.0f, 1.0f));
    _althold_lean_angle_max = _althold_lean_angle_max + (_dt / (_dt + _angle_limit_tc)) * (althold_lean_angle_max - _althold_lean_angle_max);
}

void AC_AttitudeControl_River::set_throttle_out(float throttle_in, bool apply_angle_boost, float filter_cutoff)
{
    _throttle_in = throttle_in;
    update_althold_lean_angle_max(throttle_in);
    _motors.set_throttle_filter_cutoff(filter_cutoff);
    if (apply_angle_boost) {
        // Apply angle boost
        throttle_in = get_throttle_boosted(throttle_in);
    } else {
        // Clear angle_boost for logging purposes
        _angle_boost = 0.0f;
    }
    _motors.set_throttle(throttle_in);
    _motors.set_throttle_avg_max(get_throttle_avg_max(MAX(throttle_in, _throttle_in)));
}

void AC_AttitudeControl_River::set_throttle_mix_max(float ratio)
{
    ratio = constrain_float(ratio, 0.0f, 1.0f);
    _throttle_rpy_mix_desired = (1.0f - ratio) * _thr_mix_min + ratio * _thr_mix_max;
}

// returns a throttle including compensation for roll/pitch angle
// throttle value should be 0 ~ 1
float AC_AttitudeControl_River::get_throttle_boosted(float throttle_in)
{
    if (!_angle_boost_enabled) {
        _angle_boost = 0;
        return throttle_in;
    }
    // inverted_factor is 1 for tilt angles below 60 degrees
    // inverted_factor reduces from 1 to 0 for tilt angles between 60 and 90 degrees

    float cos_tilt = _ahrs.cos_pitch() * _ahrs.cos_roll();
    float inverted_factor = constrain_float(2.0f * cos_tilt, 0.0f, 1.0f);
    float boost_factor = 1.0f / constrain_float(cos_tilt, 0.5f, 1.0f);

    float throttle_out = throttle_in * inverted_factor * boost_factor;
    _angle_boost = constrain_float(throttle_out - throttle_in, -1.0f, 1.0f);
    return throttle_out;
}

// returns a throttle including compensation for roll/pitch angle
// throttle value should be 0 ~ 1
float AC_AttitudeControl_River::get_throttle_avg_max(float throttle_in)
{
    throttle_in = constrain_float(throttle_in, 0.0f, 1.0f);
    return MAX(throttle_in, throttle_in * MAX(0.0f, 1.0f - _throttle_rpy_mix) + _motors.get_throttle_hover() * _throttle_rpy_mix);
}

// update_throttle_rpy_mix - slew set_throttle_rpy_mix to requested value
void AC_AttitudeControl_River::update_throttle_rpy_mix()
{
    // slew _throttle_rpy_mix to _throttle_rpy_mix_desired
    if (_throttle_rpy_mix < _throttle_rpy_mix_desired) {
        // increase quickly (i.e. from 0.1 to 0.9 in 0.4 seconds)
        _throttle_rpy_mix += MIN(2.0f * _dt, _throttle_rpy_mix_desired - _throttle_rpy_mix);
    } else if (_throttle_rpy_mix > _throttle_rpy_mix_desired) {
        // reduce more slowly (from 0.9 to 0.1 in 1.6 seconds)
        _throttle_rpy_mix -= MIN(0.5f * _dt, _throttle_rpy_mix - _throttle_rpy_mix_desired);
    }
    _throttle_rpy_mix = constrain_float(_throttle_rpy_mix, 0.1f, AC_ATTITUDE_CONTROL_MAX);
}

void AC_AttitudeControl_River::rate_controller_run(){
 
    // move throttle vs attitude mixing towards desired (called from here because this is conveniently called on every iteration)
    update_throttle_rpy_mix();

    Vector3f gyro_latest = _ahrs.get_gyro_latest();
    _motors.set_roll(get_rate_roll_pid().update_all(_rate_target_ang_vel.x, gyro_latest.x, _motors.limit.roll));
    _motors.set_pitch(get_rate_pitch_pid().update_all(_rate_target_ang_vel.y, gyro_latest.y, _motors.limit.pitch));
    _motors.set_yaw(get_rate_yaw_pid().update_all(_rate_target_ang_vel.z, gyro_latest.z, _motors.limit.yaw));

    _rate_sysid_ang_vel.zero();
    _actuator_sysid.zero();
    
    control_monitor_update();
}


// sanity check parameters.  should be called once before takeoff
void AC_AttitudeControl_River::parameter_sanity_check()
{
    // sanity check throttle mix parameters
    if (_thr_mix_man < 0.1f || _thr_mix_man > 4.0f) {
        // parameter description recommends thr-mix-man be no higher than 0.9 but we allow up to 4.0
        // which can be useful for very high powered copters with very low hover throttle
        _thr_mix_man.set_and_save(AC_ATTITUDE_CONTROL_MAN_DEFAULT);
    }
    if (_thr_mix_min < 0.1f || _thr_mix_min > 0.25f) {
        _thr_mix_min.set_and_save(AC_ATTITUDE_CONTROL_MIN_DEFAULT);
    }
    if (_thr_mix_max < 0.5f || _thr_mix_max > AC_ATTITUDE_CONTROL_MAX) {
        // parameter description recommends thr-mix-max be no higher than 0.9 but we allow up to 5.0
        // which can be useful for very high powered copters with very low hover throttle
        _thr_mix_max.set_and_save(AC_ATTITUDE_CONTROL_MAX_DEFAULT);
    }
    if (_thr_mix_min > _thr_mix_max) {
        _thr_mix_min.set_and_save(AC_ATTITUDE_CONTROL_MIN_DEFAULT);
        _thr_mix_max.set_and_save(AC_ATTITUDE_CONTROL_MAX_DEFAULT);
    }
}

float AC_AttitudeControl_River::map_cube(float x, float y, float z)
{
    float out = 0.0f;
    out = x*sqrtf(1 - powf(y,2)/2.0f - powf(z,2)/2.0f + (powf(y,2)*powf(z,2))/3.0f);
    return out;
}

// static uint8_t counter = 0;

void AC_AttitudeControl_River::output_to_boat(float roll, float pitch)
{
    // Forças Calculadas pelo controlador de posição são calculadas aqui.
    X = -((float)pitch)/lean_angle_max();
    Y =  ((float)roll) /lean_angle_max();

    // Saturação das Forças
    X = constrain_float(X,-1.0f,1.0f);
    Y = constrain_float(Y,-1.0f,1.0f);

    _motors.set_forward(X);
    _motors.set_lateral(Y);
    // _motors.set_yaw(Z);
    // SE DER MERDA FAZER O MAPCUBE AQUI E COLOCAR O TN DE UMA VEZ

}


void AC_AttitudeControl_River::passthrough_servo(float PWM){
    _motors.set_lateral(PWM);
}
  
//  MATHAUS
void AC_AttitudeControl_River::input_rate_stabilize_roll_pitch_yaw(float fx, float fy, float yaw)
{
    // _motors.set_forward(fx);
    // _motors.set_lateral(fy);
    float yaw_rate = radians(yaw * 0.01f)/radians(_ang_vel_yaw_max);

    _motors.set_yaw(yaw_rate);
}

// Command an euler roll and pitch angle and an euler yaw rate with angular velocity feedforward and smoothing
void AC_AttitudeControl_River::input_euler_angle_roll_pitch_euler_rate_yaw(float euler_roll_angle_cd, float euler_pitch_angle_cd, float euler_yaw_rate_cds)
{
    output_to_boat(euler_roll_angle_cd,euler_pitch_angle_cd);

    input_rate_stabilize_roll_pitch_yaw(0.0f,0.0f,euler_yaw_rate_cds); //new
    

    // Convert from centidegrees on public interface to radians
    // float euler_roll_angle = 0*radians(euler_roll_angle_cd * 0.01f);
    // float euler_pitch_angle = 0*radians(euler_pitch_angle_cd * 0.01f);
    float euler_yaw_rate = radians(euler_yaw_rate_cds * 0.01f);

    // calculate the attitude target euler angles
    _attitude_target_quat.to_euler(_attitude_target_euler_angle.x, _attitude_target_euler_angle.y, _attitude_target_euler_angle.z);

    // Add roll trim to compensate tail rotor thrust in heli (will return zero on multirotors)
    // euler_roll_angle += get_roll_trim_rad();

    if (_rate_bf_ff_enabled) {
        // translate the roll pitch and yaw acceleration limits to the euler axis
        Vector3f euler_accel = euler_accel_limit(_attitude_target_euler_angle, Vector3f(get_accel_roll_max_radss(), get_accel_pitch_max_radss(), get_accel_yaw_max_radss()));

        // When acceleration limiting and feedforward are enabled, the sqrt controller is used to compute an euler
        // angular velocity that will cause the euler angle to smoothly stop at the input angle with limited deceleration
        // and an exponential decay specified by smoothing_gain at the end.
        _attitude_target_euler_rate.x = input_shaping_angle(0, _input_tc, euler_accel.x, _attitude_target_euler_rate.x, _dt);
        _attitude_target_euler_rate.y = input_shaping_angle(0, _input_tc, euler_accel.y, _attitude_target_euler_rate.y, _dt);

        // When yaw acceleration limiting is enabled, the yaw input shaper constrains angular acceleration about the yaw axis, slewing
        // the output rate towards the input rate.
        _attitude_target_euler_rate.z = input_shaping_ang_vel(_attitude_target_euler_rate.z, euler_yaw_rate, euler_accel.z, _dt);

        // Convert euler angle derivative of desired attitude into a body-frame angular velocity vector for feedforward
        euler_rate_to_ang_vel(_attitude_target_euler_angle, _attitude_target_euler_rate, _attitude_target_ang_vel);
        // Limit the angular velocity
        ang_vel_limit(_attitude_target_ang_vel, radians(_ang_vel_roll_max), radians(_ang_vel_pitch_max), radians(_ang_vel_yaw_max));
        // Convert body-frame angular velocity into euler angle derivative of desired attitude
        ang_vel_to_euler_rate(_attitude_target_euler_angle, _attitude_target_ang_vel, _attitude_target_euler_rate);
    } else {
        // When feedforward is not enabled, the target euler angle is input into the target and the feedforward rate is zeroed.
        _attitude_target_euler_angle.x = 0.0f;
        _attitude_target_euler_angle.y = 0.0f;
        _attitude_target_euler_angle.z += euler_yaw_rate*_dt;
        // Compute quaternion target attitude
        _attitude_target_quat.from_euler(_attitude_target_euler_angle.x, _attitude_target_euler_angle.y, _attitude_target_euler_angle.z);

        // Set rate feedforward requests to zero
        _attitude_target_euler_rate = Vector3f(0.0f, 0.0f, 0.0f);
        _attitude_target_ang_vel    = Vector3f(0.0f, 0.0f, 0.0f);
    }

    // Call quaternion attitude controller
    attitude_controller_run_quat();
}


// Command an euler roll, pitch and yaw angle with angular velocity feedforward and smoothing
void AC_AttitudeControl_River::input_euler_angle_roll_pitch_yaw(float euler_roll_angle_cd, float euler_pitch_angle_cd, float euler_yaw_angle_cd, bool slew_yaw){

    output_to_boat(euler_roll_angle_cd,euler_pitch_angle_cd); //Mathaus

    // Convert from centidegrees on public interface to radians
    // Mathaus
    float euler_roll_angle  = 0*radians(euler_roll_angle_cd*0.01f);
    float euler_pitch_angle = 0*radians(euler_pitch_angle_cd*0.01f);
    float euler_yaw_angle  = radians(euler_yaw_angle_cd*0.01f);

    // calculate the attitude target euler angles
    _attitude_target_quat.to_euler(_attitude_target_euler_angle.x, _attitude_target_euler_angle.y, _attitude_target_euler_angle.z);
    
    // Add roll trim to compensate tail rotor thrust in heli (will return zero on multirotors)
    euler_roll_angle += get_roll_trim_rad();

    if (_rate_bf_ff_enabled) {
        // translate the roll pitch and yaw acceleration limits to the euler axis
        Vector3f euler_accel = euler_accel_limit(_attitude_target_euler_angle, Vector3f(get_accel_roll_max_radss(), get_accel_pitch_max_radss(), get_accel_yaw_max_radss()));

        // When acceleration limiting and feedforward are enabled, the sqrt controller is used to compute an euler
        // angular velocity that will cause the euler angle to smoothly stop at the input angle with limited deceleration
        // and an exponential decay specified by _input_tc at the end.
        _attitude_target_euler_rate.x = input_shaping_angle(0.0f, _input_tc, euler_accel.x, _attitude_target_euler_rate.x, _dt);
        _attitude_target_euler_rate.y = input_shaping_angle(0.0f, _input_tc, euler_accel.y, _attitude_target_euler_rate.y, _dt);
        _attitude_target_euler_rate.z = input_shaping_angle(wrap_PI(euler_yaw_angle - _attitude_target_euler_angle.z), _input_tc, euler_accel.z, _attitude_target_euler_rate.z, _dt);
        if (slew_yaw) {
            _attitude_target_euler_rate.z = constrain_float(_attitude_target_euler_rate.z, -get_slew_yaw_rads(), get_slew_yaw_rads());
        }

        // Convert euler angle derivative of desired attitude into a body-frame angular velocity vector for feedforward
        euler_rate_to_ang_vel(_attitude_target_euler_angle, _attitude_target_euler_rate, _attitude_target_ang_vel);
        // Limit the angular velocity
        ang_vel_limit(_attitude_target_ang_vel, radians(_ang_vel_roll_max), radians(_ang_vel_pitch_max), radians(_ang_vel_yaw_max));
        // Convert body-frame angular velocity into euler angle derivative of desired attitude
        ang_vel_to_euler_rate(_attitude_target_euler_angle, _attitude_target_ang_vel, _attitude_target_euler_rate);
    } else {
        // When feedforward is not enabled, the target euler angle is input into the target and the feedforward rate is zeroed.
        _attitude_target_euler_angle.x = euler_roll_angle;
        _attitude_target_euler_angle.y = euler_pitch_angle;
        if (slew_yaw) {
            // Compute constrained angle error
            float angle_error = constrain_float(wrap_PI(euler_yaw_angle - _attitude_target_euler_angle.z), -get_slew_yaw_rads() * _dt, get_slew_yaw_rads() * _dt);
            // Update attitude target from constrained angle error
            _attitude_target_euler_angle.z = wrap_PI(angle_error + _attitude_target_euler_angle.z);
        } else {
            _attitude_target_euler_angle.z = euler_yaw_angle;
        }
        // Compute quaternion target attitude
        _attitude_target_quat.from_euler(_attitude_target_euler_angle.x, _attitude_target_euler_angle.y, _attitude_target_euler_angle.z);

        // Set rate feedforward requests to zero
        _attitude_target_euler_rate = Vector3f(0.0f, 0.0f, 0.0f);
        _attitude_target_ang_vel = Vector3f(0.0f, 0.0f, 0.0f);
    }

    // Call quaternion attitude controller
    attitude_controller_run_quat();
}


// // Command an angular velocity with angular velocity feedforward and smoothing
void AC_AttitudeControl_River::input_rate_bf_roll_pitch_yaw(float roll_rate_bf_cds, float pitch_rate_bf_cds, float yaw_rate_bf_cds)
{
    output_to_boat(pitch_rate_bf_cds,roll_rate_bf_cds); //Mathaus

    // Convert from centidegrees on public interface to radians
    float roll_rate_rads  = 0.0f;
    float pitch_rate_rads = 0.0f;
    float yaw_rate_rads   = radians(yaw_rate_bf_cds * 0.01f);

    // calculate the attitude target euler angles
    _attitude_target_quat.to_euler(_attitude_target_euler_angle.x, _attitude_target_euler_angle.y, _attitude_target_euler_angle.z);

    if (_rate_bf_ff_enabled) {
        // Compute acceleration-limited body frame rates
        // When acceleration limiting is enabled, the input shaper constrains angular acceleration about the axis, slewing
        // the output rate towards the input rate.
        _attitude_target_ang_vel.x = input_shaping_ang_vel(_attitude_target_ang_vel.x*0.0f, roll_rate_rads, get_accel_roll_max_radss(), _dt);
        _attitude_target_ang_vel.y = input_shaping_ang_vel(_attitude_target_ang_vel.y*0.0f, pitch_rate_rads, get_accel_pitch_max_radss(), _dt);
        _attitude_target_ang_vel.z = input_shaping_ang_vel(_attitude_target_ang_vel.z, yaw_rate_rads, get_accel_yaw_max_radss(), _dt);

        // Convert body-frame angular velocity into euler angle derivative of desired attitude
        ang_vel_to_euler_rate(_attitude_target_euler_angle, _attitude_target_ang_vel, _attitude_target_euler_rate);
    } else {
        // When feedforward is not enabled, the quaternion is calculated and is input into the target and the feedforward rate is zeroed.
        Quaternion attitude_target_update_quat;
        attitude_target_update_quat.from_axis_angle(Vector3f(roll_rate_rads * _dt, pitch_rate_rads * _dt, yaw_rate_rads * _dt));
        _attitude_target_quat = _attitude_target_quat * attitude_target_update_quat;
        _attitude_target_quat.normalize();

        // Set rate feedforward requests to zero
        _attitude_target_euler_rate = Vector3f(0.0f, 0.0f, 0.0f);
        _attitude_target_ang_vel    = Vector3f(0.0f, 0.0f, 0.0f);
    }

    // Call quaternion attitude controller
    attitude_controller_run_quat();
}



// // Command an euler roll, pitch, and yaw rate with angular velocity feedforward and smoothing
// void AC_AttitudeControl_River::input_euler_rate_roll_pitch_yaw(float euler_roll_rate_cds, float euler_pitch_rate_cds, float euler_yaw_rate_cds)
// {
//     output_to_boat(euler_pitch_rate_cds/(lean_angle_max()),euler_roll_rate_cds/(lean_angle_max()),euler_yaw_rate_cds/(lean_angle_max())); //Mathaus
    

//     // Convert from centidegrees on public interface to radians
//     float euler_roll_rate = 0.0f;
//     float euler_pitch_rate = 0.0f;
//     float euler_yaw_rate = radians(euler_yaw_rate_cds * 0.01f);

//     // calculate the attitude target euler angles
//     _attitude_target_quat.to_euler(_attitude_target_euler_angle.x, _attitude_target_euler_angle.y, _attitude_target_euler_angle.z);

//     if (_rate_bf_ff_enabled) {
//         // translate the roll pitch and yaw acceleration limits to the euler axis
//         Vector3f euler_accel = euler_accel_limit(_attitude_target_euler_angle, Vector3f(get_accel_roll_max_radss(), get_accel_pitch_max_radss(), get_accel_yaw_max_radss()));

//         // When acceleration limiting is enabled, the input shaper constrains angular acceleration, slewing
//         // the output rate towards the input rate.
//         _attitude_target_euler_rate.x = 0.0f;
//         _attitude_target_euler_rate.y = 0.0f;
//         _attitude_target_euler_rate.z = input_shaping_ang_vel(_attitude_target_euler_rate.z, euler_yaw_rate, euler_accel.z, _dt);

//         // Convert euler angle derivative of desired attitude into a body-frame angular velocity vector for feedforward
//         euler_rate_to_ang_vel(_attitude_target_euler_angle, _attitude_target_euler_rate, _attitude_target_ang_vel);
//     } else {
//         // When feedforward is not enabled, the target euler angle is input into the target and the feedforward rate is zeroed.
//         // Pitch angle is restricted to +- 85.0 degrees to avoid gimbal lock discontinuities.
//         _attitude_target_euler_angle.x = wrap_PI(_attitude_target_euler_angle.x + euler_roll_rate * _dt);
//         _attitude_target_euler_angle.y = constrain_float(_attitude_target_euler_angle.y + euler_pitch_rate * _dt, radians(-85.0f), radians(85.0f));
//         _attitude_target_euler_angle.z = wrap_2PI(_attitude_target_euler_angle.z + euler_yaw_rate * _dt);

//         // Set rate feedforward requests to zero
//         _attitude_target_euler_rate = Vector3f(0.0f, 0.0f, 0.0f);
//         _attitude_target_ang_vel = Vector3f(0.0f, 0.0f, 0.0f);

//         // Compute quaternion target attitude
//         _attitude_target_quat.from_euler(_attitude_target_euler_angle.x, _attitude_target_euler_angle.y, _attitude_target_euler_angle.z);
//     }

//     // Call quaternion attitude controller
//     attitude_controller_run_quat();
// }



// // Command an angular velocity with angular velocity smoothing using rate loops only with no attitude loop stabilization
// void AC_AttitudeControl_River::input_rate_bf_roll_pitch_yaw_2(float roll_rate_bf_cds, float pitch_rate_bf_cds, float yaw_rate_bf_cds)
// {
//     output_to_boat(pitch_rate_bf_cds/(lean_angle_max()),roll_rate_bf_cds/(lean_angle_max()),yaw_rate_bf_cds/(lean_angle_max())); //Mathaus
    
//     // Convert from centidegrees on public interface to radians
//     float roll_rate_rads = 0.0f;
//     float pitch_rate_rads = 0.0f;
//     float yaw_rate_rads = radians(yaw_rate_bf_cds * 0.01f);

//     // Compute acceleration-limited body frame rates
//     // When acceleration limiting is enabled, the input shaper constrains angular acceleration about the axis, slewing
//     // the output rate towards the input rate.
//     _attitude_target_ang_vel.x = input_shaping_ang_vel(_attitude_target_ang_vel.x, roll_rate_rads, get_accel_roll_max_radss(), _dt);
//     _attitude_target_ang_vel.y = input_shaping_ang_vel(_attitude_target_ang_vel.y, pitch_rate_rads, get_accel_pitch_max_radss(), _dt);
//     _attitude_target_ang_vel.z = input_shaping_ang_vel(_attitude_target_ang_vel.z, yaw_rate_rads, get_accel_yaw_max_radss(), _dt);

//     // Update the unused targets attitude based on current attitude to condition mode change
//     _ahrs.get_quat_body_to_ned(_attitude_target_quat);
//     _attitude_target_quat.to_euler(_attitude_target_euler_angle.x, _attitude_target_euler_angle.y, _attitude_target_euler_angle.z);
//     // Convert body-frame angular velocity into euler angle derivative of desired attitude
//     ang_vel_to_euler_rate(_attitude_target_euler_angle, _attitude_target_ang_vel, _attitude_target_euler_rate);
//     _rate_target_ang_vel = _attitude_target_ang_vel;
// }

// // Command an angular velocity with angular velocity smoothing using rate loops only with integrated rate error stabilization
// void AC_AttitudeControl_River::input_rate_bf_roll_pitch_yaw_3(float roll_rate_bf_cds, float pitch_rate_bf_cds, float yaw_rate_bf_cds)
// {
//     output_to_boat(pitch_rate_bf_cds/(lean_angle_max()),roll_rate_bf_cds/(lean_angle_max()),yaw_rate_bf_cds/(lean_angle_max())); //Mathaus
//     // Convert from centidegrees on public interface to radians
//     float roll_rate_rads = 0.0f;
//     float pitch_rate_rads = 0.0f;
//     float yaw_rate_rads = radians(yaw_rate_bf_cds * 0.01f);

//     // Update attitude error
//     Vector3f attitude_error_vector;
//     _attitude_ang_error.to_axis_angle(attitude_error_vector);

//     Quaternion attitude_ang_error_update_quat;
//     // limit the integrated error angle
//     float err_mag = attitude_error_vector.length();
//     if (err_mag > AC_ATTITUDE_THRUST_ERROR_ANGLE) {
//         attitude_error_vector *= AC_ATTITUDE_THRUST_ERROR_ANGLE / err_mag;
//         _attitude_ang_error.from_axis_angle(attitude_error_vector);
//     }

//     Vector3f gyro_latest = _ahrs.get_gyro_latest();
//     attitude_ang_error_update_quat.from_axis_angle(Vector3f((_attitude_target_ang_vel.x-gyro_latest.x) * _dt, (_attitude_target_ang_vel.y-gyro_latest.y) * _dt, (_attitude_target_ang_vel.z-gyro_latest.z) * _dt));
//     _attitude_ang_error = attitude_ang_error_update_quat * _attitude_ang_error;

//     // Compute acceleration-limited body frame rates
//     // When acceleration limiting is enabled, the input shaper constrains angular acceleration about the axis, slewing
//     // the output rate towards the input rate.
//     _attitude_target_ang_vel.x = input_shaping_ang_vel( 0.0f, roll_rate_rads, get_accel_roll_max_radss(), _dt);
//     _attitude_target_ang_vel.y = input_shaping_ang_vel( 0.0f, pitch_rate_rads, get_accel_pitch_max_radss(), _dt);
//     _attitude_target_ang_vel.z = input_shaping_ang_vel( 0.0f, yaw_rate_rads, get_accel_yaw_max_radss(), _dt);

//     // Retrieve quaternion vehicle attitude
//     Quaternion attitude_vehicle_quat;
//     _ahrs.get_quat_body_to_ned(attitude_vehicle_quat);

//     // Update the unused targets attitude based on current attitude to condition mode change
//     _attitude_target_quat = attitude_vehicle_quat * _attitude_ang_error;

//     // calculate the attitude target euler angles
//     _attitude_target_quat.to_euler(_attitude_target_euler_angle.x, _attitude_target_euler_angle.y, _attitude_target_euler_angle.z);

//     // Convert body-frame angular velocity into euler angle derivative of desired attitude
//     ang_vel_to_euler_rate(_attitude_target_euler_angle, _attitude_target_ang_vel, _attitude_target_euler_rate);

//     // Compute the angular velocity target from the integrated rate error
//     _attitude_ang_error.to_axis_angle(attitude_error_vector);
//     _rate_target_ang_vel = update_ang_vel_target_from_att_error(attitude_error_vector);
//     _rate_target_ang_vel += _attitude_target_ang_vel;

//     // ensure Quaternions stay normalized
//     _attitude_ang_error.normalize();
// }

// // Command an angular step (i.e change) in body frame angle
// // Used to command a step in angle without exciting the orthogonal axis during autotune
// void AC_AttitudeControl_River::input_angle_step_bf_roll_pitch_yaw(float roll_angle_step_bf_cd, float pitch_angle_step_bf_cd, float yaw_angle_step_bf_cd)
// {
//     output_to_boat(pitch_angle_step_bf_cd/(lean_angle_max()),roll_angle_step_bf_cd/(lean_angle_max()),yaw_angle_step_bf_cd/(lean_angle_max())); //Mathaus
//     // Convert from centidegrees on public interface to radians
//     float roll_step_rads  = 0.0f;
//     float pitch_step_rads = 0.0f;
//     float yaw_step_rads = radians(yaw_angle_step_bf_cd * 0.01f);

//     // rotate attitude target by desired step
//     Quaternion attitude_target_update_quat;
//     attitude_target_update_quat.from_axis_angle(Vector3f(roll_step_rads, pitch_step_rads, yaw_step_rads));
//     _attitude_target_quat = _attitude_target_quat * attitude_target_update_quat;
//     _attitude_target_quat.normalize();

//     // calculate the attitude target euler angles
//     _attitude_target_quat.to_euler(_attitude_target_euler_angle.x, _attitude_target_euler_angle.y, _attitude_target_euler_angle.z);

//     // Set rate feedforward requests to zero
//     _attitude_target_euler_rate = Vector3f(0.0f, 0.0f, 0.0f);
//     _attitude_target_ang_vel = Vector3f(0.0f, 0.0f, 0.0f);

//     // Call quaternion attitude controller
//     attitude_controller_run_quat();
// }