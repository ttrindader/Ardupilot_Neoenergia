/*
 *       AP_MotorsRiver.cpp - ArduCopter motors library
 *       Code by Mathaus. mathaus.silva@engenharia.ufjf.br
 */

#include <AP_HAL/AP_HAL.h>
#include "AP_MotorsRiver.h"
#include <GCS_MAVLink/GCS.h>


extern const AP_HAL::HAL &hal;

const AP_Param::GroupInfo AP_MotorsRiver::var_info[] = {
    AP_NESTEDGROUPINFO(AP_MotorsMulticopter, 0),
    
    // @Param: SRVO_MIN_PWM
    // @DisplayName: Servo Min
    // @Description: PWM value sent by the Receiver for the minimum value of the servo position
    // @Range: 0 3000
    // @Units: PWM
    // @User: Advanced
    AP_GROUPINFO("SRVO_MIN_PWM",1,AP_MotorsRiver, _r_srv_min_pwm, 550),

    // @Param: SRVO_MAX_PWM
    // @DisplayName: Servo Max
    // @Description: PWM value sent by the Receiver for the maximum value of the servo position
    // @Range: 0 3000
    // @Units: PWM
    // @User: Advanced
    AP_GROUPINFO("SRVO_MAX_PWM",2,AP_MotorsRiver, _r_srv_max_pwm,  2500),

    AP_GROUPEND
};

/* ****************************** Mathaus *********************************
***************************************************************************/
void AP_MotorsRiver::setup_motors(motor_frame_class frame_class, motor_frame_type frame_type){
    // remove existing motors
    for (int8_t i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; i++) {
        remove_motor(i);
    }
    bool success = true;

    add_motor(AP_MOTORS_MOT_1, 45, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 1);
    add_motor(AP_MOTORS_MOT_2, -135, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 3);
    add_motor(AP_MOTORS_MOT_3, -45, AP_MOTORS_MATRIX_YAW_FACTOR_CW, 4);
    add_motor(AP_MOTORS_MOT_4, 135, AP_MOTORS_MATRIX_YAW_FACTOR_CW, 2);
  
    normalise_rpy_factors();
    set_initialised_ok(success);
}

void AP_MotorsRiver::output_to_motors() 
{
    int8_t i;
    switch (_spool_state) {
    case SpoolState::SHUT_DOWN: {
        // no output
        for (i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; i++)
        {
            if (motor_enabled[i]) {
                _actuator[i] = 0.0f;
            }
        }
        break;
    }
    case SpoolState::GROUND_IDLE:
        // sends output to motors when armed but not flying
        for (i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; i++) {
            if (motor_enabled[i]) {
                set_actuator_with_slew(_actuator[i], actuator_spin_up_to_ground_idle());
            }
        }
        break;
    case SpoolState::SPOOLING_UP:
    case SpoolState::THROTTLE_UNLIMITED:
    case SpoolState::SPOOLING_DOWN:
        // set motor output based on thrust requests
        for (i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; i++) {
            if (motor_enabled[i]) {
                set_actuator_with_slew(_actuator[i], thrust_to_actuator(_thrust_rpyt_out[i]));
            }
        }
        break;
    }

    // convert output to PWM and send to each motor
    for (i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; i++) {
        if (motor_enabled[i]) {
            rc_write(i, output_to_pwm(_actuator[i])); 
        }
    }

}

/* ****************************** Mathaus *********************************
***************************************************************************/
void AP_MotorsRiver::direct_allocation(float &PWM1,float &PWM2,float &PWM3,float &PWM4) {
    Fx_out = (float)(PWM1 + PWM2 + PWM3 + PWM4)/4.0f;
    Fy_out = (float)(0.0);
    Tn_out = (float)((-PWM1 + PWM2 + PWM3 - PWM4))/2.0f;
}

uint8_t counter = 0;

void AP_MotorsRiver::CalibrateServo(float &Pwm_servo){
       
    Pwm_servo = get_lateral() -2930;//(theta_1, 986.0, 1897.0);

    counter++;
    if (counter > 150) {
        counter = 0;
        gcs().send_text(MAV_SEVERITY_CRITICAL, "Pwm_servo Calibrado %5.3f", (double)Pwm_servo);
    }
}

float _key_radio_passthrough = 0.0f;

// pilot input in the -1 ~ +1 range for roll, pitch and yaw. 0~1 range for throttle
void AP_MotorsRiver::radio_key_passthrough_to_motors(float key)
{
    _key_radio_passthrough = key;
}



// output_armed - sends commands to the motors
// includes new scaling stability patch

float AP_MotorsRiver::map_cube(float x, float y, float z)
{
    float out = 0.0f;
    out = x*sqrtf(1 - powf(y,2)/2.0f - powf(z,2)/2.0f + (powf(y,2)*powf(z,2))/3.0f);
    return out;
}

void AP_MotorsRiver::output_armed_stabilizing() {

    xo = constrain_float(get_forward(),-1.0f,1.0f);
    yo = constrain_float(get_lateral(),-1.0f,1.0f);
    zo = constrain_float(get_yaw(),-1.0f,1.0f);

    float theta = atan2f(yo,xo)*RAD_TO_DEG;

    // Angular Speed for Theta, when Theta = 180 Speed = max(1)
    float theta_spd = theta / 180.0f;


    yo = 0.0f;

    zo = zo + theta_spd;

    Fx = map_cube(xo,yo,zo);
    Fy = map_cube(yo,xo,zo);
    Tn = map_cube(zo,yo,xo);
    
    
    if(_key_radio_passthrough<0)
    {
        Differential_allocation_matrix(Fx, Fy, Tn, Pwm1, Pwm2, Pwm3, Pwm4);
        //FOSSEN_allocation_matrix(Fx, Fy, Tn, theta_m1, theta_m2, theta_m3, theta_m4, Pwm1, Pwm2, Pwm3, Pwm4);
    }else{
        Differential_allocation_matrix(Fx, Fy, Tn, Pwm1, Pwm2, Pwm3, Pwm4);
    }

    motor_enabled[0] ? _thrust_rpyt_out[0] = Pwm1 : _thrust_rpyt_out[0] = 0.0f;
    motor_enabled[1] ? _thrust_rpyt_out[1] = Pwm2 : _thrust_rpyt_out[1] = 0.0f;
    motor_enabled[2] ? _thrust_rpyt_out[2] = Pwm3 : _thrust_rpyt_out[2] = 0.0f;
    motor_enabled[3] ? _thrust_rpyt_out[3] = Pwm4 : _thrust_rpyt_out[3] = 0.0f;

}


void AP_MotorsRiver::Differential_allocation_matrix(float FX,float FY,float TN,float &PWM1,float &PWM2,float &PWM3,float &PWM4) {
    /// TRABALHA COM RADIANOS
    /// Fx = força no eixo X - Seu valor deve variar de -1 a 1
    /// Fy = força no eixo y - Seu valor deve variar de -1 a 1
    /// N  = tork de guinada - Seu valor deve variar de -1 a 1
    /// Função para alocar as forças do barco a partir da metodologia descrita em FOSSEN

    FX = constrain_float(FX,-1.0f,1.0f)*4.0f; // Se o PWM de cada motor varia de 0 a 1, a força tem que variar de 0 a N*1, onde N é o número de motores
    TN = constrain_float(TN,-1.0f,1.0f)*4.0f;

    FT = sqrtf(sq(FX)+sq(TN));

    if(FT<0.02) {

        //Envia todos os PWMs muito pequenos (Nulos-Na prática) Os valores aqui, não estão normalizados entre 0 e 1
        PWM2 = 0.0f;//NormtoPWM(0.0f);
        PWM3 = 0.0f;//NormtoPWM(0.0f);
        PWM4 = 0.0f;//NormtoPWM(0.0f);
        PWM1 = 0.0f;//NormtoPWM(0.0f);

    } else {
        // ========================================== PWM calculado a partir da força e dos angulos ====================================
        PWM1 = FX/(4.0f) - TN/(4.0f);
        PWM2 = FX/(4.0f) + TN/(4.0f);
        PWM3 = FX/(4.0f) + TN/(4.0f);
        PWM4 = FX/(4.0f) - TN/(4.0f);

        // Saturação
        PWM1 = constrain_float(PWM1,0.0f,1.0f);
        PWM2 = constrain_float(PWM2,0.0f,1.0f);
        PWM3 = constrain_float(PWM3,0.0f,1.0f);
        PWM4 = constrain_float(PWM4,0.0f,1.0f);
    }

    direct_allocation(PWM1, PWM2, PWM3, PWM4);
}


// // check for failed motor
// //   should be run immediately after output_armed_stabilizing
// //   first argument is the sum of:
// //      a) throttle_thrust_best_rpy : throttle level (from 0 to 1) providing maximum roll, pitch and yaw range without climbing
// //      b) thr_adj: the difference between the pilot's desired throttle and throttle_thrust_best_rpy
// //   records filtered motor output values in _thrust_rpyt_out_filt array
// //   sets thrust_balanced to true if motors are balanced, false if a motor failure is detected
// //   sets _motor_lost_index to index of failed motor
// void AP_MotorsRiver::check_for_failed_motor(float throttle_thrust_best_plus_adj){
//     // record filtered and scaled thrust output for motor loss monitoring purposes
//     float alpha = 1.0f / (1.0f + _loop_rate * 0.5f);
//     for (uint8_t i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; i++)
//     {
//         if (motor_enabled[i])
//         {
//             _thrust_rpyt_out_filt[i] += alpha * (_thrust_rpyt_out[i] - _thrust_rpyt_out_filt[i]);
//         }
//     }
//     float rpyt_high = 0.0f;
//     float rpyt_sum = 0.0f;
//     uint8_t number_motors = 0.0f;
//     for (uint8_t i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; i++)
//     {
//         if (motor_enabled[i])
//         {
//             number_motors += 1;
//             rpyt_sum += _thrust_rpyt_out_filt[i];
//             // record highest filtered thrust command
//             if (_thrust_rpyt_out_filt[i] > rpyt_high)
//             {
//                 rpyt_high = _thrust_rpyt_out_filt[i];
//                 // hold motor lost index constant while thrust boost is active
//                 if (!_thrust_boost)
//                 {
//                     _motor_lost_index = i;
//                 }
//             }
//         }
//     }
//     float thrust_balance = 1.0f;
//     if (rpyt_sum > 0.1f)
//     {
//         thrust_balance = rpyt_high * number_motors / rpyt_sum;
//     }
//     // ensure thrust balance does not activate for multirotors with less than 6 motors
//     if (number_motors >= 6 && thrust_balance >= 1.5f && _thrust_balanced)
//     {
//         _thrust_balanced = false;
//     }
//     if (thrust_balance <= 1.25f && !_thrust_balanced)
//     {
//         _thrust_balanced = true;
//     }
//     // check to see if thrust boost is using more throttle than _throttle_thrust_max
//     if ((_throttle_thrust_max * get_compensation_gain() > throttle_thrust_best_plus_adj) && (rpyt_high < 0.9f) && _thrust_balanced)
//     {
//         _thrust_boost = false;
//     }
// }
// // output_test_seq - spin a motor at the pwm value specified
// //  motor_seq is the motor's sequence number from 1 to the number of motors on the frame
// //  pwm value is an actual pwm value that will be output, normally in the range of 1000 ~ 2000
// void AP_MotorsRiver::output_test_seq(uint8_t motor_seq, int16_t pwm)
// {
//     // exit immediately if not armed
//     if (!armed())
//     {
//         return;
//     }
//     // loop through all the possible orders spinning any motors that match that description
//     for (uint8_t i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; i++)
//     {
//         if (motor_enabled[i] && _test_order[i] == motor_seq)
//         {
//             // turn on this motor
//             rc_write(i, pwm);
//         }
//     }
// }
// // output_test_num - spin a motor connected to the specified output channel
// //  (should only be performed during testing)
// //  If a motor output channel is remapped, the mapped channel is used.
// //  Returns true if motor output is set, false otherwise
// //  pwm value is an actual pwm value that will be output, normally in the range of 1000 ~ 2000
// bool AP_MotorsRiver::output_test_num(uint8_t output_channel, int16_t pwm)
// {
//     if (!armed())
//     {
//         return false;
//     }
//     // Is channel in supported range?
//     if (output_channel > AP_MOTORS_MAX_NUM_MOTORS - 1)
//     {
//         return false;
//     }
//     // Is motor enabled?
//     if (!motor_enabled[output_channel])
//     {
//         return false;
//     }
//     rc_write(output_channel, pwm); // output
//     return true;
// }
// // add_motor
// void AP_MotorsRiver::add_motor_raw(int8_t motor_num, float roll_fac, float pitch_fac, float yaw_fac, uint8_t testing_order)
// {
//     // ensure valid motor number is provided
//     if (motor_num >= 0 && motor_num < AP_MOTORS_MAX_NUM_MOTORS)
//     {
//         // increment number of motors if this motor is being newly motor_enabled
//         if (!motor_enabled[motor_num])
//         {
//             motor_enabled[motor_num] = true;
//         }
//         // set roll, pitch, thottle factors and opposite motor (for stability patch)
//         _roll_factor[motor_num] = roll_fac;
//         _pitch_factor[motor_num] = pitch_fac;
//         _yaw_factor[motor_num] = yaw_fac;
//         // set order that motor appears in test
//         _test_order[motor_num] = testing_order;
//         // call parent class method
//         add_motor_num(motor_num);
//     }
// }
// // add_motor using just position and prop direction - assumes that for each motor, roll and pitch factors are equal
// void AP_MotorsRiver::add_motor(int8_t motor_num, float angle_degrees, float yaw_factor, uint8_t testing_order)
// {
//     add_motor(motor_num, angle_degrees, angle_degrees, yaw_factor, testing_order);
// }
// // add_motor using position and prop direction. Roll and Pitch factors can differ (for asymmetrical frames)
// void AP_MotorsRiver::add_motor(int8_t motor_num, float roll_factor_in_degrees, float pitch_factor_in_degrees, float yaw_factor, uint8_t testing_order)
// {
//     add_motor_raw(
//         motor_num,
//         cosf(radians(roll_factor_in_degrees + 90)),
//         cosf(radians(pitch_factor_in_degrees)),
//         yaw_factor,
//         testing_order);
// }
// // remove_motor - disabled motor and clears all roll, pitch, throttle factors for this motor
// void AP_MotorsRiver::remove_motor(int8_t motor_num)
// {
//     // ensure valid motor number is provided
//     if (motor_num >= 0 && motor_num < AP_MOTORS_MAX_NUM_MOTORS)
//     {
//         // disable the motor, set all factors to zero
//         motor_enabled[motor_num] = false;
//         _roll_factor[motor_num] = 0;
//         _pitch_factor[motor_num] = 0;
//         _yaw_factor[motor_num] = 0;
//     }
// }

// normalizes the roll, pitch and yaw factors so maximum magnitude is 0.5
// void AP_MotorsRiver::normalise_rpy_factors()
// {
//     float roll_fac = 0.0f;
//     float pitch_fac = 0.0f;
//     float yaw_fac = 0.0f;
//     // find maximum roll, pitch and yaw factors
//     for (uint8_t i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; i++)
//     {
//         if (motor_enabled[i])
//         {
//             if (roll_fac < fabsf(_roll_factor[i]))
//             {
//                 roll_fac = fabsf(_roll_factor[i]);
//             }
//             if (pitch_fac < fabsf(_pitch_factor[i]))
//             {
//                 pitch_fac = fabsf(_pitch_factor[i]);
//             }
//             if (yaw_fac < fabsf(_yaw_factor[i]))
//             {
//                 yaw_fac = fabsf(_yaw_factor[i]);
//             }
//         }
//     }
//     // scale factors back to -0.5 to +0.5 for each axis
//     for (uint8_t i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; i++)
//     {
//         if (motor_enabled[i])
//         {
//             if (!is_zero(roll_fac))
//             {
//                 _roll_factor[i] = 0.5f * _roll_factor[i] / roll_fac;
//             }
//             if (!is_zero(pitch_fac))
//             {
//                 _pitch_factor[i] = 0.5f * _pitch_factor[i] / pitch_fac;
//             }
//             if (!is_zero(yaw_fac))
//             {
//                 _yaw_factor[i] = 0.5f * _yaw_factor[i] / yaw_fac;
//             }
//         }
//     }
// }

// /*
//   call vehicle supplied thrust compensation if set. This allows
//   vehicle code to compensate for vehicle specific motor arrangements
//   such as tiltrotors or tiltwings
// */
// void AP_MotorsRiver::thrust_compensation(void)
// {
//     if (_thrust_compensation_callback)
//     {
//         _thrust_compensation_callback(_thrust_rpyt_out, AP_MOTORS_MAX_NUM_MOTORS);
//     }
// }
