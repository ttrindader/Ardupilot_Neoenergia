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

void AP_MotorsRiver::output_to_motors() {
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

    pwm_servo_angle(_actuator[8],_actuator[9],_actuator[10],_actuator[11],theta_m1,theta_m2,theta_m3,theta_m4);

    // CalibrateServo(_actuator[8]);// Descomentar se for calibrar

    // convert output to PWM and send to each motor
    for (i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; i++) {
        if (motor_enabled[i]) {
            if(i<7) {
                rc_write(i, output_to_pwm(_actuator[i]));
            } else {
                rc_write(i, (int16_t)_actuator[i]);
            }
        }
    }

}

/* ****************************** Mathaus *********************************
***************************************************************************/
void AP_MotorsRiver::direct_allocation(float &Theta1,float &Theta2,float &Theta3,float &Theta4,float &PWM1,float &PWM2,float &PWM3,float &PWM4) {
    Fx_out = (float)(PWM1*k1*cosf(Theta1) + PWM2*k2*cosf(Theta2) + PWM3*k3*cosf(Theta3) + PWM4*k4*cosf(Theta4));
    Fy_out = (float)(PWM1*k1*sinf(Theta1) + PWM2*k2*sinf(Theta2) + PWM3*k3*sinf(Theta3) + PWM4*k4*sinf(Theta4));
    Tn_out = (float)(Lx*(PWM1*k1*sinf(Theta1) - PWM2*k2*sinf(Theta2) + PWM3*k3*sinf(Theta3) - PWM4*k4*sinf(Theta4)) - Ly*(PWM1*k1*cosf(Theta1) - PWM2*k2*cosf(Theta2) - PWM3*k3*cosf(Theta3) + PWM4*k4*cosf(Theta4)));
}

float AP_MotorsRiver::PWMtoNorm(float pwm) {
    /// Entra um valor de PWM e sai de 0 a 1
    float V = float(pwm - Pwmmin)/float(Pwmmax-Pwmmin);
    return constrain_float(V,0.0f,1.0f);
}

float AP_MotorsRiver::NormtoPWM(float val) {
    /// Entra um valor de 0 a 1 e sai um PWM
    return val*(Pwmmax-Pwmmin) + Pwmmin;
}

int AP_MotorsRiver::servo_angle_to_pwm(float angle,float srv_min_pwm, float srv_max_pwm) {
    /// Nessa função deve-se inserir os valores mínimos e maxímos do pwm  considerando 0 a 180 como angulos mínimos e máximos
    //Entrada de angulo deve ser  de -180 a 180 ELE CHEGARÁ A 180 DEVIDO A ENGRENAGEM
    
    angle = constrain_float(angle,-180.0f,180.0f);

    angle = 180.0f - angle;

    //valor que o servo entende como 0 graus
    float srv_min_angle = 0.0f;

    //valor que o servo entende como 360
    float srv_max_angle = 360.0f;

    int pwm =  srv_min_pwm + angle * (srv_max_pwm - srv_min_pwm)/(srv_max_angle - srv_min_angle);

    return pwm;
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

// output_armed - sends commands to the motors
// includes new scaling stability patch

void AP_MotorsRiver::output_armed_stabilizing() {
    
    Fx = get_forward();
    Fy = get_lateral(); //Colocar zero para calibrar
    Tn = get_yaw();

    FOSSEN_alocation_matrix(Fx, Fy, Tn, theta_m1, theta_m2, theta_m3, theta_m4, Pwm1, Pwm2, Pwm3, Pwm4);

    motor_enabled[0] ? _thrust_rpyt_out[0] = Pwm1 : _thrust_rpyt_out[0] = 0.0f;
    motor_enabled[1] ? _thrust_rpyt_out[1] = Pwm2 : _thrust_rpyt_out[1] = 0.0f;
    motor_enabled[2] ? _thrust_rpyt_out[2] = Pwm3 : _thrust_rpyt_out[2] = 0.0f;
    motor_enabled[3] ? _thrust_rpyt_out[3] = Pwm4 : _thrust_rpyt_out[3] = 0.0f;

}

void AP_MotorsRiver::pwm_servo_angle(float &Pwm_servo_m1, float &Pwm_servo_m2, float &Pwm_servo_m3, float &Pwm_servo_m4, float theta_1, float theta_2, float theta_3, float theta_4) {
    /// todos os angulos devem estar em graus nesta função

    if (get_throttle() < 0.05f) {
        theta_1 = 0.0f;
        theta_2 = 0.0f;
        theta_3 = 0.0f;
        theta_4 = 0.0f;

        _thrust_rpyt_out[0] = get_pwm_output_min();
        _thrust_rpyt_out[1] = get_pwm_output_min();
        _thrust_rpyt_out[2] = get_pwm_output_min();
        _thrust_rpyt_out[3] = get_pwm_output_min();
    }
    
    // BARCO GRANDE

    Pwm_servo_m1 = servo_angle_to_pwm(theta_1,550.0,2500.0);//675.0,2329.0);
    Pwm_servo_m2 = servo_angle_to_pwm(theta_2,550.0,2500.0);//664.0,2144.0);
    Pwm_servo_m3 = servo_angle_to_pwm(theta_3,550.0,2500.0);//656.0,2400.0);
    Pwm_servo_m4 = servo_angle_to_pwm(theta_4,550.0,2500.0);//700.0,2345.0);

    // BARCO PEQUENO
    // Pwm_servo_m1 = servo_angle_to_pwm(theta_1, 550.0, 2500.0);
    // Pwm_servo_m2 = servo_angle_to_pwm(theta_2, 550.0, 2500.0);
    // Pwm_servo_m3 = servo_angle_to_pwm(theta_3, 550.0, 2500.0);
    // Pwm_servo_m4 = servo_angle_to_pwm(theta_4, 550.0, 2500.0);

}

void AP_MotorsRiver::FOSSEN_alocation_matrix(float FX,float FY,float TN,float &Theta1,float &Theta2,float &Theta3,float &Theta4,float &PWM1,float &PWM2,float &PWM3,float &PWM4) {
    /// TRABALHA COM RADIANOS
    /// Fx = força no eixo X - Seu valor deve variar de -1 a 1
    /// Fy = força no eixo y - Seu valor deve variar de -1 a 1
    /// N  = tork de guinada - Seu valor deve variar de -1 a 1
    /// Função para alocar as forças do barco a partir da metodologia descrita em FOSSEN

    FX = constrain_float(FX,-1.0f,1.0f);
    FY = constrain_float(FY,-1.0f,1.0f);
    TN = constrain_float(TN,-1.0f,1.0f);

    TN = TN * Nmax;
    FX = FX * Fmax;
    FY = FY * Fmax;

    FT = safe_sqrt(sq(TN) + sq(FX) + sq(FY));
    FT = constrain_float(FT,0.0f,Fmax);

    // Converte o valor normalizado de 0  a 1 para PWM
    PWM1 = NormtoPWM(PWM1);
    PWM2 = NormtoPWM(PWM2);
    PWM3 = NormtoPWM(PWM3);
    PWM4 = NormtoPWM(PWM4);

    // Convertendo de grau para Radianos
    Theta1 = Theta1 * DEG_TO_RAD;
    Theta2 = Theta2 * DEG_TO_RAD;
    Theta3 = Theta3 * DEG_TO_RAD;
    Theta4 = Theta4 * DEG_TO_RAD;

    if(FT<0.02*Fmax) {
        // Se as forças são muito pequenas (proximas a zero) nao executa a matriz de alocação envia todos os angulos  nulos
        Theta1 = 0.0f;
        Theta2 = 0.0f;
        Theta3 = 0.0f;
        Theta4 = 0.0f;

        //Envia todos os PWMs muito pequenos (Nulos-Na prática) Os valores aqui, não estão normalizados entre 0 e 1
        PWM1 = NormtoPWM(0.0f);
        PWM2 = NormtoPWM(0.0f);
        PWM3 = NormtoPWM(0.0f);
        PWM4 = NormtoPWM(0.0f);

    } else {
        // ========================================== PWM calculado a partir da força e dos angulos ====================================
        PWM1 = (safe_sqrt(sq(FX/(4*k1) - (Ly*TN)/(4*k1*(sq(Lx) + sq(Ly)))) + sq(FY/(4*k1) + (Lx*TN)/(4*k1*(sq(Lx) + sq(Ly))))));
        PWM2 = (safe_sqrt(sq(FX/(4*k2) + (Ly*TN)/(4*k2*(sq(Lx) + sq(Ly)))) + sq(FY/(4*k2) - (Lx*TN)/(4*k2*(sq(Lx) + sq(Ly))))));
        PWM3 = (safe_sqrt(sq(FX/(4*k3) + (Ly*TN)/(4*k3*(sq(Lx) + sq(Ly)))) + sq(FY/(4*k3) + (Lx*TN)/(4*k3*(sq(Lx) + sq(Ly))))));
        PWM4 = (safe_sqrt(sq(FX/(4*k4) - (Ly*TN)/(4*k4*(sq(Lx) + sq(Ly)))) + sq(FY/(4*k4) - (Lx*TN)/(4*k4*(sq(Lx) + sq(Ly))))));

        // Saturação
        PWM1 = constrain_float(PWM1,Pwmmin,Pwmmax);
        PWM2 = constrain_float(PWM2,Pwmmin,Pwmmax);
        PWM3 = constrain_float(PWM3,Pwmmin,Pwmmax);
        PWM4 = constrain_float(PWM4,Pwmmin,Pwmmax);

        // =============================== Arco seno do angulo calculado a partir da força e do novo PWM ===============================
        Theta1 = atan2f((FY/(4*k1) + (Lx*TN)/(4*k1*(sq(Lx) + sq(Ly)))),(FX/(4*k1) - (Ly*TN)/(4*k1*(sq(Lx) + sq(Ly)))));
        Theta2 = atan2f((FY/(4*k2) - (Lx*TN)/(4*k2*(sq(Lx) + sq(Ly)))),(FX/(4*k2) + (Ly*TN)/(4*k2*(sq(Lx) + sq(Ly)))));
        Theta3 = atan2f((FY/(4*k3) + (Lx*TN)/(4*k3*(sq(Lx) + sq(Ly)))),(FX/(4*k3) + (Ly*TN)/(4*k3*(sq(Lx) + sq(Ly)))));
        Theta4 = atan2f((FY/(4*k4) - (Lx*TN)/(4*k4*(sq(Lx) + sq(Ly)))),(FX/(4*k4) - (Ly*TN)/(4*k4*(sq(Lx) + sq(Ly)))));

        // Saturação
        Theta1 = constrain_float(Theta1,-M_PI,M_PI);
        Theta2 = constrain_float(Theta2,-M_PI,M_PI);
        Theta3 = constrain_float(Theta3,-M_PI,M_PI);
        Theta4 = constrain_float(Theta4,-M_PI,M_PI);
    }

    direct_allocation(Theta1, Theta2, Theta3, Theta4, PWM1, PWM2, PWM3, PWM4);

    // Normaliza o valor de PWM encontrado entre 0 e 1 para ativar a saida entre mínima e maxima potência
    PWM1 = PWMtoNorm(PWM1);
    PWM2 = PWMtoNorm(PWM2);
    PWM3 = PWMtoNorm(PWM3);
    PWM4 = PWMtoNorm(PWM4);

    // Conveter o valor de Theta para Graus
    Theta1 = Theta1 * RAD_TO_DEG;
    Theta2 = Theta2 * RAD_TO_DEG;
    Theta3 = Theta3 * RAD_TO_DEG;
    Theta4 = Theta4 * RAD_TO_DEG;
}

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

    add_motor_raw(AP_MOTORS_MOT_9 , 0, 0, 0, 5);
    add_motor_raw(AP_MOTORS_MOT_10, 0, 0, 0, 6);
    add_motor_raw(AP_MOTORS_MOT_11, 0, 0, 0, 7);
    add_motor_raw(AP_MOTORS_MOT_12, 0, 0, 0, 8);
  
    normalise_rpy_factors();

    set_initialised_ok(success);

    // switch (frame_class)
    // {
    // case MOTOR_FRAME_QUAD:
    //     switch (frame_type)
    //     {
    //     case MOTOR_FRAME_TYPE_PLUS:
    //     case MOTOR_FRAME_TYPE_X:
    //         break;
    //     default:
    //         // quad frame class does not support this frame type
    //             // success = false
    //         break;
    //     }
    //     break; // quad
    // default:
    //     // quad frame class does not support this frame type
    //     // success = false;
    //     break;
    // } // switch frame_class
    // normalise factors to magnitude 0.5
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
