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
    add_motor(AP_MOTORS_MOT_2, -135, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 2);
    add_motor(AP_MOTORS_MOT_3, -45, AP_MOTORS_MATRIX_YAW_FACTOR_CW, 3);
    add_motor(AP_MOTORS_MOT_4, 135, AP_MOTORS_MATRIX_YAW_FACTOR_CW, 4);
    add_motor(AP_MOTORS_MOT_5, -45, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 5);
    add_motor(AP_MOTORS_MOT_6, 135, AP_MOTORS_MATRIX_YAW_FACTOR_CW, 6);
    
  
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

    // float theta = atan2f(yo,xo)*RAD_TO_DEG;
    // Angular Speed for Theta, when Theta = 180 Speed = max(1)
    // float theta_spd = theta / 180.0f;
    yo = 0.0f;

    // zo = zo + theta_spd;

    Fx = map_cube(xo,yo,zo);
    Fy = map_cube(yo,xo,zo);
    Tn = map_cube(zo,yo,xo);

    allocation_matrix(Fx, Fy, Tn, Pwm1, Pwm2, Pwm3, Pwm4, Pwm5, Pwm6);
        
    // if(_key_radio_passthrough < 0)
    // {
    //     Differential_allocation_matrix(Fx, Fy, Tn, Pwm1, Pwm2, Pwm3, Pwm4);
    //     //FOSSEN_allocation_matrix(Fx, Fy, Tn, theta_m1, theta_m2, theta_m3, theta_m4, Pwm1, Pwm2, Pwm3, Pwm4);
    // }else{
    //     Differential_allocation_matrix(Fx, Fy, Tn, Pwm1, Pwm2, Pwm3, Pwm4);
    // }

    motor_enabled[0] ? _thrust_rpyt_out[0] = Pwm1 : _thrust_rpyt_out[0] = 0.0f;
    motor_enabled[1] ? _thrust_rpyt_out[1] = Pwm2 : _thrust_rpyt_out[1] = 0.0f;
    motor_enabled[2] ? _thrust_rpyt_out[2] = Pwm3 : _thrust_rpyt_out[2] = 0.0f;
    motor_enabled[3] ? _thrust_rpyt_out[3] = Pwm4 : _thrust_rpyt_out[3] = 0.0f;
    motor_enabled[4] ? _thrust_rpyt_out[4] = Pwm5 : _thrust_rpyt_out[4] = 0.0f;
    motor_enabled[5] ? _thrust_rpyt_out[5] = Pwm6 : _thrust_rpyt_out[5] = 0.0f;
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

    // direct_allocation(PWM1, PWM2, PWM3, PWM4);
}

void AP_MotorsRiver::allocation_matrix(float FX,float FY,float TN,float &PWM1,float &PWM2,float &PWM3,float &PWM4,float &PWM5,float &PWM6) {
    /// TRABALHA COM RADIANOS
    /// Fx = força no eixo X - Seu valor deve variar de -1 a 1
    /// Fy = força no eixo y - Seu valor deve variar de -1 a 1
    /// N  = tork de guinada - Seu valor deve variar de -1 a 1
    /// Função para alocar as forças do barco a partir da metodologia descrita em FOSSEN

    FX = constrain_float(FX,-1.0f,1.0f)*4.0f; // Se o PWM de cada motor varia de 0 a 1, a força tem que variar de 0 a N*1, onde N é o número de motores
    TN = constrain_float(TN,-1.0f,1.0f)*3.0f;

    FT = sqrtf(sq(FX)+sq(TN));

    if(FT<0.02) {

        //Envia todos os PWMs muito pequenos (Nulos-Na prática) Os valores aqui, não estão normalizados entre 0 e 1
        PWM2 = 0.0f;//NormtoPWM(0.0f);
        PWM3 = 0.0f;//NormtoPWM(0.0f);
        PWM4 = 0.0f;//NormtoPWM(0.0f);
        PWM1 = 0.0f;//NormtoPWM(0.0f);
        PWM5 = 0.0f;//NormtoPWM(0.0f);
        PWM6 = 0.0f;//NormtoPWM(0.0f);

    } else {
        // ========================================== PWM calculado a partir da força e dos angulos ====================================
        PWM1 = FX/(6) - TN/(6*Ly);
        PWM2 = FX/(6) + TN/(6*Ly);
        PWM3 = FX/(6) + TN/(6*Ly);
        PWM4 = FX/(6) - TN/(6*Ly);
        PWM5 = -FX/(6) - TN/(6*Ly);
        PWM6 = -FX/(6) + TN/(6*Ly);

        // Saturação
        PWM1 = constrain_float(PWM1,0.0f,1.0f);
        PWM2 = constrain_float(PWM2,0.0f,1.0f);
        PWM3 = constrain_float(PWM3,0.0f,1.0f);
        PWM4 = constrain_float(PWM4,0.0f,1.0f);
        PWM5 = constrain_float(PWM5,0.0f,1.0f);
        PWM6 = constrain_float(PWM6,0.0f,1.0f);
    }

    // direct_allocation(PWM1, PWM2, PWM3, PWM4);
}