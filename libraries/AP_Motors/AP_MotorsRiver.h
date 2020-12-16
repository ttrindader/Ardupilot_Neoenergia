/// @file	AP_MotorsRiver.h
/// @brief	Motor control class for Matrixcopters
#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>        // ArduPilot Mega Vector/Matrix math Library
#include <RC_Channel/RC_Channel.h>     // RC Channel Library
#include "AP_MotorsMatrix.h"
#include <AP_Param/AP_Param.h>

#define AP_MOTORS_MATRIX_YAW_FACTOR_CW   -1
#define AP_MOTORS_MATRIX_YAW_FACTOR_CCW   1

/// @class      AP_MotorsRiver
class AP_MotorsRiver : public AP_MotorsMatrix {
public:
    
 // Propriedade Física do Barco
    float FT = 0.0f;
    float FM1 = GRAVITY_MSS*2.1f;
    float FM2 = GRAVITY_MSS*2.1f;
    float FM3 = GRAVITY_MSS*2.1f;
    float FM4 = GRAVITY_MSS*2.1f;

    float Fmax = FM1 + FM2 + FM3 + FM4;       // Força e torque maximos do barco

    float L  = 0.54f;          // Tamanho do braço do barco
    float Lx = L*cosf(M_PI/4.0f);
    float Ly = L*cosf(M_PI/4.0f);

    //
    float Pwmmax = 1001.0f; // Esse valor será a faixa de pwm que eu vou escolher para trabalhar --------------- // Esse valor é atualizado no AduCopter.cpp para corresponder aos valores de memória
    float Pwmmin = 1.0f;    // Esse valor é atualizado no AduCopter.cpp para corresponder aos valores de memória
    float Nmax = L*Fmax;

    float k1 = (FM1)/(Pwmmax-Pwmmin); // Esse valor é atualizado no AduCopter.cpp para corresponder aos valores de memória
    float k2 = (FM2)/(Pwmmax-Pwmmin); // Esse valor é atualizado no AduCopter.cpp para corresponder aos valores de memória
    float k3 = (FM3)/(Pwmmax-Pwmmin); // Esse valor é atualizado no AduCopter.cpp para corresponder aos valores de memória
    float k4 = (FM4)/(Pwmmax-Pwmmin); // Esse valor é atualizado no AduCopter.cpp para corresponder aos valores de memória

    uint8_t counter = 0;
    //  PWM do Servo Motores Barco
    float servo_m1 = 0.0f;
    float servo_m2 = 0.0f;
    float servo_m3 = 0.0f;
    float servo_m4 = 0.0f;

    // Angulo dos Servo Motores Barco
    float theta_m1 =  0.0f;
    float theta_m2 =  0.0f;
    float theta_m3 =  0.0f;
    float theta_m4 =  0.0f;

    float Pwm1 = 0.0f;
    float Pwm2 = 0.0f;
    float Pwm3 = 0.0f;
    float Pwm4 = 0.0f;

    // Para o Log
    float Fx = 0.0f;
    float Fy = 0.0f;
    float Tn = 0.0f;

    float Fx_out = 0.0f;
    float Fy_out = 0.0f;
    float Tn_out = 0.0f;  

    /// Constructor
    AP_MotorsRiver(uint16_t loop_rate, uint16_t speed_hz = AP_MOTORS_SPEED_DEFAULT) :
        AP_MotorsMatrix(loop_rate, speed_hz)
    {};

    // // init
    // void                init(motor_frame_class frame_class, motor_frame_type frame_type) override;

    // // set frame class (i.e. quad, hexa, heli) and type (i.e. x, plus)
    // void                set_frame_class_and_type(motor_frame_class frame_class, motor_frame_type frame_type) override;

    // // set update rate to motors - a value in hertz
    // // you must have setup_motors before calling this
    // void                set_update_rate(uint16_t speed_hz) override;

    // // output_test_seq - spin a motor at the pwm value specified
    // //  motor_seq is the motor's sequence number from 1 to the number of motors on the frame
    // //  pwm value is an actual pwm value that will be output, normally in the range of 1000 ~ 2000
    // virtual void        output_test_seq(uint8_t motor_seq, int16_t pwm) override;

    // // output_test_num - spin a motor connected to the specified output channel
    // //  (should only be performed during testing)
    // //  If a motor output channel is remapped, the mapped channel is used.
    // //  Returns true if motor output is set, false otherwise
    // //  pwm value is an actual pwm value that will be output, normally in the range of 1000 ~ 2000
    // bool                output_test_num(uint8_t motor, int16_t pwm);

    // configures the motors for the defined frame_class and frame_type
    void                setup_motors(motor_frame_class frame_class, motor_frame_type frame_type) override;
    
    // output_to_motors - sends minimum values out to the motors
    void                output_to_motors() override;

    // // get_motor_mask - returns a bitmask of which outputs are being used for motors (1 means being used)
    // //  this can be used to ensure other pwm outputs (i.e. for servos) do not conflict
    // uint16_t            get_motor_mask() override;

    // // return number of motor that has failed.  Should only be called if get_thrust_boost() returns true
    // uint8_t             get_lost_motor() const override { return _motor_lost_index; }

    // // return the roll factor of any motor, this is used for tilt rotors and tail sitters
    // // using copter motors for forward flight
    // float               get_roll_factor(uint8_t i) override { return _roll_factor[i]; }

    // var_info for holding Parameter information
    static const struct AP_Param::GroupInfo var_info[];

protected:
// River Params
    AP_Float        _r_srv_min_pwm; // valor de pwm para o  minimo do direcionamento dos servos (-180 graus)
    AP_Float        _r_srv_max_pwm; // valor de pwm para o  máximo do direcionamento dos servos (+180 graus)
 
    void FOSSEN_alocation_matrix(float FX,float FY,float tN,float &theta_motor1,float &theta_motor2,float &theta_motor3,float &theta_motor4,float &PWM1 ,float &PWM2 ,float &PWM3 ,float &PWM4);
    void pwm_servo_angle(float &Pwm_servo_m1,float &Pwm_servo_m2,float &Pwm_servo_m3,float &Pwm_servo_m4,float theta_m1,float theta_m2,float theta_m3,float theta_m4);
    void CalibrateServo(float &Pwm_servo);
    void direct_allocation(float &Theta1,float &Theta2,float &Theta3,float &Theta4,float &PWM1,float &PWM2,float &PWM3,float &PWM4);
    float PWMtoNorm(float pwm);
    float NormtoPWM(float pwm);
    int servo_angle_to_pwm(float angle,float srv_min_pwm,float srv_max_pwm);

    // output - sends commands to the motors
    void                output_armed_stabilizing() override;

};
