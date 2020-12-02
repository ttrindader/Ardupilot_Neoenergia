/// @file	AP_MotorsRiver.h
/// @brief	Motor control class for Matrixcopters
#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>        // ArduPilot Mega Vector/Matrix math Library
#include <RC_Channel/RC_Channel.h>     // RC Channel Library
#include "AP_MotorsMatrix.h"

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

    void radio_key_passthrough_to_motors_key( float key) ;

    // // get_motor_mask - returns a bitmask of which outputs are being used for motors (1 means being used)
    // //  this can be used to ensure other pwm outputs (i.e. for servos) do not conflict
    // uint16_t            get_motor_mask() override;

    // // return number of motor that has failed.  Should only be called if get_thrust_boost() returns true
    // uint8_t             get_lost_motor() const override { return _motor_lost_index; }

    // // return the roll factor of any motor, this is used for tilt rotors and tail sitters
    // // using copter motors for forward flight
    // float               get_roll_factor(uint8_t i) override { return _roll_factor[i]; }

protected:
 
    void FOSSEN_allocation_matrix(float FX,float FY,float tN,float &theta_motor1,float &theta_motor2,float &theta_motor3,float &theta_motor4,float &PWM1 ,float &PWM2 ,float &PWM3 ,float &PWM4);
    void Differential_allocation_matrix(float FX,float FY,float tN,float &theta_motor1,float &theta_motor2,float &theta_motor3,float &theta_motor4,float &PWM1 ,float &PWM2 ,float &PWM3 ,float &PWM4);
    void pwm_servo_angle(float &Pwm_servo_m1,float &Pwm_servo_m2,float &Pwm_servo_m3,float &Pwm_servo_m4,float theta_m1,float theta_m2,float theta_m3,float theta_m4);
    void CalibrateServo(float &Pwm_servo);
    void direct_allocation(float &Theta1,float &Theta2,float &Theta3,float &Theta4,float &PWM1,float &PWM2,float &PWM3,float &PWM4);
    float PWMtoNorm(float pwm);
    float NormtoPWM(float pwm);
    int servo_angle_to_pwm(float angle,float srv_min_pwm,float srv_max_pwm);

    // output - sends commands to the motors
    void                output_armed_stabilizing() override;

    // // check for failed motor
    // void                check_for_failed_motor(float throttle_thrust_best);

    // // add_motor using raw roll, pitch, throttle and yaw factors
    //  void                add_motor_raw(int8_t motor_num, float roll_fac, float pitch_fac, float yaw_fac, uint8_t testing_order);

    // // add_motor using just position and yaw_factor (or prop direction)
    // void                add_motor(int8_t motor_num, float angle_degrees, float yaw_factor, uint8_t testing_order);

    // // add_motor using separate roll and pitch factors (for asymmetrical frames) and prop direction
    // void                add_motor(int8_t motor_num, float roll_factor_in_degrees, float pitch_factor_in_degrees, float yaw_factor, uint8_t testing_order);

    // // remove_motor
    // void                remove_motor(int8_t motor_num);


    // // normalizes the roll, pitch and yaw factors so maximum magnitude is 0.5
    // void                normalise_rpy_factors();

    // // call vehicle supplied thrust compensation if set
    // void                thrust_compensation(void) override;

    // float               _roll_factor[AP_MOTORS_MAX_NUM_MOTORS]; // each motors contribution to roll
    // float               _pitch_factor[AP_MOTORS_MAX_NUM_MOTORS]; // each motors contribution to pitch
    // float               _yaw_factor[AP_MOTORS_MAX_NUM_MOTORS];  // each motors contribution to yaw (normally 1 or -1)
    // float               _thrust_rpyt_out[AP_MOTORS_MAX_NUM_MOTORS]; // combined roll, pitch, yaw and throttle outputs to motors in 0~1 range
    // uint8_t             _test_order[AP_MOTORS_MAX_NUM_MOTORS];  // order of the motors in the test sequence
    // motor_frame_class   _last_frame_class; // most recently requested frame class (i.e. quad, hexa, octa, etc)
    // motor_frame_type    _last_frame_type; // most recently requested frame type (i.e. plus, x, v, etc)

    // // motor failure handling
    // float               _thrust_rpyt_out_filt[AP_MOTORS_MAX_NUM_MOTORS];    // filtered thrust outputs with 1 second time constant
    // uint8_t             _motor_lost_index;  // index number of the lost motor
};
