#include "Copter.h"

/*
 * Init and run calls for stabilize flight mode
 */

// stabilize_run - runs the main stabilize controller
// should be called at 100hz or more
static float ini  = 0;
static int entra = 0;

static uint8_t counter = 0;
float t_atual = 0.0f ;

// SetPoint
uint16_t Tempo[10] = {5000,10000,15000,20000,25000,30000,35000,40000,45000,50000} ;
float SP_roll[10]   = {1500,2000,2500,-2500,0000,-3000,-4500,000,000,000};
float SP_pitch[10]  = {0,0,0,-0,0000,-0,-0,000,000,000};

void Mode::SinalAntonio(float &target_roll,float &target_pitch,uint16_t Vet_Tempo[], float Vet_SP_roll[],float Vet_SP_pitch[],float key)
{
        if(key>0){
        
        if(entra == 0){
        entra = 1;
        ini = AP_HAL::millis();
        }

        t_atual = AP_HAL::millis() - ini;

        for(uint8_t i=0;i < sizeof(Vet_Tempo);i++)
        {
            if(t_atual>Vet_Tempo[i])
            {
                target_roll  = Vet_SP_roll[i];
                target_pitch = Vet_SP_pitch[i];

            }
        }


        // if (t_atual>5000){ // 5 segundos
        //     target_roll = 1500;
        //     if(t_atual>10000){ // 10 segundos
        //         target_roll = 2000;
        //         if(t_atual>15000){ // 15 segundos
        //             target_roll = 2500;
        //             if(t_atual>20000){ // 20 segundos
        //                 target_roll = 4500;
        //             }
        //         }
        //     }
        // }
    }else{
        entra = 0;
    }
    counter++;
    if (counter > 150) {
        counter = 0;
        gcs().send_text(MAV_SEVERITY_CRITICAL, "target_roll: %5.3f target_pitch: %5.3f time: %5.3f", (double)target_roll, (double)target_pitch, (double)t_atual);
    }
}


void ModeStabilize::run()
{
    // apply simple mode transform to pilot inputs
    update_simple_mode();

    // convert pilot input to lean angles
    float target_roll, target_pitch;
    get_pilot_desired_lean_angles(target_roll, target_pitch, copter.aparm.angle_max, copter.aparm.angle_max);

    // get pilot's desired yaw rate
    float target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());

    if (!motors->armed()) {
        // Motors should be Stopped
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::SHUT_DOWN);
    } else if (copter.ap.throttle_zero) {
        // Attempting to Land
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
    } else {
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
    }

    switch (motors->get_spool_state()) {
    case AP_Motors::SpoolState::SHUT_DOWN:
        // Motors Stopped
        attitude_control->set_yaw_target_to_current_heading();
        attitude_control->reset_rate_controller_I_terms();
        break;

    case AP_Motors::SpoolState::GROUND_IDLE:
        // Landed
        attitude_control->set_yaw_target_to_current_heading();
        attitude_control->reset_rate_controller_I_terms();
        break;

    case AP_Motors::SpoolState::THROTTLE_UNLIMITED:
        // clear landing flag above zero throttle
        if (!motors->limit.throttle_lower) {
            set_land_complete(false);
        }
        break;

    case AP_Motors::SpoolState::SPOOLING_UP:
    case AP_Motors::SpoolState::SPOOLING_DOWN:
        // do nothing
        break;
    }


    // o Sinal deve ser colocado aqui, antes do input ...
    SinalAntonio(target_roll,target_pitch, Tempo, SP_roll,SP_pitch,channel_key->norm_input());

    // call attitude controller
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate);

    // output pilot's throttle
    attitude_control->set_throttle_out(get_pilot_desired_throttle(), true,g.throttle_filt);
}
