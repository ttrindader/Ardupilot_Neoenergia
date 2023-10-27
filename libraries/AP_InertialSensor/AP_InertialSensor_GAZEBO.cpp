#include <AP_HAL/AP_HAL.h>
#include "AP_InertialSensor_GAZEBO.h"
#include <stdio.h>
#include <GCS_MAVLink/GCS.h> //TTR: TO DEBUG
//#include <SITL/SITL.h>

//#if CONFIG_HAL_BOARD == HAL_BOARD_SITL

const extern AP_HAL::HAL& hal;

AP_InertialSensor_GAZEBO::AP_InertialSensor_GAZEBO(AP_InertialSensor &imu, const uint16_t sample_rates[]) :
    AP_InertialSensor_Backend(imu),
    gyro_sample_hz(sample_rates[0]),
    accel_sample_hz(sample_rates[1])
{
}

/*
  detect the sensor
 */
AP_InertialSensor_Backend *AP_InertialSensor_GAZEBO::detect(AP_InertialSensor &_imu, const uint16_t sample_rates[])
{
    AP_InertialSensor_GAZEBO *sensor = new AP_InertialSensor_GAZEBO(_imu, sample_rates);
    if (sensor == nullptr) {
        return nullptr;
    }
    if (!sensor->init_sensor()) {
        delete sensor;
        return nullptr;
    }
    return sensor;
}

bool AP_InertialSensor_GAZEBO::init_sensor(void)
{
   // sitl = AP::sitl();
   //sitl = sitl();
    //if (sitl == nullptr) {
    //   gcs().send_text(MAV_SEVERITY_CRITICAL, "no sesnsors"); //TTR: initial debug
    //    return false;
        
    //}
    return true;
}

/*
  get accel data from gazebo
 */
void AP_InertialSensor_GAZEBO::generate_accel()
{  
    float xAccel =  GazeboMsgs::data.imuLinearAccelerationXYZ[0];
    float yAccel =  GazeboMsgs::data.imuLinearAccelerationXYZ[1];
    float zAccel =  -GazeboMsgs::data.imuLinearAccelerationXYZ[2];        
    
   /*
    gcs().send_text(MAV_SEVERITY_CRITICAL, "com p = %f", xAccel); //TTR: initial debug
    gcs().send_text(MAV_SEVERITY_CRITICAL, "com q = %f", yAccel); //TTR: initial debug   
    gcs().send_text(MAV_SEVERITY_CRITICAL, "com q = %f", zAccel); //TTR: initial debug         
    gcs().send_text(MAV_SEVERITY_CRITICAL, "OLHA EU AQUI GERANDO ACCEL" ); //TTR: initial debug */   
     


    Vector3f accel = Vector3f(xAccel, yAccel, zAccel);

    _notify_new_accel_sensor_rate_sample(accel_instance, accel);
    
    _rotate_and_correct_accel(accel_instance, accel);
    
    _notify_new_accel_raw_sample(accel_instance, accel, AP_HAL::micros64());

    _publish_temperature(accel_instance, 28);
    }

/*
  get gyro data from gazebo
 */
void AP_InertialSensor_GAZEBO::generate_gyro()
{
  
    float p =  GazeboMsgs::data.imuAngularVelocityRPY[0];
    float q =  GazeboMsgs::data.imuAngularVelocityRPY[1];
    float r =  -GazeboMsgs::data.imuAngularVelocityRPY[2];        
    
   
    //gcs().send_text(MAV_SEVERITY_CRITICAL, "com p = %f", p); //TTR: initial debug
    //gcs().send_text(MAV_SEVERITY_CRITICAL, "com q = %f", q); //TTR: initial debug   
    //gcs().send_text(MAV_SEVERITY_CRITICAL, "com q = %f", r); //TTR: initial debug         
    //gcs().send_text(MAV_SEVERITY_CRITICAL, "OLHA EU AQUI GERANDO GYRO" ); //TTR: initial debug    
    
    Vector3f gyro = Vector3f(p, q, r);
    
    _notify_new_gyro_sensor_rate_sample(gyro_instance, gyro);
      
    _notify_new_gyro_raw_sample(gyro_instance, gyro, AP_HAL::micros64());

}

void AP_InertialSensor_GAZEBO::timer_update(void)
{
    uint64_t now = AP_HAL::micros64();
#if 0
    // insert a 1s pause in IMU data. This triggers a pause in EK2
    // processing that leads to some interesting issues
    if (now > 5e6 && now < 6e6) {
        return;
    }
#endif

   // if (now > 5e6 && now < 6e6) {
   //     return;
   // }

    if (now >= next_accel_sample) {
        if (((1U << accel_instance) & accel_fail_mask) == 0) {
            generate_accel();
            if (next_accel_sample == 0) {
                next_accel_sample = now + 1000000UL / accel_sample_hz;
            } else {
                while (now >= next_accel_sample) {
                    next_accel_sample += 1000000UL / accel_sample_hz;
                }
            }
        }
    }
    if (now >= next_gyro_sample) {
        if (((1U << gyro_instance) & gyro_fail_mask) == 0) {
            generate_gyro();
            if (next_gyro_sample == 0) {
                next_gyro_sample = now + 1000000UL / gyro_sample_hz;
            } else {
                while (now >= next_gyro_sample) {
                    next_gyro_sample += 1000000UL / gyro_sample_hz;
                }
            }
        }
    }
}


bool AP_InertialSensor_GAZEBO::update(void) 
{
    update_accel(accel_instance);
    update_gyro(gyro_instance);
    return true;
}

uint8_t AP_InertialSensor_GAZEBO::bus_id = 0;

void AP_InertialSensor_GAZEBO::start()
{
    gyro_instance = _imu.register_gyro(gyro_sample_hz,
                                        AP_HAL::Device::make_bus_id(AP_HAL::Device::BUS_TYPE_SITL, bus_id, 1, DEVTYPE_GAZEBO));
    accel_instance = _imu.register_accel(accel_sample_hz,
                                        AP_HAL::Device::make_bus_id(AP_HAL::Device::BUS_TYPE_SITL, bus_id, 2, DEVTYPE_GAZEBO));
    bus_id++;
    hal.scheduler->register_timer_process(FUNCTOR_BIND_MEMBER(&AP_InertialSensor_GAZEBO::timer_update, void));
}

//#endif // HAL_BOARD_SITL
