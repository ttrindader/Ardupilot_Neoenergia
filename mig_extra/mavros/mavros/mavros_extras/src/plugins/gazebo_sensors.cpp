 #include <mavros/mavros_plugin.h>
 #include <pluginlib/class_list_macros.h>
 #include <iostream>
#include <mavros_msgs/GazeboMavlink.h>

 namespace mavros {
 namespace extra_plugins{

 class GazeboSensorsPlugin : public plugin::PluginBase {
 public:
     GazeboSensorsPlugin() : PluginBase(),
         nh("~gazebo_sensors")

    { };

     void initialize(UAS &uas_)
     {
         PluginBase::initialize(uas_);
         gazebo_sensors_sub = nh.subscribe("/mavros/gazebo/sensors", 10, &GazeboSensorsPlugin::gazebo_sensors_cb, this);
     };

     Subscriptions get_subscriptions()
     {
         return {/* RX disabled */ };
     }

 private:
     ros::NodeHandle nh;
     ros::Subscriber gazebo_sensors_sub;

    void gazebo_sensors_cb(const mavros_msgs::GazeboMavlink::ConstPtr &req)
     {
         /*
         std::cout << "imuLinearAccelerationX : " << req->imuLinearAccelerationXYZ[0] <<  std::endl;
         std::cout << "imuLinearAccelerationY : " << req->imuLinearAccelerationXYZ[1] <<  std::endl;         
         std::cout << "imuLinearAccelerationZ : " << req->imuLinearAccelerationXYZ[2] <<  std::endl;                  
         std::cout << "imuAngularVelocityR : " << req->imuAngularVelocityRPY[0] <<  std::endl;         
         std::cout << "imuAngularVelocityP : " << req->imuAngularVelocityRPY[1] <<  std::endl;       
         std::cout << "imuAngularVelocityY : " << req->imuAngularVelocityRPY[2] <<  std::endl;*/                                      
         mavlink::common::msg::GAZEBO_MESSAGE gs {};
         gs.imuLinearAccelerationXYZ[0] = req->imuLinearAccelerationXYZ[0];
         gs.imuLinearAccelerationXYZ[1] = req->imuLinearAccelerationXYZ[1];
         gs.imuLinearAccelerationXYZ[2] = req->imuLinearAccelerationXYZ[2];                  
         gs.imuAngularVelocityRPY[0] = req->imuAngularVelocityRPY[0];
         gs.imuAngularVelocityRPY[1] = req->imuAngularVelocityRPY[1];
         gs.imuAngularVelocityRPY[2] = req->imuAngularVelocityRPY[2];  
         gs.magneticFieldXYZ[0] = req->magneticFieldXYZ[0];
         gs.magneticFieldXYZ[1] = req->magneticFieldXYZ[1];
         gs.magneticFieldXYZ[2] = req->magneticFieldXYZ[2]; 
         gs.latitude = req->latitude;
         gs.longitude = req->longitude;
         gs.altitude = req->altitude;    
         gs.velocityENU[0] = req->velocityENU[0];
         gs.velocityENU[1] = req->velocityENU[1];
         gs.velocityENU[2] = req->velocityENU[2];                         
         
                 
         UAS_FCU(m_uas)->send_message_ignore_drop(gs);
     }
 };
 }   // namespace extra_plugins
 }   // namespace mavros

PLUGINLIB_EXPORT_CLASS(mavros::extra_plugins::GazeboSensorsPlugin, mavros::plugin::PluginBase)

