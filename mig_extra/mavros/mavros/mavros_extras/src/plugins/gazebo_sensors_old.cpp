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
         //std::cout << "Got sensor_1 : " << req->sensor_1 <<  std::endl;
         //std::cout << "Got sensor_2 : " << req->sensor_2 <<  std::endl;         
         mavlink::common::msg::GAZEBO_MESSAGE gs {};
         gs.sensor_1 = req->sensor_1;
         gs.sensor_2 = req->sensor_2;   
         UAS_FCU(m_uas)->send_message_ignore_drop(gs);
     }
 };
 }   // namespace extra_plugins
 }   // namespace mavros

PLUGINLIB_EXPORT_CLASS(mavros::extra_plugins::GazeboSensorsPlugin, mavros::plugin::PluginBase)

