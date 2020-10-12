#include <iostream>
#include <thread>
#include <chrono>
#include <map>

#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <geometry_msgs/TwistStamped.h>

#include "aslan_msgs/RemoteCmd.h"
#include "aslan_msgs/VehicleCmd.h"
#include "aslan_msgs/AccelCmd.h"
#include "aslan_msgs/BrakeCmd.h"
#include "aslan_msgs/SteerCmd.h"
#include "aslan_msgs/ControlCommandStamped.h"

class TwistGate
{
  using remote_msgs_t = aslan_msgs::RemoteCmd;
  using vehicle_cmd_msg_t = aslan_msgs::VehicleCmd;

  public:
    TwistGate(const ros::NodeHandle& nh, const ros::NodeHandle& private_nh);
    ~TwistGate();
    int RUN(int argc, char** argv);
    aslan_msgs::VehicleCmd reset_vehicle_cmd_msg_test();
  private:
    void watchdog_timer();
    void remote_cmd_callback(const remote_msgs_t::ConstPtr& input_msg);
    void auto_cmd_twist_cmd_callback(const geometry_msgs::TwistStamped::ConstPtr& input_msg);
    void accel_cmd_callback(const aslan_msgs::AccelCmd::ConstPtr& input_msg);
    void steer_cmd_callback(const aslan_msgs::SteerCmd::ConstPtr& input_msg);
    void brake_cmd_callback(const aslan_msgs::BrakeCmd::ConstPtr& input_msg);
    void lamp_cmd_callback(const aslan_msgs::LampCmd::ConstPtr& input_msg);
    void ctrl_cmd_callback(const aslan_msgs::ControlCommandStamped::ConstPtr& input_msg);

    void reset_vehicle_cmd_msg();

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Publisher emergency_stop_pub_;
    ros::Publisher control_command_pub_;
    ros::Publisher vehicle_cmd_pub_;
    ros::Publisher state_cmd_pub_;
    ros::Subscriber remote_cmd_sub_;
    std::map<std::string , ros::Subscriber> auto_cmd_sub_stdmap_;

    vehicle_cmd_msg_t twist_gate_msg_;
    std_msgs::Bool emergency_stop_msg_;
    ros::Time remote_cmd_time_;
    ros::Duration timeout_period_;

    std::thread watchdog_timer_thread_;
    enum class CommandMode{AUTO=1, REMOTE=2} command_mode_, previous_command_mode_;
    std_msgs::String command_mode_topic_;

    // still send is true
    bool send_emergency_cmd = false;
};