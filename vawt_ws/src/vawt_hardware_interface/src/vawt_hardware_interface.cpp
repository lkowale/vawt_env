#include <sstream>
#include <vawt_hardware_interface/vawt_hardware_interface.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <joint_limits_interface/joint_limits_rosparam.h>

using namespace hardware_interface;
using joint_limits_interface::JointLimits;
using joint_limits_interface::SoftJointLimits;
using joint_limits_interface::PositionJointSoftLimitsHandle;
using joint_limits_interface::PositionJointSoftLimitsInterface;

namespace vawt_hardware_interface
{
    VAWTHardwareInterface::VAWTHardwareInterface(ros::NodeHandle& nh) : nh_(nh) {
        init();
        controller_manager_.reset(new controller_manager::ControllerManager(this, nh_));
        nh_.param("/vawt_1/hardware_interface/loop_hz", loop_hz_, 0.1);
        ros::Duration update_freq = ros::Duration(1.0/loop_hz_);
        non_realtime_loop_ = nh_.createTimer(update_freq, &VAWTHardwareInterface::update, this);
        command_publisher = nh_.advertise<std_msgs::Float32>("servo_command", 100);
        ROS_INFO("Initialized VAWTHardwareInterface");
    }

    VAWTHardwareInterface::~VAWTHardwareInterface() {

    }

    void VAWTHardwareInterface::init() {
        // Get joint names
        nh_.getParam("/vawt_1/hardware_interface/joints", joint_names_);
         //joint_names_="blade_1_joint blade_2_joint";
        num_joints_ = joint_names_.size();
        ROS_INFO("VAWTHardwareInterface initializer joints: %i",num_joints_);

        // Resize vectors
        joint_position_.resize(num_joints_);
        joint_velocity_.resize(num_joints_);
        joint_effort_.resize(num_joints_);
        joint_position_command_.resize(num_joints_);
        joint_velocity_command_.resize(num_joints_);
        joint_effort_command_.resize(num_joints_);
        servo_position_.resize(num_joints_);

        // Initialize Controller 
        for (int i = 0; i < num_joints_; ++i) {

             // Create joint state interface
            JointStateHandle jointStateHandle(joint_names_[i], &joint_position_[i], &joint_velocity_[i], &joint_effort_[i]);
             joint_state_interface_.registerHandle(jointStateHandle);

            // Create position joint interface
            JointHandle jointPositionHandle(jointStateHandle, &joint_position_command_[i]);
            JointLimits limits;
            SoftJointLimits softLimits;
            getJointLimits(joint_names_[i], nh_, limits);
            PositionJointSoftLimitsHandle jointLimitsHandle(jointPositionHandle, limits, softLimits);
            positionJointSoftLimitsInterface.registerHandle(jointLimitsHandle);
            position_joint_interface_.registerHandle(jointPositionHandle);

            // Create effort joint interface
            JointHandle jointEffortHandle(jointStateHandle, &joint_effort_command_[i]);
            effort_joint_interface_.registerHandle(jointEffortHandle);
        }

        registerInterface(&joint_state_interface_);
        registerInterface(&position_joint_interface_);
        registerInterface(&effort_joint_interface_);
        registerInterface(&positionJointSoftLimitsInterface);
    }

    void VAWTHardwareInterface::update(const ros::TimerEvent& e) {
        elapsed_time_ = ros::Duration(e.current_real - e.last_real);
        read();
        controller_manager_->update(ros::Time::now(), elapsed_time_);
        write(elapsed_time_);
    }

    void VAWTHardwareInterface::read() {
        for (int i = 0; i < num_joints_; i++) {
            joint_position_[i] = servo_position_[i];
        }
    }

    void VAWTHardwareInterface::write(ros::Duration elapsed_time) {
        positionJointSoftLimitsInterface.enforceLimits(elapsed_time);
        for (int i = 0; i < num_joints_; i++) {
        /*
            std_msgs::Float32 msg;
            double command = joint_position_command_[i];
            msg.data = command;
            servo_position_[i] = command;
            command_publisher.publish(msg);
          */
            ROS_INFO("Got %s joint_position_command_ %f joint_effort_command_[i] %f", joint_names_[i].c_str(), joint_position_command_[i],joint_effort_command_[i]);
        }
    }
}