
#include <ros_control_boilerplate/generic_hw_control_loop.h>
#include <vawt_robot_control/vawt_robot_hw_interface.h>
// previous hardware interface
//#include <vawt_hardware_interface/vawt_hardware_interface.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "vawt_robot_hw_interface");
  ros::NodeHandle nh;

  // NOTE: We run the ROS loop in a separate thread as external calls such
  // as service callbacks to load controllers can block the (main) control loop
  ros::AsyncSpinner spinner(3);
  spinner.start();

  // Create the hardware interface specific to your robot
  boost::shared_ptr<vawt_robot_control::VawtRobotHWInterface> vawt_robot_hw_interface
    (new vawt_robot_control::VawtRobotHWInterface(nh));
  vawt_robot_hw_interface->init();
//    vawt_hardware_interface::VAWTHardwareInterface vhi(nh);


  // Start the control loop
  ros_control_boilerplate::GenericHWControlLoop control_loop(nh, vawt_robot_hw_interface);
  control_loop.run(); // Blocks until shutdown signal recieved

  return 0;
}
