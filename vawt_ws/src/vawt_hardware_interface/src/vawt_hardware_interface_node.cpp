#include <vawt_hardware_interface/vawt_hardware_interface.h>
#include <ros/callback_queue.h>
int main(int argc, char** argv)
{

    ros::init(argc, argv, "vawt_hardware_interface");
    ros::CallbackQueue ros_queue;

    ros::NodeHandle nh;
    nh.setCallbackQueue(&ros_queue);
    vawt_hardware_interface::VAWTHardwareInterface vhi(nh);

    ros::MultiThreadedSpinner spinner(0);
    spinner.spin(&ros_queue);
    return 0;
}