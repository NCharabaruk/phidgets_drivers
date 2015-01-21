#include "phidgets_mc/mc_ros_i.h"

namespace phidgets {

MCRosI::MCRosI(ros::NodeHandle nh, ros::NodeHandle nh_private):
  MC(),
  nh_(nh), 
  nh_private_(nh_private)
{
  ROS_INFO ("Starting Phidgets IR");

  initDevice();
}

void MCRosI::initDevice()
{
	ROS_INFO("Opening device");
	open(-1);

	ROS_INFO("Waiting for motor controller to be attached...");
	int result = waitForAttachment(10000);
	if(result)
	{
	  const char *err;
		CPhidget_getErrorDescription(result, &err);
		ROS_FATAL("Problem waiting for motor controller attachment: %s", err);
	}
}

void MCRosI::updateHandler(int index, int positionChange)
{
  MC::updateHandler(index, position);
}

} // namespace phidgets

