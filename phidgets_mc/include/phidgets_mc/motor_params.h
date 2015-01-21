#ifndef PHIDGETS_MC_MC_ROS_I_H
#define PHIDGETS_MC_MC_ROS_I_H

#include <ros/ros.h>
#include <phidgets_api/mc.h>

namespace phidgets {

class MCRosI : public MC
{

  public:

    IRRosI(ros::NodeHandle nh, ros::NodeHandle nh_private);

  private:

    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    void initDevice();
    void updateHandler(int index, int positionChange);
};

} //namespace phidgets

#endif // PHIDGETS_MC_MC_ROS_I_H

Phidgets Common
CPhidget_set_OnAttach_Handler(CPhidgetHandle phid, int( *fptr)(CPhidgetHandle phid, void *userPtr), void *userPtr);
CPhidget_set_OnDetach_Handler(CPhidgetHandle phid, int( *fptr)(CPhidgetHandle phid, void *userPtr), void *userPtr);
