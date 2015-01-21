#include "phidgets_api/mc.h"

namespace phidgets {

MC::MC():
  Phidget(),
  mc_handle_(0)
{
  // create the handle
  CPhidgetMotorControl_create(&mc_handle_);

  // pass handle to base class
  Phidget::init((CPhidgetHandle)mc_handle_);

  // register base class callbacks
  Phidget::registerHandlers();
  
  // register encoder position callback
	CPhidgetMotorControl_set_OnEncoderPositionUpdate_Handler(mc_handle_, EncoderPositionUpdateHandler, this);
}

void setVelocity(int index, double velocity)
{
  CPhidgetMotorControl_setVelocity(mc_handle_, index, velocity);
}

void setBraking(int index, double braking)
{
  CPhidgetMotorControl_setBraking(mc_handle_, index, braking);
}

int Imu::SpatialDataHandler(CPhidgetSpatialHandle handle, void *userptr, CPhidgetSpatial_SpatialEventDataHandle *data, int count)
{
  ((Imu*)userptr)->dataHandler(data, count);
  return 0;
}

void Imu::dataHandler(CPhidgetSpatial_SpatialEventDataHandle *data, int count)
{
  printf("Empty data handler\n");
}

int MC::EncoderPositionUpdateHandler(CPhidgetMotorControlHandle motor, void *userPtr, int index, int positionChange)
{
  ((MC*)userptr)->updateHandler(index, positionChange);
}

void updateHandler(int index, int positionChange)
{
  printf("The encoder position is ", positionChange);
}
