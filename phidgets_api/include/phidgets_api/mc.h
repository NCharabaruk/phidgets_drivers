#ifndef PHIDGETS_API_MC_H
#define PHIDGETS_API_MC_H

#include "phidgets_api/phidget.h"

namespace phidgets 
{
  class MC: public Phidget
  {
    public:
    
      MC();
      
    protected:
  
      CPhidgetMotorControlHandle mc_handle_;
      
      void setVelocity(int index, double velocity);
      
      void setBraking(int index, double braking);
      
      virtual void updateHandler(int index, int positionChange);
      
    private:
    static int EncoderPositionUpdateHandler(CPhidgetMotorControlHandle motor, void *userPtr, int index, int positionChange);
  };

} //namespace phidgets

#endif // PHIDGETS_API_IMU_H
