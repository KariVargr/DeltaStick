#ifndef DELTACAL_h
#define DELTACAL_h

#include "Arduino.h"

class DELTACAL
{
  public:
  void setup(float e, float f, float re, float rf);

  int ForwardKinematic(float theta1, float theta2, float theta3, float &x0, float &y0, float &z0);
  int InverseKinematic(float x0, float y0, float z0, float &theta1, float &theta2, float &theta3);

  private:
  int calcAngleYZ(float x0, float y0, float z0, float &theta);

};
#endif

