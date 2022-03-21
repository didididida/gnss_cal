#include "ekf.h"




// bound angle between -180 and 180
float ekfNav::constrainAngle180(float dta) {
  if(dta >  M_PI) dta -= (M_PI*2.0f);
  if(dta < -M_PI) dta += (M_PI*2.0f);
  return dta;
}

// bound angle between 0 and 360
float ekfNav::constrainAngle360(float dta){
  dta = fmod(dta,2.0f*M_PI);
  if (dta < 0)
    dta += 2.0f*M_PI;
  return dta;
}