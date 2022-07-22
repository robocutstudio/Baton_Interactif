/* projct:  Baton_Interactif
   authrs:  vince (vincent@robocutstudio.om)
   target:  TinyPICO
   hardwr:  BEATS_PCB w/ IMU Bno055
*/

#include "map.h"

//rework of "map" to use float
float mapf(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

/*************************************************/
/*
        Orientation
*/
/*************************************************/
float getOrientationX(float val) {
  float ret;
  if (val >= 180) {
    ret = mapf(val, 180, 360, 0, 1);
  } else {
    ret = mapf(val, 0, 180, 1, 0);
  }
  return ret;
}

float getOrientationY(float val) {
  float ret;
  if (val >= 0) {
    ret = mapf(val, 0, 90, 0, 1);
  } else {
    ret = mapf(val, -90, 0, 1, 0);
  }
  return ret;
}

float getOrientationZ(float val) {
  float ret;
  if (val >= 0) {
    ret = mapf(val, 0, 180, 0, 1);
    
  } else {
    ret = mapf(val, -180, 0, 1, 0);
  }
  return ret;
}


/*************************************************/
/*
        ACCELEROMETER
*/
/*************************************************/

float trueAccelX;
float getAccelX(float val) {
  float ret;
  trueAccelX = mapf(val, -50, 50, 0, 100);
  if (trueAccelX > 53) {
    ret = mapf(trueAccelX, 50, 100, 0, 1);
  } else if (trueAccelX < 47) {
    ret = mapf(trueAccelX, 0, 50, 0, 1);
  } else {
    ret = 0.00;
  }
  return ret;
}

float trueAccelY;
float getAccelY(float val) {
  float ret;
  trueAccelY = mapf(val, -50, 50, 0, 100);
  if (trueAccelY > 53) {
    ret = mapf(trueAccelY, 50, 100, 0, 1);
  } else if (trueAccelY < 47) {
    ret = mapf(trueAccelY, 0, 50, 0, 1);
  } else {
    ret = 0.00;
  }
  return ret;
}

float trueAccelZ;
float getAccelZ(float val) {
  float ret;
  trueAccelZ = mapf(val, -50, 50, 0, 100);
  if (trueAccelZ > 53) {
    ret = mapf(trueAccelZ, 50, 100, 0, 1);
  } else if (trueAccelZ < 47) {
    ret = mapf(trueAccelZ, 0, 50, 0, 1);
  } else {
    ret = 0.00;
  }
  return ret;
}


/*************************************************/
/*
        Gyroscope
*/
/*************************************************/
float getGyroX(float val) {
  float ret;
  if (val > 5) {
    ret = mapf(val, 0, 500, 0, 1);
  }
  else if (val < -5) {
    ret = mapf(val, -500, 0, 1, 0);
  } else {
    ret = 0.00;
  }
  return ret;
}

float getGyroY(float val) {
  float ret;
  if (val >= 5) {
    ret = mapf(val, 0, 500, 0, 1);
  }
  else if (val < -5) {
    ret = mapf(val, -500, 0, 1, 0);
  } else {
    ret = 0.00;
  }
  return ret;
}

float getGyroZ(float val) {
  float ret;
  if (val >= 5) {
    ret = mapf(val, 0, 500, 0, 1);
  }
  else if (val < -5) {
    ret = mapf(val, -500, 0, 1, 0);
  } else {
    ret = 0.00;
  }
  return ret;
}
