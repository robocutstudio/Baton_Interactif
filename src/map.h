/* projct:  Baton_Interactif
   authrs:  vince (vincent@robocutstudio.om)
   target:  TinyPICO
   hardwr:  BEATS_PCB w/ IMU Bno055
*/


#ifndef MAP_H
#define MAP_H

float mapf(float x, float in_min, float in_max, float out_min, float out_max);

float getOrientationX(float val);
float getOrientationY(float val);
float getOrientationZ(float val);

float getAccelX(float val);
float getAccelY(float val);
float getAccelZ(float val);

float getGyroX(float val);
float getGyroY(float val);
float getGyroZ(float val);



#endif //END MAP_H
