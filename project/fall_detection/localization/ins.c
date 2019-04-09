#include <stdio.h>
#include <stdlib.h>
#include <math.h>

// -----------------------------------------------------------------
// This package defines:
//
//    fnVectorNorm  : Calculates norm of a vector
//    fnDetectStill : Detect if the sensor is in still phase by
//                    comparing with threshold value
//    fnZuptVelocity: Estimate velocity by integrating the accelerometer
//                    readings
//
// -----------------------------------------------------------------

// threshold value to detect still phase (deg/sec)
float glStillPhaseThreshold = 0.01;

// Accelerator sampling duration (seconds)
float glASamplingDuration = 0.001;


// Computes the norm of a vector
float fnVectorNorm (float x, float y, float z) {
   float xSq = x * x;
   float ySq = y * y;
   float zSq = z * z;

   return (sqrtf (xSq+ySq+zSq));
}


// Detects still phase by comparing the gyro-norm against a threshold
int fnDetectStill (float gyroNorm) {
   if (gyroNorm < glStillPhaseThreshold) return (1);
   else return (0);
   
}


// Function to implement zero-update correction of the velocity values
// obtained by integrating the accelro output
float fnZuptVelocity (
      float ax, float ay, float az, float gx, float gy, float gz, float vt) {

   float gyroNormValue = fnVectorNorm (gx, gy, gz);
   float vnew = vt;
   if (fnDetectStill (gyroNormValue) == 1) return (0.0);
   else {
      vnew += glASamplingDuration * fnVectorNorm (ax, ay, az);
   }

   return (vnew);
}


