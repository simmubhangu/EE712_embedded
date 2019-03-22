/*
  HMC5883L Triple Axis Digital Compass + MPU6050 (GY-86 / GY-87). Compass Example.
  Read more: http://www.jarzebski.pl/arduino/czujniki-i-sensory/3-osiowy-magnetometr-hmc5883l.html
  GIT: https://github.com/jarzebski/Arduino-HMC5883L
  Web: http://www.jarzebski.pl
  (c) 2014 by Korneliusz Jarzebski
*/

#include "Wire.h"

// I2Cdev and HMC5883L must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include "HMC5883L.h"
#include "MPU6050.h"
#include "helper_3dmath.h"

HMC5883L compass;
MPU6050 mpu;

#define delta_t .01
float pitch=0;
float pitchAcc;
float roll=0;
float rollAcc;
float P_CompCoeff= 0.5;
float yaw=0;
float output_[3];
float output_gyro[3];
float prev_output[3];
float prev_output_gyro[3];
float prev_input_gyro[3];
extern uint16_t cycleTime = 1000;
float acc[3];
float gyro[3];
float *filter_gyro;
float *filter_acc ;
char angle_threshold = 90;
char fsm_state = "state_0";
#define RAD_TO_DEG 57.2957786



void setup()
{
  Serial.begin(115200);

  // If you have GY-86 or GY-87 module.
  // To access HMC5883L you need to disable the I2C Master Mode and Sleep Mode, and enable I2C Bypass Mode

  while(!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_16G))
  {
    Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
    delay(500);
  }

  mpu.setI2CMasterModeEnabled(false);
  mpu.setI2CBypassEnabled(true) ;
  mpu.setSleepEnabled(false);

  // Initialize Initialize HMC5883L
  Serial.println("Initialize HMC5883L");
  while (!compass.begin())
  {
    Serial.println("Could not find a valid HMC5883L sensor, check wiring!");
    delay(500);
  }

  // Set measurement range
  compass.setRange(HMC5883L_RANGE_1_3GA);

  // Set measurement mode
  compass.setMeasurementMode(HMC5883L_CONTINOUS);

  // Set data rate
  compass.setDataRate(HMC5883L_DATARATE_30HZ);

  // Set number of samples averaged
  compass.setSamples(HMC5883L_SAMPLES_8);

  // Set calibration offset. See HMC5883L_calibration.ino
  compass.setOffset(0, 0); 
}
void get_MPU_data()
{
  Vector normAccel = mpu.readNormalizeAccel();
  Vector normgyro = mpu.readNormalizeGyro();

    acc[0] = normAccel.XAxis;
    acc[1] = normAccel.YAxis;
    acc[2] = normAccel.ZAxis;
    gyro[0] = normgyro.XAxis;
    gyro[1] = normgyro.YAxis;
    gyro[2] = normgyro.ZAxis;

//    float final_value= sqrt(acc[0]*acc[0] + acc[1]*acc[1]+acc[2]*acc[2]);
      //Serial.println(final_value-9.8);
     ////////////////// filter data /////////////////////
    
    filter_acc = lowpassfilter(acc,5);
    filter_gyro = highpassfilter(gyro, 5);
    
    
    float pitch = ComplementaryFilter_pitch(filter_acc[0],filter_acc[1],filter_acc[2],filter_gyro[0],filter_gyro[1],filter_gyro[2]);
    float roll = ComplementaryFilter_roll(filter_acc[0],filter_acc[1],filter_acc[2],filter_gyro[0],filter_gyro[1],filter_gyro[2]);

      
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////// 1st order complementry filter for pitch //////////////////////////////////
float ComplementaryFilter_pitch(float ax,float ay,float az,float gx,float gy,float gz) 
{
   float squaresum=sqrt(ay*ay+az*az);
   pitch+=((-gy/32.8f)*(delta_t/1000000.0f));
   //Serial.println(pitch);
   pitchAcc =atan(ax/squaresum)*RAD_TO_DEG;
   pitch =P_CompCoeff*pitch + (1.0f-P_CompCoeff)*pitchAcc;
   Serial.println(pitch);
  // delay(2)/;
   return pitch;
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////
float ComplementaryFilter_roll(float ax,float ay,float az,float gx,float gy,float gz) 
{
   float squaresum=sqrt(ax*ax+az*az);
   roll+=((-gx/32.8f)*(delta_t/1000000.0f));
   //Serial.println(pitch);
   rollAcc =atan(ay/sqrt(squaresum))*RAD_TO_DEG;
   roll =P_CompCoeff*roll + (1.0f-P_CompCoeff)*rollAcc;
   Serial.println(roll);
  // delay(2)/;
   return roll;
}

 
////////////////////////////////Low pass filter for accelerlometer /////////////////////////////////////////
float *lowpassfilter(float *input_, uint8_t f_cut) {
   
   float dT = (float)cycleTime * 0.000001f;
   float RC= 1.0f / ( 2.0f * (float)M_PI * f_cut );
   float alpha = dT /(RC +dT);
   for(int i=0;i<3;i++)
   {
    output_[i] = (*(input_+i)* alpha) + (1-alpha) * (prev_output[i]);
    prev_output[i]=output_[i];
    //Serial.println(output_[0]);
//    Serial.println(" "/);
   }
  float *ptr = output_;
  return ptr;
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////// high pass filter for gyroscope ///////////////////////////////////////////////
float *highpassfilter(float *input_gyro, uint8_t f_cut) {
   
   float dT = (float)cycleTime * 0.000001f;
   float RC= 1.0f / ( 2.0f * (float)M_PI * f_cut );
   float alpha = RC /(RC +dT);
   for(int i=0;i<3;i++)
   {
    output_gyro[i] = ((prev_output_gyro[i]* alpha) +  ((*(input_gyro+i) - prev_input_gyro[i]) * alpha)) ;
    prev_output_gyro[i]=output_gyro[i];
    prev_input_gyro[i]= *(input_gyro+i);
    //Serial.println(output_[0]);

   }
  float *pt = output_gyro;
  return pt;
}
void fall_detection(float peak_value, float angle)
{
  if (peak_value>=200 & fsm_state=="state_0")
  {
    Serial.println("state_1");
    fsm_state = "state_1"; 
    delay(100); 
  }

  if ((fsm_state == "state_1") & ((angle_threshold +20 )> angle > (angle_threshold-20)))
  {
    Serial.println("state_2");
    fsm_state = "state_2";
  }

  else
  {
    fsm_state = "state_0";
  }

}


void loop()
{
  get_MPU_data();
}
