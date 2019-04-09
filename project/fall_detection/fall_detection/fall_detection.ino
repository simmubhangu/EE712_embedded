#include "Wire.h"
// I2Cdev and HMC5883L must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include "HMC5883L.h"
#include "MPU6050.h"
#include "helper_3dmath.h"
#include<SoftwareSerial.h>

SoftwareSerial bt(0,1); // uart0 of nano

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
int angle_threshold = 75;
int angle_threshold_n = -75;
char *fsm_state = "state_0";

float final_accumulative_value,final_roll,final_pitch;

//#define RAD_TO_DEG 57.2957786
int counter;

void timer1_init(void)
{
  TIMSK1 =0x01;   //enable interrupts 1
  
  TCCR1A =0xc0;  // Timer 1 mode selection
  TCCR1B =0x04;    //Timer 1 mode selection, prescalar selection
  TCNT1H = 0xff ;  //Counter higher 8 bit value
  TCNT1L = 0x00 ;  //Counter lower 8 bit value
  TCCR1B =TCCR1B|0x40;   //start Timer
}

ISR(TIMER1_OVF_vect)        // interrupt service routine 
{ 
  TCNT1H = 0xff ;  //Counter higher 8 bit value
  TCNT1L = 0x00 ;  //Counter lower 8 bit value
  counter++;
  //Serial.println(counter);
  //fall_detection(final_accumulative_value,final_roll,final_pitch);
}


void setup()
{
  Serial.begin(9600);
  timer1_init();

  // If you have GY-86 or GY-87 module.
  // To access HMC5883L you need to disable the I2C Master Mode and Sleep Mode, and enable I2C Bypass Mode

  while(!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_16G))
  {
   // Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
    delay(500);
  }

  mpu.setI2CMasterModeEnabled(false);
  mpu.setI2CBypassEnabled(true) ;
  mpu.setSleepEnabled(false);

  // Initialize Initialize HMC5883L
  //Serial.println("Initialize HMC5883L");
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

    final_accumulative_value= sqrt(acc[0]*acc[0] + acc[1]*acc[1]+acc[2]*acc[2]);

     ////////////////// filter data /////////////////////
    
    filter_acc = lowpassfilter(acc,20);
    filter_gyro = highpassfilter(gyro, 20);
    Serial.print("Raw_acc_X: "); Serial.print(acc[0]); Serial.print("  ");
    Serial.print("Raw_acc_Y: "); Serial.print(acc[1]); Serial.print("  ");
    Serial.print("Raw_acc_Y: "); Serial.print(acc[2]); Serial.print("  ");
    Serial.print("Raw_gyro_X: "); Serial.print(gyro[0]); Serial.print("  ");
    Serial.print("Raw_gyro_Y: "); Serial.print(gyro[1]); Serial.print("  ");
    Serial.print("Raw_gyro_Y: "); Serial.print(gyro[2]); Serial.print("  ");
    Serial.print("Raw_acc_X_f: "); Serial.print(filter_acc[0]); Serial.print("  ");
    Serial.print("Raw_acc_Y_f: "); Serial.print(filter_acc[1]); Serial.print("  ");
    Serial.print("Raw_acc_Y_f: "); Serial.print(filter_acc[2]); Serial.print("  ");
    Serial.print("Raw_gyro_X_f: "); Serial.print(filter_gyro[0]); Serial.print("  ");
    Serial.print("Raw_gyro_Y_f: "); Serial.print(filter_gyro[1]); Serial.print("  ");
    Serial.print("Raw_gyro_Y_f: "); Serial.print(filter_gyro[2]); Serial.print("  ");
    //Serial.println("uT");
    
    final_pitch = ComplementaryFilter_pitch(filter_acc[0],filter_acc[1],filter_acc[2],filter_gyro[0],filter_gyro[1],filter_gyro[2]);
    final_roll = ComplementaryFilter_roll(filter_acc[0],filter_acc[1],filter_acc[2],filter_gyro[0],filter_gyro[1],filter_gyro[2]);
 
 //Serial.println(final_accumulative_value);
 Serial.print("final: "); Serial.print(final_accumulative_value); Serial.print("  ");
 Serial.print("Pitch: "); Serial.print(final_pitch); Serial.print("  ");
 Serial.print("roll: "); Serial.print(final_roll); Serial.print("  ");
 Serial.println(" ");
    //fall_detection(final_accumulative_value,final_roll,final_pitch);
      
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
   //Serial.println(roll);
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

bool check_direction(float angle)
{
  for(int j=0;j<=10;j++)
    {
      if (angle <0)
      {
        return 0;
      }
      else
      
       return 1;
     delay(10);
    }
}

void fall_detection()
{
  //Serial.println(fsm_state);
  if (final_accumulative_value >= 250 && fsm_state =="state_0")
  {
    Serial.println("state_1");
    fsm_state = "state_1";
    counter=0;
    do{
      get_MPU_data();
      Serial.println(final_roll);
    }
    while(counter <= 500);   
      
    Serial.println(final_roll);
    //Serial.println(unsigned(roll_angle));
    Serial.println(final_pitch);
  }
  if ((fsm_state == "state_1") && ((angle_threshold) < final_roll))
  {
    Serial.println("state_2");
    fsm_state = "state_2";

    bool roll_dir = check_direction(final_roll);
    if(roll_dir ==0)
      {
        Serial.println("forward fall");  
      }
    else
    { 
      Serial.println("back fall");  
    }
  }
  
  if ((fsm_state == "state_1") && ((angle_threshold) < (final_pitch)))
  {
    Serial.println("state_2");
    fsm_state = "state_2";
    bool pitch_dir = check_direction(final_pitch);
    if(pitch_dir ==0)
      {
        Serial.println("right fall");  
      }
    else
    {
      Serial.println("left fall");
    }
  }
    if ((fsm_state == "state_1") && ((angle_threshold_n) > final_roll))
  {
    Serial.println("state_2");
    fsm_state = "state_2";

    bool roll_dir = check_direction(final_roll);
    if(roll_dir ==0)
      {
        Serial.println("forward fall");  
      }
    else
    { 
      Serial.println("back fall");  
    }
  }
  
  if ((fsm_state == "state_1") && ((angle_threshold_n) > (final_pitch)))
  {
    Serial.println("state_2");
    fsm_state = "state_2";
    bool pitch_dir = check_direction(final_pitch);
    if(pitch_dir ==0)
      {
        Serial.println("right fall");  
      }
    else
    {
      Serial.println("left fall");
    }
  }
  else
  {
    fsm_state = "state_0";
  }

}


void loop()
{
  get_MPU_data();
  //fall_detection();
  
}
