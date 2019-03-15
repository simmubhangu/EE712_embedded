#include <ros.h>
#include <limits.h>
#include <PID_v1.h>
#include <std_msgs/Int64.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/Range.h>


#include <ros/time.h>
#define left_b 25
#define left_f 24
#define right_b 23
#define right_f 22
#define pwm_right 7
#define pwm_left 6

static char global_m1a;
 static char global_m2a;
 static char global_m1b;
 static char global_m2b;

 static int global_counts_m1;
 static int global_counts_m2;
 volatile long global_counts_test_m1;
 volatile long global_counts_test_m2;

 static char global_last_m1a_val;
 static char global_last_m2a_val;
 static char global_last_m1b_val;
 static char global_last_m2b_val;
 unsigned long task1_time, task2_time;
unsigned long interval1=500;
unsigned long interval2=50;
float speed_m1 =0 , speed_m2 =0 ;
    static int8_t lookup_table[] = {0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0};
   static uint8_t enc1_val = 0;
   static uint8_t enc2_val = 0;

#define Left_Encoder_PinA 19
#define Left_Encoder_PinB 18
#define Right_Encoder_PinA 2
#define Right_Encoder_PinB 3




//int a = 0;
//const int analog_pin = 11;
//unsigned long range_timer;
//char frameid[] = "/ir_ranger";

float motor_left_speed = 0;
float motor_right_speed = 0;
float left_value = 0;
float right_value = 0;

ros::NodeHandle  nh;
std_msgs::Int64 int_msg;
ros::Publisher leftmotor("lwheel", &int_msg);
ros::Publisher rightmotor("rwheel", &int_msg);
//sensor_msgs::Range range_msg;
//ros::Publisher pub_range( "range_data", &range_msg);

void setup_motor()
{
    //Left motor
 pinMode(left_f,OUTPUT);
 pinMode(left_b,OUTPUT); 
 

 //Right Motor
 pinMode(right_f,OUTPUT);
pinMode(right_b,OUTPUT);
//pwm_define
  pinMode (pwm_right ,OUTPUT);
  pinMode (pwm_left ,OUTPUT);

}
 void encoder_init()
 {
   cli();

  pinMode(Left_Encoder_PinA, INPUT_PULLUP);
  pinMode(Right_Encoder_PinA, INPUT_PULLUP);
  pinMode(Left_Encoder_PinB, INPUT_PULLUP);
  pinMode(Right_Encoder_PinB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(Right_Encoder_PinA), do_Right_Encoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt (Left_Encoder_PinA), do_Left_Encoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(Right_Encoder_PinB), do_Right_Encoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt (Left_Encoder_PinB), do_Left_Encoder, CHANGE);

   // initialize the global state
   global_counts_m1 = 0;
   global_counts_m2 = 0;
   enc1_val = enc1_val | ((PINE & 0x30) >> 4);      //right encoder value
   enc2_val = enc2_val | ((PIND & 0b1100) >> 2 );  //left encoder value
    
   sei();
 }

 void do_Right_Encoder()
 {
   
     enc1_val = enc1_val << 2;
    enc1_val = enc1_val | ((PINE & 0x30) >> 4);
   
    global_counts_m1 = global_counts_m1 + lookup_table[enc1_val & 0b1111];
    global_counts_test_m1 = global_counts_test_m1 + lookup_table[enc1_val & 0b1111];

 }


 void do_Left_Encoder()
 {
   
    enc2_val = enc2_val << 2;
    enc2_val = enc2_val | ((PIND & 0b1100) >> 2) ;
   
  global_counts_m2 = global_counts_m2 + lookup_table[enc2_val & 0b1111];
  global_counts_test_m2 = global_counts_test_m2 + lookup_table[enc2_val & 0b1111];

 } 

 int getCountsM1()
 {
   cli();
   int tmp = global_counts_m1;
   sei();
   return tmp;
 }

 int getCountsM2()
 {
   cli();
   int tmp = global_counts_m2;
   sei();
   return tmp;
 }

 int getCountsAndResetM1()
  {
    cli();
      int tmp = global_counts_test_m1;
      global_counts_test_m1 = 0;
    sei();
    return tmp;
  }

  int getCountsAndResetM2()
  {
    cli();
      int tmp = global_counts_test_m2;
      global_counts_test_m2 = 0;
    sei();
    return tmp;
  }


 
void left_movement( const std_msgs::Float32 &data)
{
  left_value = data.data;
  
}
void right_movement(const std_msgs::Float32 &data)
{
 right_value = data.data;
  
}

ros::Subscriber<std_msgs::Float32> lwheel_sub("left_wheel_speed", &left_movement );
ros::Subscriber<std_msgs::Float32> rwheel_sub("right_wheel_speed", &right_movement);

double LeftMotorSetpoint=0.0, LeftMotorInput, LeftMotorOutput;//motor m2
double RightMotorSetpoint=0.0, RightMotorInput, RightMotorOutput;//motor m2
double LeftMotorKp=1.5, LeftMotorKi=1.0, LeftMotorKd=0.00;
double RightMotorKp=1.6, RightMotorKi=1.0, RightMotorKd=0.00;

PID LeftMotorPID(&LeftMotorInput, &LeftMotorOutput, &LeftMotorSetpoint, LeftMotorKp, LeftMotorKi, LeftMotorKd, DIRECT);
PID RightMotorPID(&RightMotorInput, &RightMotorOutput, &RightMotorSetpoint, RightMotorKp, RightMotorKi, RightMotorKd, DIRECT);
   

void setup()
{
  pinMode (right_b ,OUTPUT);
  pinMode (right_f ,OUTPUT);
  pinMode (left_b ,OUTPUT);
  pinMode (left_f ,OUTPUT);
  pinMode (pwm_right ,OUTPUT);
  pinMode (pwm_left ,OUTPUT);
  
    LeftMotorSetpoint = 0;
    LeftMotorPID.SetOutputLimits(-255,255);
    LeftMotorPID.SetMode(AUTOMATIC);
    RightMotorSetpoint = 0;
    RightMotorPID.SetOutputLimits(-255,255);
    RightMotorPID.SetMode(AUTOMATIC);
    
   encoder_init();
   nh.initNode();
  // setupSharp();
   nh.subscribe(lwheel_sub);
   nh.subscribe(rwheel_sub);
   nh.advertise(leftmotor);
   nh.advertise(rightmotor);

  
}
int speedToRPM(float speed)
{
    int rpm=0;
  rpm = (int)(speed * 293.707);
  return rpm;
}
void Update_Motors()
{
  
  moverightMotor(right_value);
  moveleftMotor(left_value);
  
}
void moveleftMotor(float leftmotorValue)
{
 // int right_pwm = (abs(rightServoValue));
  if (leftmotorValue>0)
  {
 //int  leftmotorValueMap=  map(leftmotorValue,0,255,20,255);
 digitalWrite(left_f,HIGH);
 digitalWrite(left_b,LOW);
//analogWrite(pwm_left,(right_pwm + 30));
analogWrite(pwm_left,leftmotorValue);    
  }
  else if(leftmotorValue<0)
  {
   // int  leftmotorValueMap=  map(abs(leftmotorValue),0,255,20,255);
 digitalWrite(left_f,LOW);
 digitalWrite(left_b,HIGH);
 //analogWrite(pwm_left,(right_pwm + 30));
 analogWrite(pwm_left,abs(leftmotorValue)); 
 
  }
  
  else if(leftmotorValue == 0)
  {
 digitalWrite(left_f,LOW);
 digitalWrite(left_b,LOW);
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void moverightMotor(float rightmotorValue)
{
 //int left_pwm = (abs(leftServoValue)); 
 if (rightmotorValue > 0)
  {
  //  int  rightmotorValueMap=  map(rightmotorValue,0,255,20,255);

digitalWrite(right_b,LOW);
digitalWrite(right_f,HIGH);
 //analogWrite(pwm_right,(left_pwm+ 30));
  analogWrite(pwm_right,rightmotorValue);
  }
  else if(rightmotorValue < 0)
  {
    //int  rightmotorValueMap=  map(abs(rightmotorValue),0,255,20,255); 
 digitalWrite(right_b,HIGH);
 digitalWrite(right_f,LOW);
 //analogWrite(pwm_right,(left_pwm +30));
 analogWrite(pwm_right,abs(rightmotorValue)); 

  }
  else if(rightmotorValue == 0)
  {

   digitalWrite(right_f,LOW);
   digitalWrite(right_b,LOW);
  
   }  
  
}
int Update_Encoders()
{
  return global_counts_m2;
}
int Update_Encoders2()
{
  return global_counts_m1;
}

void pid_computation()
{

if ((millis() - task2_time) >= interval2)
       {
          int current_count_m1, current_count_m2;
          task2_time = millis();
          current_count_m1 = getCountsAndResetM1();
          current_count_m2 = getCountsAndResetM2();
          speed_m1 = (current_count_m1 * 1000.0 * 60)/(1680*interval2);
          speed_m2 = (current_count_m2 * 1000.0 * 60)/(1680*interval2);
       }
  
    if ((abs(left_value) > 0) && (abs(right_value) > 0))
    {
          LeftMotorSetpoint = speedToRPM(left_value); //in rpm
          RightMotorSetpoint = speedToRPM(right_value); //in rpm
          LeftMotorInput = speed_m2;
          LeftMotorPID.Compute(); 
          int speed_out_left =LeftMotorOutput;
          moveleftMotor(speed_out_left);    
          RightMotorInput = speed_m1;
          RightMotorPID.Compute();    
          int speed_out_right =RightMotorOutput;
          moverightMotor(speed_out_right);
    }
    else
    {
        moveleftMotor(0);
        moverightMotor(0);
    }     


  
}
void loop()
{
  int_msg.data = Update_Encoders();
  leftmotor.publish( &int_msg );
  int_msg.data = Update_Encoders2();
  rightmotor.publish( &int_msg );
//  publishsharp();
  nh.spinOnce();
  delay(100);
    pid_computation();
}
