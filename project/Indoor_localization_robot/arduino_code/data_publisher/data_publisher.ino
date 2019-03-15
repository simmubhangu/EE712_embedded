#include <ros.h>
#include <limits.h>
#include <PID_v1.h>
#include <std_msgs/Int64.h>
#include <std_msgs/Int16.h>
#include <sensor_msgs/Range.h>


#include <ros/time.h>
#define left_b 27
#define left_f 26
#define right_b 25
#define right_f 24
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

int tick_per_rev = 2240;



//int a = 0;
//const int analog_pin = 11;
//unsigned long range_timer;
//char frameid[] = "/ir_ranger";

float motor_left_speed = 0;
float motor_right_speed = 0;
int k_data = 0;
int right_value = 0;

ros::NodeHandle  nh;
//nh.setBaud(57600);
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


 
void key_data( const std_msgs::Int16 &data)
{
  k_data = data.data;
  
}
//void right_movement(const std_msgs::Int16 &data)
//{
// right_value = data.data;
//  
//}

ros::Subscriber<std_msgs::Int16> lwheel_sub("/input_key", &key_data );
//ros::Subscriber<std_msgs::Int8> rwheel_sub("right_wheel_speed", &right_movement);

void setup()
{
  pinMode (right_b ,OUTPUT);
  pinMode (right_f ,OUTPUT);
  pinMode (left_b ,OUTPUT);
  pinMode (left_f ,OUTPUT);
  pinMode (pwm_right ,OUTPUT);
  pinMode (pwm_left ,OUTPUT);
 
   encoder_init();
   nh.initNode();
  // setupSharp();
   nh.subscribe(lwheel_sub);
//   nh.subscribe(rwheel_sub);
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
  
  motor_control(k_data);
//  moveleftMotor(left_value);
  
}


void forward(int l_value, int r_value)
{
    digitalWrite(left_f,HIGH);
    digitalWrite(left_b,LOW);
    //analogWrite(pwm_left,(right_pwm + 30));
    analogWrite(pwm_left,l_value); 
    digitalWrite(right_b,LOW);
    digitalWrite(right_f,HIGH);
    //analogWrite(pwm_right,(left_pwm+ 30));
    analogWrite(pwm_right,r_value);
}

void backward(int l_value, int r_value)
{
    digitalWrite(left_f,LOW);
    digitalWrite(left_b,HIGH);
    //analogWrite(pwm_left,(right_pwm + 30));
    analogWrite(pwm_left,l_value); 
    digitalWrite(right_b,HIGH);
    digitalWrite(right_f,LOW);
    //analogWrite(pwm_right,(left_pwm+ 30));
    analogWrite(pwm_right,r_value);
}
void left(int l_value, int r_value)
{
    digitalWrite(left_f,LOW);
    digitalWrite(left_b,HIGH);
    //analogWrite(pwm_left,(right_pwm + 30));
    analogWrite(pwm_left,l_value); 
    digitalWrite(right_b,LOW);
    digitalWrite(right_f,HIGH);
    //analogWrite(pwm_right,(left_pwm+ 30));
    analogWrite(pwm_right,r_value);
}
void right(int l_value, int r_value)
{
    digitalWrite(left_f,HIGH);
    digitalWrite(left_b,LOW);
    //analogWrite(pwm_left,(right_pwm + 30));
    analogWrite(pwm_left,l_value); 
    digitalWrite(right_b,HIGH);
    digitalWrite(right_f,LOW);
    //analogWrite(pwm_right,(left_pwm+ 30));
    analogWrite(pwm_right,r_value);
}
void stop_robot()
{
    digitalWrite(left_f,LOW);
    digitalWrite(left_b,LOW);
    //analogWrite(pwm_left,(right_pwm + 30));
    analogWrite(pwm_left,0); 
    digitalWrite(right_b,LOW);
    digitalWrite(right_f,LOW);
    //analogWrite(pwm_right,(left_pwm+ 30));
    analogWrite(pwm_right,0);
}



////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void motor_control(int k_data)
{
 //int left_pwm = (abs(leftServoValue)); 
 if (k_data == 20)
  {
    forward(0,0);
  }
  else if(k_data == 10)
  {
   forward(255,255);
  }
  else if(k_data == 30)
  {
    left(255,255);
  }
  else if(k_data == 40)
  {
    right(255,255);  
  }
  else if(k_data == 110)
  {
    backward(255,255);
  }
  else if(k_data == 100)
  {
    forward(80,255);  
  }
    else if(k_data == 120)
  {
    forward(255,80);  
  }
  else
  {
    stop_robot();
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


void loop()
{
  int_msg.data = Update_Encoders();
  leftmotor.publish( &int_msg );
  int_msg.data = Update_Encoders2();
  rightmotor.publish( &int_msg );
  Update_Motors();
//  publishsharp();
  nh.spinOnce();
  delay(100);
    
}
