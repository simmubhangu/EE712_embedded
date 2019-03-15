
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
int left_PWM = 6;
int left_forward = 26;
int left_backward = 27;

int right_PWM = 7;
int right_forward = 24;
int right_backward = 25;


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
   int tmp = global_counts_m1;
   global_counts_m1 = 0;
   sei();
   return tmp;
 }

 int getCountsAndResetM2()
 {
   cli();
   int tmp = global_counts_m2;
   global_counts_m2 = 0;
   sei();
   return tmp;
 }

 void motor_init()
 {
     pinMode(right_PWM, OUTPUT);
     pinMode(right_forward, OUTPUT);
     pinMode(right_backward, OUTPUT);

     pinMode(left_PWM, OUTPUT);
     pinMode(left_forward, OUTPUT);
     pinMode(left_backward, OUTPUT);

 }

 void backward()
  {
    digitalWrite(right_forward, HIGH);
    digitalWrite(right_backward, LOW);

    digitalWrite(left_forward, HIGH);
    digitalWrite(left_backward, LOW);
    
  }

  void forward()
  {
    digitalWrite(right_forward, LOW);
    digitalWrite(right_backward, HIGH);

    digitalWrite(left_forward, LOW);
    digitalWrite(left_backward, HIGH);
    
  }

void velocity(int l_wheel, int r_wheel)
{
analogWrite(right_PWM, r_wheel);
analogWrite(left_PWM, l_wheel);
  
}
  
 void setup()
 {
   encoder_init();
   motor_init();
   Serial.begin(57600);
   while (!Serial) {
     ; // wait for Serial1 port to connect. Needed for native USB port only
   }
   
 }

 void loop()
 {
    velocity(100,100);
    forward();
    delay(1000);
    velocity(120,120);
    backward();
    delay(1000);
    
     Serial.print("EncoderA: ");
     Serial.println(global_counts_m1);

    Serial.print("EncoderB ");
    Serial.println(global_counts_m2);

    delay(1000);

    //  Serial.print("PIC0: ");
    //  Serial.println((PINJ & (1<<0)),HEX);

    //    Serial.print("PIC1: ");
    //   Serial.println((PINJ & (1<<1)), HEX);


 }
