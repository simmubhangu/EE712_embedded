


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

 void setup()
 {
     pinMode(right_PWM, OUTPUT);
     pinMode(right_forward, OUTPUT);
     pinMode(right_backward, OUTPUT);

     pinMode(left_PWM, OUTPUT);
     pinMode(left_forward, OUTPUT);
     pinMode(left_backward, OUTPUT);


   Serial.begin(57600);
   while (!Serial) {
     ; // wait for Serial1 port to connect. Needed for native USB port only
   }
   Serial.println("chal gya");
 }

unsigned long task1_time, task2_time;
unsigned long interval1=500;
unsigned long interval2=50;
unsigned long now;
unsigned long dt;
unsigned long task_start;
unsigned long task_end;
unsigned long task_dt;
 


 void loop()
 {

         int speed_abs =255;
         analogWrite(right_PWM, speed_abs);
         analogWrite(left_PWM, speed_abs);

         backward();
         delay(2000);
         analogWrite(right_PWM, speed_abs);
         analogWrite(left_PWM, speed_abs);

         forward();
         delay(2000);
 

 }
