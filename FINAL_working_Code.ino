/*
A simple navigation code
*/

#define ENC_K 32
#define PINAR 2
#define PINBR 4
#define PINAL 21
#define PINBL 12

volatile long enc_countR;
volatile float enc_angleR;
volatile long enc_countL;
volatile float enc_angleL;
unsigned long t0=0;

//Variables (pins):
float x=10.2 ,y=20 ,z=80 ,q=54, w=5;
const int pinAI1 = 9;       // Pin allocation for AI1
const int pinAI2 = 6;       // Pin allocation for AI2
const int pinBI1 = 27;       // Pin allocation for AI1
const int pinBI2 = 29;       // Pin allocation for AI2
const int pinPWMA = 3;       // Pin allocation for the PWM pin
const int pinPWMB = 5;       // Pin allocation for the PWM pin
const int pinStandBy = 25;   // Pin allocation for the standby pin





const int pingPin_right = 13; // Trigger Pin of Ultrasonic Sensor
const int echoPin_right = 45; // Echo Pin of Ultrasonic Sensor


const int pingPin_left = 7; // Trigger Pin of Ultrasonic Sensor
const int echoPin_left = 53; // Echo Pin of Ultrasonic Sensor
float front=20;
float REF=7.7;
float right_s,NORM_SPEED_L=100,NORM_SPEED_R=100,K=2;
int speed_right;
int speed_left;
float error;


boolean AI1 = 0;            // AI1 pin value
boolean AI2 = 0;            // AI2 pin value
boolean BI1 = 0;            // AI1 pin value
boolean BI2 = 0;            // AI2 pin value
boolean standBy = 0;        // standBy pin Value

boolean rotDirect = 0;      // Rotation direction variable
unsigned char pwmValue = 0; // PWM value to be written to the output



//Robot arm

// import the servo library:
#include <Servo.h>
#include <EEPROM.h>


  
// create an object for a servo:
Servo S1;
Servo S2;
Servo S3;
// define the analog input pin that the potentiometer is 
// connected to, and the PWM output that the servo is 
// connected to:
int P1_pot = 0;
int P2_pot = 1;
int P3_pot = 2;
int S1_BASE = 46;
int S2_Elbow =11;
int S3_Wrist = 8;
int servo_1_PulseWidth;
int servo_2_PulseWidth;
int servo_3_PulseWidth;
// declare variables, for use later:
int P1_val;
int P2_val;
int P3_val;
int S1_BASE_angle;
int S2_Elbow_angle;
int S3_Wrist_angle;

int ref_1_PulseWidth=1481;
int ref_2_PulseWidth=1283;
int ref_3_PulseWidth=1396;

float BASE[45]= {1524,1524,1524,1524,1524,1524,1524,1524,1524,1524,1524,1524,1524,1524,1524,1524,1524,1321,1330,1397,1430,1467,1502,1538,1571,1600,1645,1671,1712,1743,1776,1776,1481,1524,1524};
float WRIST[45]={1896,1896,1896,1881,1910,1930,1955,1964,1981,2019,2045,2087,2116,2160,2189,2227,2255,2078,2089,2060,2026,2026,2026,2026,2026,2026,2028,2028,2026,2026,2028,2028,2000,2000,2000};
float ELBOW[45]={2000,2000,2000,2000,2016,2003,1990,1972,1972,1954,1930,1898,1878,1857,1821,1800,2100,2100,1828,1843,1856,1889,1880,1907,1906,1931,1901,1915,1931,1940,1936,2100,2100,2100,2100,2100};


float currentAngle=0;
float currentAngle2=0;
 float servo_3_RadAngle;
  float servo_2_RadAngle;
   float servo_1_RadAngle;


void setup()
{ 
  pinMode(pinStandBy, OUTPUT);
  pinMode(pinAI1, OUTPUT);
  pinMode(pinAI2, OUTPUT);
  pinMode(pinBI1, OUTPUT);
  pinMode(pinBI2, OUTPUT);
  pinMode(pinPWMA, OUTPUT);
  pinMode(pinPWMB, OUTPUT);

   pinMode(PINAR, INPUT);
  pinMode(PINBR, INPUT);
  pinMode(PINAL, INPUT);
  pinMode(PINBL, INPUT);
  attachInterrupt(digitalPinToInterrupt(PINAR), channelAR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PINBR), channelBR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PINAL), channelAL, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PINBL), channelBL, CHANGE);
  enc_countR=0;
  enc_angleR=0;
  enc_countL=0;
  enc_angleL=0;
  
  t0=0;

  standBy = true;
  digitalWrite(pinStandBy, standBy);

  delay(1000);

// 1896 initial position
  S1.writeMicroseconds(1524);
  S2.writeMicroseconds(2100);
  S3.writeMicroseconds(1696);


  S1.attach(S1_BASE);  
  S2.attach(S2_Elbow);
  S3.attach(S3_Wrist);

}


void loop()
{  Serial.begin(115200);
 Serial.println("I am waiting  ");
  delay(3000);

   Serial.print("HELLO  ");
  stop_wheels();
 Serial.print("HELLO  ");
 stop_wheels();
delay(5000);
forward(100,102);
delay(500);
stop_wheels();

  float d;

  bool tuuurn=0;
  enc_countL=0;
  while(tuuurn!=1)
  {
   Serial.print("I m turning right  ");
   tuuurn=TURN_90_Right();
  }
  
   Serial.print("I have finsihed turning right  ");
 //right_s=right_sensor(); // taking the first measurement
 forward(100,100);

 // GOING FORWARD AFTER INITAL TURN
  while(left_sensor()>18.5)  // might need changing  make it smaller
  {
    PID_TO_THE_WALL(9,8,100);
  //  d=right_sensor();
   // Serial.print("  distance to wall: ");
   // Serial.print(d);
  }
  // we 
  stop();
  
// WE NEED TO TURN TO THE LEFT
  tuuurn=0;
  enc_countR=0;
  enc_countL=0;
  delay(1000);
  while(tuuurn!=1)
  {
   Serial.print("Im turning Left  ");
   tuuurn=TURN_90_Left();
  }
  
   Serial.print("I have finished turning Left  ");
 //right_s=right_sensor(); // taking the first measurement
 forward(50,50 );

 

  // GOING THORUG THE TUNNEL

 while(left_sensor()>35.0)
  {
   PID_TO_THE_WALL(15.74,10,50);
  }
   stop_wheels();
   delay(100);
   S3.writeMicroseconds(1896);// PEN POSTITION TO PASS THE TUNNEL
   delay(100);
    Serial.print(" I HAVE MOVE ARM UP ");
  forward(50,50);
  while(left_sensor()>4.75) // 4.75 was working
  {
  //  Serial.print(" DISTANCE TO WALL ");
  // Serial.print(right_sensor()) ; 
   PID_TO_THE_WALL(15.74,10,50);

  }
  //STOPPING AT THE BOARD
  stop_wheels();


  //delay(6000);

  
  delay(1000);



int i=2;
for (i=2;i<=19;i++) 
{
   A_to_B( BASE[i-1], ELBOW[i-1], WRIST[i-1], BASE[i] , ELBOW[i], WRIST[i], 1000, 13);
   Serial.println(i);
}
//delay(2000);Fpi
for (i=23;i<=34;i++) // 23 before working
{
   A_to_B( BASE[i-1], ELBOW[i-1], WRIST[i-1], BASE[i] , ELBOW[i], WRIST[i], 1000, 13);
   Serial.println(i);
}



  delay(2000);

  go_back();
  delay(500);
  stop_wheels();
  //ROTATING TO THE LEFT
  tuuurn=0;
  enc_countL=0;
  while(tuuurn!=1)
  {
   Serial.print("I m turning Left  ");
   tuuurn=TURN_90_Left2();
  }
  
   Serial.print("I have finsihed turning Left  ");
 //right_s=right_sensor(); // taking the first measurement
 forward(100,100);
  // GOING FORWARD
  while(left_sensor()>19.5) // 19.5 before
  {
    
    PID_TO_THE_WALL(8,0,100);
  }
  // ROTATING TO THE LEFT AGAIN

  S3.writeMicroseconds(1696); // PEN POSTITION TO PASS THE TUNNEL
  tuuurn=0;
  enc_countL=0;
  while(tuuurn!=1)
  {
   Serial.print("I m turning Left  ");
   tuuurn=TURN_90_Left1();
  }
  
   Serial.print("I have finsihed turning Left  ");

  forward(100,100);
  while(left_sensor()>12)
  {
    PID_TO_THE_WALL(16.5,8,100);
  }
  stop_wheels();
  delay(1000);
  
  tuuurn=0;
  enc_countL=0;
  while(tuuurn!=1)
  {
   Serial.print("Im turning Left  ");
   tuuurn=TURN_180();
  }
  S3.writeMicroseconds(1896);

  stop_wheels();
  delay(100000);
}
  //main ends

  ///Functions:
  void rotate_left(){  
    digitalWrite(pinStandBy, true);
    digitalWrite(pinAI1,      true);
    digitalWrite(pinAI2,      false);
    digitalWrite(pinBI1,      true);
    digitalWrite(pinBI2,      false);
    analogWrite (pinPWMA, 50);
    analogWrite (pinPWMB, 50);
    }
  void rotate_right(){   // CORRECT
    digitalWrite(pinStandBy, true);
    digitalWrite(pinAI1,      false);
    digitalWrite(pinAI2,      true);
    digitalWrite(pinBI1,      false);
    digitalWrite(pinBI2,      true);
    analogWrite (pinPWMA, 50);
    analogWrite (pinPWMB, 50);
    }
  void stop_wheels(){
    digitalWrite(pinStandBy, false);}
  void go_back(){ // go backwards
    digitalWrite(pinStandBy, true);
    digitalWrite(pinAI1,      false);
    digitalWrite(pinAI2,      true);
    digitalWrite(pinBI1,      true);
    digitalWrite(pinBI2,      false);
    analogWrite (pinPWMA, 100);
    analogWrite (pinPWMB, 100);
    }
void forward(int SPEED_left, int SPEED_right){ 
  digitalWrite(pinStandBy, true);
  digitalWrite(pinAI1,      true);
  digitalWrite(pinAI2,      false);
  digitalWrite(pinBI1,      false);
  digitalWrite(pinBI2,      true);
  analogWrite (pinPWMA, SPEED_right); // TO INCREASE THIS TO TURN LEFT
  analogWrite (pinPWMB, SPEED_left);
}
  



// float front_sensor(){
//   float duration, cm;
  
//   pinMode(pingPin_front, OUTPUT);
//   digitalWrite(pingPin_front, LOW);
  
//   delayMicroseconds(2);
//   digitalWrite(pingPin_front, HIGH);
  
//   delayMicroseconds(10);
//   digitalWrite(pingPin_front, LOW);
  
//   pinMode(echoPin_front, INPUT);
  
//   duration = pulseIn(echoPin_front, HIGH);
  
//    cm = ((duration / 2) / 29);
  
//   Serial.print(cm,4); Serial.print("cm_f  ");
//   delay(100);


//   return cm;}
// float back_sensor(){
//   float duration, cm;
  
//   pinMode(pingPin_back, OUTPUT);
//   digitalWrite(pingPin_back, LOW);
  
//   delayMicroseconds(2);
//   digitalWrite(pingPin_back, HIGH);
  
//   delayMicroseconds(10);
//   digitalWrite(pingPin_back, LOW);
  
//   pinMode(echoPin_back, INPUT);
  
//   duration = pulseIn(echoPin_back, HIGH);
  
//   cm = ((duration / 2) / 29);
  
//   //Serial.print(cm); Serial.println("cm");
//   delay(100);


//   return cm;
//   }
float right_sensor(){
  float duration, cm;
  
  pinMode(pingPin_right, OUTPUT);
  digitalWrite(pingPin_right, LOW);
  
  delayMicroseconds(2);
  digitalWrite(pingPin_right, HIGH);
  
  delayMicroseconds(10);
  digitalWrite(pingPin_right, LOW);
  
  pinMode(echoPin_right, INPUT);
  
  duration = pulseIn(echoPin_right, HIGH);
  
  cm = ((duration / 2) / 29);
  
  Serial.print(cm,8); Serial.println("cm_r");
  delay(100);


  return cm;
  }
float left_sensor(){
  float duration, cm;
  
  pinMode(pingPin_left, OUTPUT);
  digitalWrite(pingPin_left, LOW);
  
  delayMicroseconds(2);
  digitalWrite(pingPin_left, HIGH);
  
  delayMicroseconds(10);
  digitalWrite(pingPin_left, LOW);
  
  pinMode(echoPin_left, INPUT);
  
  duration = pulseIn(echoPin_left, HIGH);
  
   cm = ((duration / 2) / 29);
  
  Serial.print(cm); Serial.println("cm_l");
  delay(100);


  return cm;
  }
long microsecondsToCentimeters(long microseconds){
  return microseconds / 29 / 2;
  }


void PID_TO_THE_WALL(float REF_d,int min,int SPEED){
  REF=REF_d;
  NORM_SPEED_L=SPEED;
  NORM_SPEED_R=SPEED;
  
  
K=3;
 right_s=right_sensor();
//Serial.print("I m going forward ");



if(right_s<min)
{
  right_s=REF; // AVOIDING READING BELOW certain VALUE
}
if(right_s>(REF-0.5))
{
  if (right_s<(REF+0.5))
  {
    right_s=REF; // ADDING A GAP WHERE WE GO TO GO
   // K=1; // CURRENT
  }
}




if(right_s<(REF)) //we are too close to the wall we need to move to the left 
  {
    error=REF-right_s;
    error=error*K;
    
    speed_right=NORM_SPEED_R+error;
    speed_left=NORM_SPEED_L-error;

    if(speed_right>130)
    {
      speed_right=130;
    }
    if(speed_right<0)
    {
      speed_right=0;
    }
    
    if(speed_left>130)
    {
      speed_left=130;
    }
    if(speed_left<0)
    {
      speed_left=0;
    }
  //  Serial.print("TURNING TO THE LEFT SPEED OF right:   ");
  //  Serial.print(speed_right);
  //  Serial.print("SPEED OF left:   ");
 //   Serial.print(speed_left);
  //  Serial.print("   ");
    

    forward(speed_left,speed_right);

  }
  else if(right_s>(REF)) //we are too far from the wall we need to move to the right
    {
    error=REF-right_s;
    error=error*K;
    
    speed_right=NORM_SPEED_R+error;
    speed_left=NORM_SPEED_L-error;
    
    if(speed_right>130)
    {
      speed_right=130;
    }
    if(speed_right<0)
    {
      speed_right=0;
    }
    
    if(speed_left>130)
    {
      speed_left=130;
    }
    if(speed_left<0)
    {
      speed_left=0;
    }

    forward(speed_left,speed_right);
  //  Serial.print("TURNING TO THE RIGHT   SPEED OF right:   ");
   // Serial.print(speed_right);
  //  Serial.print("SPEED OF left:   ");
  //  Serial.print(speed_left);
  //   Serial.print("   ");
    }
    
    
  }
  bool TURN_90_Right()
  {
     bool TURN_DONE=0;
    
    if(millis()-t0>20)
  {
    
    
    t0=millis();
    Serial.print("\nEncoder count right = ");
    Serial.print(enc_countR);
    Serial.print("\nEncoder angle right = ");
    Serial.print(enc_angleR);
    Serial.print("\nEncoder count left = ");
    Serial.print(enc_countL);
    Serial.print("\nEncoder angle left = ");
    Serial.print(enc_angleL);
    delay(5);

    // Set the speed of the motor by adjusting the pwmValue
  
    
    // Write the AI1 and AI2 values to the configuration pins
    digitalWrite(pinAI1, AI1);
    digitalWrite(pinAI2, AI2);
    digitalWrite(pinBI1, BI1);
    digitalWrite(pinBI2, BI2);

    // Write the pwnValue to the PWM pin
    analogWrite(pinPWMA, pwmValue);
    analogWrite(pinPWMB, pwmValue);
    
    if(enc_countL<-1540)
    {
          
          return 1;
    }
    //return 1;
    rotate_right();
    return TURN_DONE;
    
    

    // Display the board variable status to the Serial Monitor
  

    // wait 2000ms
  


  }
}
bool TURN_90_Left()
  {
     bool TURN_DONE=0;
    
    if(millis()-t0>20)
  {
    
    
    t0=millis();
    Serial.print("\nEncoder count right = ");
    Serial.print(enc_countR);
    Serial.print("\nEncoder angle right = ");
    Serial.print(enc_angleR);
    Serial.print("\nEncoder count left = ");
    Serial.print(enc_countL);
    Serial.print("\nEncoder angle left = ");
    Serial.print(enc_angleL);
    delay(5);

    // Set the speed of the motor by adjusting the pwmValue
  
    
    // Write the AI1 and AI2 values to the configuration pins
    digitalWrite(pinAI1, AI1);
    digitalWrite(pinAI2, AI2);
    digitalWrite(pinBI1, BI1);
    digitalWrite(pinBI2, BI2);

    // Write the pwnValue to the PWM pin
    analogWrite(pinPWMA, pwmValue);
    analogWrite(pinPWMB, pwmValue);
    
    if(enc_countL>1270) // 1260 previosuly
    {
          
          return 1;
    }
    //return 1;
    rotate_left();
    return TURN_DONE;
    
    

    // Display the board variable status to the Serial Monitor
  

    // wait 2000ms
  


  }
}


bool TURN_90_Left1()
  {
     bool TURN_DONE=0;
    
    if(millis()-t0>20)
  {
    
    
    t0=millis();
    Serial.print("\nEncoder count right = ");
    Serial.print(enc_countR);
    Serial.print("\nEncoder angle right = ");
    Serial.print(enc_angleR);
    Serial.print("\nEncoder count left = ");
    Serial.print(enc_countL);
    Serial.print("\nEncoder angle left = ");
    Serial.print(enc_angleL);
    delay(10);

    // Set the speed of the motor by adjusting the pwmValue
  
    
    // Write the AI1 and AI2 values to the configuration pins
    digitalWrite(pinAI1, AI1);
    digitalWrite(pinAI2, AI2);
    digitalWrite(pinBI1, BI1);
    digitalWrite(pinBI2, BI2);

    // Write the pwnValue to the PWM pin
    analogWrite(pinPWMA, pwmValue);
    analogWrite(pinPWMB, pwmValue);
    
    if(enc_countL>1290)
    {
          
          return 1;
    }
    //return 1;
    rotate_left();
    return TURN_DONE;
    
    

    // Display the board variable status to the Serial Monitor
  

    // wait 2000ms
  


  }
}


bool TURN_90_Left2()
  {
     bool TURN_DONE=0;
    
    if(millis()-t0>20)
  {
    
    
    t0=millis();
    Serial.print("\nEncoder count right = ");
    Serial.print(enc_countR);
    Serial.print("\nEncoder angle right = ");
    Serial.print(enc_angleR);
    Serial.print("\nEncoder count left = ");
    Serial.print(enc_countL);
    Serial.print("\nEncoder angle left = ");
    Serial.print(enc_angleL);
    delay(10);

    // Set the speed of the motor by adjusting the pwmValue
  
    
    // Write the AI1 and AI2 values to the configuration pins
    digitalWrite(pinAI1, AI1);
    digitalWrite(pinAI2, AI2);
    digitalWrite(pinBI1, BI1);
    digitalWrite(pinBI2, BI2);

    // Write the pwnValue to the PWM pin
    analogWrite(pinPWMA, pwmValue);
    analogWrite(pinPWMB, pwmValue);
    
    if(enc_countL>1450)
    {
          
          return 1;
    }
    //return 1;
    rotate_left();
    return TURN_DONE;
    
    

    // Display the board variable status to the Serial Monitor
  

    // wait 2000ms
  


  }
}


bool TURN_180()
  {
     bool TURN_DONE=0;
    
    if(millis()-t0>20)
  {
    
    
    t0=millis();
    Serial.print("\nEncoder count right = ");
    Serial.print(enc_countR);
    Serial.print("\nEncoder angle right = ");
    Serial.print(enc_angleR);
    Serial.print("\nEncoder count left = ");
    Serial.print(enc_countL);
    Serial.print("\nEncoder angle left = ");
    Serial.print(enc_angleL);
    delay(5);

    // Set the speed of the motor by adjusting the pwmValue
  
    
    // Write the AI1 and AI2 values to the configuration pins
    digitalWrite(pinAI1, AI1);
    digitalWrite(pinAI2, AI2);
    digitalWrite(pinBI1, BI1);
    digitalWrite(pinBI2, BI2);

    // Write the pwnValue to the PWM pin
    analogWrite(pinPWMA, pwmValue);
    analogWrite(pinPWMB, pwmValue);
    
    if(enc_countL>2750)
    {
          
          return 1;
    }
    //return 1;
    rotate_left();
    return TURN_DONE;
    
    

    // Display the board variable status to the Serial Monitor
  

    // wait 2000ms
  


  }
}


void channelAR()
{
  if(digitalRead(PINAR)==HIGH)
  {
    if(digitalRead(PINBR)==LOW)
    {
      enc_countR=enc_countR+1;
    }
    else
    {
      enc_countR=enc_countR-1;
    }
  }
  else
  {
    if(digitalRead(PINBR)==HIGH)
    {
      enc_countR=enc_countR+1;
    }
    else
    {
      enc_countR=enc_countR-1;
    }
  }
  enc_angleR=360*(float)enc_countR/ENC_K;
}

void channelBR()
{
  if(digitalRead(PINBR)==HIGH)
  {
    if(digitalRead(PINAR)==HIGH)
    {
      enc_countR=enc_countR+1;
    }
    else
    {
      enc_countR=enc_countR-1;
    }
  }
  else
  {
    if(digitalRead(PINAR)==LOW)
    {
      enc_countR=enc_countR+1;
    }
    else
    {
      enc_countR=enc_countR-1;
    }
  }
  enc_angleR=360*(float)enc_countR/ENC_K;
}

void channelAL()
{
  if(digitalRead(PINAL)==HIGH)
  {
    if(digitalRead(PINBL)==LOW)
    {
      enc_countL=enc_countL+1;
    }
    else
    {
      enc_countL=enc_countL-1;
    }
  }
  else
  {
    if(digitalRead(PINBL)==HIGH)
    {
      enc_countL=enc_countL+1;
    }
    else
    {
      enc_countL=enc_countL-1;
    }
  }
  enc_angleL=360*(float)enc_countL/ENC_K;
}

void channelBL()
{
  
  if(digitalRead(PINBL)==HIGH)
  {
    if(digitalRead(PINAL)==HIGH)
    {
      enc_countL=enc_countL+1;
    }
    else
    {
      enc_countL=enc_countL-1;
    }
  }
  else
  {
    if(digitalRead(PINAL)==LOW)
    {
      enc_countL=enc_countL+1;
    }
    else
    {
      enc_countL=enc_countL-1;
    }
  }
  enc_angleL=360*(float)enc_countL/ENC_K;
}
void stop()
{
    digitalWrite(pinStandBy, true);
  digitalWrite(pinAI1,      false);
  digitalWrite(pinAI2,      false);
  digitalWrite(pinBI1,      false);
  digitalWrite(pinBI2,      false);
  analogWrite (pinPWMA, 200);
  analogWrite (pinPWMB, 200);
}
  


// FUNCTION TO MOVE FROM A TO B within 4s 
void A_to_B( float S1_s, float S2_s, float S3_s, float S1_e , float S2_e , float S3_e, int delay_1, int steps)
{
float step_S1,step_S2,step_S3;
int total_movement_time=delay_1;
int step_number=steps;
float each_step_delay=total_movement_time/(step_number+1); // it must be smaller to make sure we will do it within total delay time
int x;
// CALCULATING STEP SIZE
step_S1=(S1_e-S1_s)/step_number;
step_S2=(S2_e-S2_s)/step_number;
step_S3=(S3_e-S3_s)/step_number;

// TAKING TIMES
unsigned long previousMillis = millis();
unsigned long currentMillis = millis();
unsigned long doing_task_milis;

// CHECKING has 4 second passed
x=0;
while(currentMillis<=(previousMillis+total_movement_time))
{
  currentMillis = millis();
  if(x!=step_number) // MAKING SURE WE DO NOT DO MORE STEPS
  {
  doing_task_milis=currentMillis ;
     // CALCULATING NEW RAD ANGLE for each servo
    S3_s=S3_s+step_S3;
    S2_s=S2_s+step_S2;
    S1_s=S1_s+step_S1;
    // CHECKING LIMIT of S3
    //S3_s=check_limit_S3(S3_s,S2_s);
    if (S2_s<=-1.06)
    {
      S2_s=-1.06;
    }
   // PRINTING VALUES
    S1.writeMicroseconds(S1_s);
    S2.writeMicroseconds(S2_s);
    S3.writeMicroseconds(S3_s);
    
    
    
    Serial.print("  STEP: BASE:  ");
    Serial.print(S1_s);
     Serial.print("  ELBOW:  ");
    Serial.print(S2_s);
     Serial.print("  WRIST:  ");
    Serial.print(S3_s);
    // MOVING SERVO
    
    while(doing_task_milis<=(currentMillis+each_step_delay)) // check has step_delay_times passed
    {
    doing_task_milis = millis();
    //Serial.println("  I'm waiting  ");
    }
    x++;
    Serial.print("  STEP NUMBER:  ");
    Serial.println(x);
  }
  else
  {
 
  }
  
  
  
   
    
  }
  
}


  