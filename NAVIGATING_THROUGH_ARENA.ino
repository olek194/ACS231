/*
A simple navigation code


*/


//Variables (pins):
float x=10.2 ,y=6.6 ,z=80 ,q=54, w=5;
const int pinAI1 = 9;       // Pin allocation for AI1
const int pinAI2 = 6;       // Pin allocation for AI2
const int pinBI1 = 27;       // Pin allocation for AI1
const int pinBI2 = 29;       // Pin allocation for AI2
const int pinPWMA = 3;       // Pin allocation for the PWM pin
const int pinPWMB = 10;       // Pin allocation for the PWM pin
const int pinStandBy = 25;   // Pin allocation for the standby pin


const int pingPin_front = 8; // Trigger Pin of Ultrasonic Sensor
const int echoPin_front = 49; // Echo Pin of Ultrasonic Sensor


const int pingPin_back = 35; // Trigger Pin of Ultrasonic Sensor
const int echoPin_back = 36; // Echo Pin of Ultrasonic Sensor


const int pingPin_right = 13; // Trigger Pin of Ultrasonic Sensor
const int echoPin_right = 45; // Echo Pin of Ultrasonic Sensor


const int pingPin_left = 7; // Trigger Pin of Ultrasonic Sensor
const int echoPin_left = 53; // Echo Pin of Ultrasonic Sensor
float front=20;
float REF=x;
float right_s,NORM_SPEED=100,K=10;
int speed_right;
int speed_left;
int error;



void setup()
{
  Serial.begin(9600);


  pinMode(pinStandBy, OUTPUT);
  pinMode(pinAI1, OUTPUT);
  pinMode(pinAI2, OUTPUT);
  pinMode(pinBI1, OUTPUT);
  pinMode(pinBI2, OUTPUT);
  pinMode(pinPWMA, OUTPUT);
  pinMode(pinPWMB, OUTPUT);
}


void loop()
{  
   Serial.print("HELLO  ");
  stop_wheels();
 Serial.print("HELLO  ");
delay(2000);
forward(100,100);
delay(500);
stop_wheels();


  
 






  
  rotate_right();
  while(right_sensor()>x)
  {
   Serial.print("I m turning right  ");
  }
 right_s=right_sensor(); // taking the first measurement
 forward(100,100);

 // GOING FORWARD AFTER INITAL TURN
  while(front_sensor()>y)
  {
    PID_TO_THE_WALL();
  }
  // we 

// WE NEED TO TURN TO THE LEFT
  
  rotate_left();
  int ok=1;
  while(ok==1){
    Serial.print("This is the front distane::   ");
    ok=1;
  while(ok==1)
  {
    if(right_sensor()>22)
    {
      while(ok==1){
          if(right_sensor()<17){
          ok=0;
        }
      }
        
    }
  }
    Serial.print("I m turning left  ");
Serial.println(" ");
  }
  // GOING THORUG THE TUNNEL
  REF=16.3;
  while(front_sensor()>8)
  {
    PID_TO_THE_WALL();
  }
  //STOPPING AT THE BOARD
  stop_wheels();
  delay(3000);
  //ROTATING TO THE LEFT
  rotate_left();
  delay(2000);
  // GOING FORWARD
  while(front_sensor()>8)
  {
    PID_TO_THE_WALL();
  }
  // ROTATING TO THE LEFT AGAIN
  rotate_left();
  ok=1;
  while(ok==1)
  {
    if(right_sensor()>22)
    {
      while(ok==1){
          if(right_sensor()<17){
          ok=0;
        }
      }
        
    }
  }
  //GOING FORWARD
forward(100,100);
while(front_sensor()>8)
{
  PID_TO_THE_WALL();
}
stop_wheels();
delay(10000);
  
}//main ends

///Functions:
void rotate_left(){  
  digitalWrite(pinStandBy, true);
  digitalWrite(pinAI1,      true);
  digitalWrite(pinAI2,      false);
  digitalWrite(pinBI1,      true);
  digitalWrite(pinBI2,      false);
  analogWrite (pinPWMA, 100);
  analogWrite (pinPWMB, 100);
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
  



float front_sensor(){
  float duration, cm;
  
  pinMode(pingPin_front, OUTPUT);
  digitalWrite(pingPin_front, LOW);
  
  delayMicroseconds(2);
  digitalWrite(pingPin_front, HIGH);
  
  delayMicroseconds(10);
  digitalWrite(pingPin_front, LOW);
  
  pinMode(echoPin_front, INPUT);
  
  duration = pulseIn(echoPin_front, HIGH);
  
   cm = ((duration / 2) / 29);
  
  Serial.print(cm,4); Serial.print("cm_f  ");
  delay(100);


  return cm;}
float back_sensor(){
  float duration, cm;
  
  pinMode(pingPin_back, OUTPUT);
  digitalWrite(pingPin_back, LOW);
  
  delayMicroseconds(2);
  digitalWrite(pingPin_back, HIGH);
  
  delayMicroseconds(10);
  digitalWrite(pingPin_back, LOW);
  
  pinMode(echoPin_back, INPUT);
  
  duration = pulseIn(echoPin_back, HIGH);
  
  cm = ((duration / 2) / 29);
  
  //Serial.print(cm); Serial.println("cm");
  delay(100);


  return cm;}
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


  return cm;}
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


  return cm;}
long microsecondsToCentimeters(long microseconds){
  return microseconds / 29 / 2;}

void PID_TO_THE_WALL(){
  right_s=right_sensor(); // taking the first measurement
Serial.print("I m going forward ");
if(right_s<(REF)) //we are too close to the wall we need to move to the left 
  {
    error=REF-right_s;
    error=error*K;
    
    speed_right=NORM_SPEED+error;
    speed_left=NORM_SPEED-error;

    if(speed_right>150)
    {
      speed_right=150;
    }
    if(speed_right<0)
    {
      speed_right=0;
    }
    
    if(speed_left>150)
    {
      speed_left=150;
    }
    if(speed_left<0)
    {
      speed_left=0;
    }
    Serial.print("TURNING TO THE LEFT SPEED OF right:   ");
    Serial.print(speed_right);
    Serial.print("SPEED OF left:   ");
    Serial.print(speed_left);
     Serial.print("   ");
    

    forward(speed_left,speed_right);

  }
  else if(right_s>(REF)) //we are too far from the wall we need to move to the right
    {
    error=REF-right_s;
    error=error*K;
    
    speed_right=NORM_SPEED+error;
    speed_left=NORM_SPEED-error;
    
    if(speed_right>150)
    {
      speed_right=150;
    }
    if(speed_right<0)
    {
      speed_right=0;
    }
    
    if(speed_left>150)
    {
      speed_left=150;
    }
    if(speed_left<0)
    {
      speed_left=0;
    }

    forward(speed_left,speed_right);
    Serial.print("TURNING TO THE RIGHT   SPEED OF right:   ");
    Serial.print(speed_right);
    Serial.print("SPEED OF left:   ");
    Serial.print(speed_left);
     Serial.print("   ");
    }
    
    
  }
  
  
  

