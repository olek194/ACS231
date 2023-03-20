/*
A simple navigation code


*/


//Variables (pins):
int w=16, x=5 ,y=10 ,z=5 ,q=54;
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


const int pingPin_right = 7; // Trigger Pin of Ultrasonic Sensor
const int echoPin_right = 53; // Echo Pin of Ultrasonic Sensor


const int pingPin_left = 13; // Trigger Pin of Ultrasonic Sensor
const int echoPin_left = 45; // Echo Pin of Ultrasonic Sensor


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
  rotate_right();
  while(front_sensor()>x)
  {
    forward();
    if(right_sensor()<y) //y is the safe distance from the wall
    {
      //increase right wheel speed
    }
  }
  stop_wheels();
  rotate_left();
  while(front_sensor()>z) //z is the stopping distance from the board
  {
    forward();
    if (front_sensor() > q || front_sensor() < q-5 ) //q is the distance between the Board n the Bot when the bot is about to enter the tunnel
    {
      if(left_sencor() < w) //w is the safe distance from the wall
      {
        //increase left wheel's magnitude of speed
      }
    }
  }


  delay(3000);
 
  rotate_left();
  while(front_sensor()>x)
  {
    forward();
    if(right_sensor()<y) //y is the same distance from the wall
    {
      //increase right wheel speed
    }
  }
  stop_wheels();
  rotate_left();
  while(front_sensor()>z) //z is the stopping distance from the board
  {
    forward();
    if (front_sensor() > q || front_sensor() < q-5 ) //q is the distance between the Board n the Bot when the bot is about to enter the tunnel
    {
      if(left_sencor() < w) //w is the same distance from the wall
      {
        //increase left wheel's magnitude of speed
      }
    }
  }
  
  stop_wheels();
  while(1){}
}//main ends

///Functions:
void forward(){  
  digitalWrite(pinStandBy, true);
  digitalWrite(pinAI1,      true);
  digitalWrite(pinAI2,      false);
  digitalWrite(pinBI1,      true);
  digitalWrite(pinBI2,      false);
  analogWrite (pinPWMA, 100);
  analogWrite (pinPWMB, 100);}
void go_back(){
  digitalWrite(pinStandBy, true);
  digitalWrite(pinAI1,      false);
  digitalWrite(pinAI2,      true);
  digitalWrite(pinBI1,      false);
  digitalWrite(pinBI2,      true);
  analogWrite (pinPWMA, 100);
  analogWrite (pinPWMB, 100);}
void stop_wheels(){
  digitalWrite(pinStandBy, false);}
void rotate_right(){ 
  digitalWrite(pinStandBy, true);
  digitalWrite(pinAI1,      false);
  digitalWrite(pinAI2,      true);
  digitalWrite(pinBI1,      true);
  digitalWrite(pinBI2,      false);
  analogWrite (pinPWMA, 50);
  analogWrite (pinPWMB, 50);
  delay(3980);
  stop_wheels();}
void rotate_left(){ 
  digitalWrite(pinStandBy, true);
  digitalWrite(pinAI1,      true);
  digitalWrite(pinAI2,      false);
  digitalWrite(pinBI1,      false);
  digitalWrite(pinBI2,      true);
  analogWrite (pinPWMA, 50);
  analogWrite (pinPWMB, 50);
  delay(3980);
  stop_wheels();}


int front_sensor(){
  long duration, cm;
  
  pinMode(pingPin_front, OUTPUT);
  digitalWrite(pingPin_front, LOW);
  
  delayMicroseconds(2);
  digitalWrite(pingPin_front, HIGH);
  
  delayMicroseconds(10);
  digitalWrite(pingPin_front, LOW);
  
  pinMode(echoPin_front, INPUT);
  
  duration = pulseIn(echoPin_front, HIGH);
  
  cm = microsecondsToCentimeters(duration);
  
  Serial.print(cm); Serial.println("cm_f");
  delay(100);


  return cm;}
int back_sensor(){
  long duration, cm;
  
  pinMode(pingPin_back, OUTPUT);
  digitalWrite(pingPin_back, LOW);
  
  delayMicroseconds(2);
  digitalWrite(pingPin_back, HIGH);
  
  delayMicroseconds(10);
  digitalWrite(pingPin_back, LOW);
  
  pinMode(echoPin_back, INPUT);
  
  duration = pulseIn(echoPin_back, HIGH);
  
  cm = microsecondsToCentimeters(duration);
  
  Serial.print(cm); Serial.println("cm");
  delay(100);


  return cm;}
int right_sensor(){
  long duration, cm;
  
  pinMode(pingPin_right, OUTPUT);
  digitalWrite(pingPin_right, LOW);
  
  delayMicroseconds(2);
  digitalWrite(pingPin_right, HIGH);
  
  delayMicroseconds(10);
  digitalWrite(pingPin_right, LOW);
  
  pinMode(echoPin_right, INPUT);
  
  duration = pulseIn(echoPin_right, HIGH);
  
  cm = microsecondsToCentimeters(duration);
  
  Serial.print(cm); Serial.println("cm_r");
  delay(100);


  return cm;}
int left_sencor(){
  long duration, cm;
  
  pinMode(pingPin_left, OUTPUT);
  digitalWrite(pingPin_left, LOW);
  
  delayMicroseconds(2);
  digitalWrite(pingPin_left, HIGH);
  
  delayMicroseconds(10);
  digitalWrite(pingPin_left, LOW);
  
  pinMode(echoPin_left, INPUT);
  
  duration = pulseIn(echoPin_left, HIGH);
  
  cm = microsecondsToCentimeters(duration);
  
  Serial.print(cm); Serial.println("cm_l");
  delay(100);


  return cm;}
long microsecondsToCentimeters(long microseconds){
  return microseconds / 29 / 2;}
/*int proportional_control(int target_distance_to_maintain, int average_speed, int sensor_input){
  int controled_speed=average_speed;
  switch (sensor_input)
  {
    case (target_distance_to_maintain-2):
      controled_speed=average_speed+10;
    break;
    case (target_distance_to_maintain):
      controled_speed=average_speed;
    break;
    case (target_distance_to_maintain+2):
      controled_speed=average_speed-10;
    break;
  }
  return controled_speed;}*/



//CODE END