#include <SoftwareSerial.h>
#include <Ultrasonic.h>

SoftwareSerial mySerial(3, 4);

//motor pin
#define L_RPWM 5
#define L_LPWM 6

#define R_RPWM 10
#define R_LPWM 11

//left
#define trigPin_1 5
#define echoPin_1 6
//front
#define trigPin_2 8
#define echoPin_2 9
//right
#define trigPin_3 11
#define echoPin_3 12

//PID
#define Kp_pin 0
#define Ki_pin 1
#define Kd_pin 2

//Ultrasonic
Ultrasonic left_distance(trigPin_1, echoPin_1);
Ultrasonic front_distance(trigPin_2, echoPin_2);
Ultrasonic right_distance(trigPin_3, echoPin_3);

//variables
double base_speed = 48;   //max 64
double diff_speed;

double d_left;  //(- theta)
double d_front;
double d_right; //(+ theta)

double theta;
double distance;

//PID___________
double Kp = 5;  
double Ki = 2;  
double Kd = 2;  
//______________
double error = 0;
double p_error = 0;
double PID_P;
double PID_I = 0;
double PID_D = 0;
double bound = 50; //??
double dt = 1;   //update every one second
//______________

//functions
void drive_car(){
  analogWrite(L_LPWM, base_speed - diff_speed);   //make sure it turns forward, if not try: L_RPWM
  analogWrite(R_LPWM, base_speed + diff_speed);   //make sure it turns forward, if not try: R_RPWM
}

void PID(){
  //Kp = (double) analogRead(Kp_pin)/500;
  //Ki = (double) analogRead(Ki_pin)/500;
  //Kd = (double) analogRead(Kd_pin)/500;
  
  error = theta;
  PID_P = error;
  PID_I += (error*dt); 
  PID_D = (error − p_error) / dt;
  p_error = error;
  
  if(PID_I > bound) PID_I = bound; 
  if(PID_I < (-1)*bound) PID_I = (-1)*bound; 
  
  diff_speed = (Kp*PID_P + Ki*PID_I + Kd*PID_D)/10;
  diff_speed = (diff_speed > 18) ? 18 : diff_speed;
  drive_car();
}

void stop_car(){
    analogWrite(L_RPWM, 0); 
    analogWrite(L_LPWM, 0); 
    analogWrite(R_RPWM, 0); 
    analogWrite(R_LPWM, 0);
    delay(200);
}

void object_detection(){
  d_left = left_distance.read();
  delay(20);
  d_front = front_distance.read();
  delay(20);
  d_right = right_distance.read();
}

//if no the sensor detects object, then turn around (left) till find object
void find_object(){
  //turn around (left)
  analogWrite(L_LPWM, 0);   //always set the other PWM Low !!!!
  analogWrite(L_RPWM, 10);  //turn back  (or L_LPWM) ?
  analogWrite(R_LPWM, 0);   //always set the other PWM Low !!!!
  analogWrite(R_RPWM, 10);  //turn forward   (or R_LPWM) ?
  do{
    d_left = (double) left_distance.read();
    delay(20);
  }while (d_left == 357);
  stop_car();
  object_detection();
}

//convert d1,d2,d3 into distance and theta
/*---------------------------*\
|             /\              |
|            /  \             |
|           /    \            |
|          /      \           |
|     d2  /        \  d1      |
|        /          \         |
|       /            \        |
|      / theta        \       |
|     /_)______________\      |
|            5 cm             |
\*___________________________*/
double cosines(double d1, double d2){
  return acos((d2*d2 + 25 - d1*d1)/(2*5*d2));
}

void object_location(){
  begin1:
  byte flag = 0;  //L,F,R
  if(d_left != 357)
    flag |= B00000100;
  if(d_front != 357)
    flag |= B00000010;
  if(d_right != 357)
    flag |= B00000001;

  switch (flag){
    case 0x00:
      find_object();
      goto begin1;
    case 0x01:    //only right
      theta = 18;   
      distance = d_right;
      break;
    case 0x02:    //only front
      theta = cosines(d_right, d_front);
      distance = d_front;
      break;
    case 0x03:    //right
      theta = cosines(d_right, d_front);
      distance = d_front;
      break;
    case 0x04:    //only left
      theta = -18;   
      distance = d_left;
      break;
    case 0x06:    //left
      theta = 0 - cosines(d_left, d_front);
      distance = d_front;
      break;
    case 0x07:    //front
      theta = cosines(d_right, d_front);
      distance = d_front;
      break;
  }
  
}


//main program start here

void setup() {
  Serial.begin(9600);
  mySerial.begin(115200);
  pinMode(L_LPWM, OUTPUT);
  pinMode(L_RPWM, OUTPUT);
  pinMode(R_LPWM, OUTPUT);
  pinMode(R_RPWM, OUTPUT);
  stop_car();
}

void loop() {
  object_detection();
  if (distance <= 20) {
    stop_car(); 
    goto End;
  }
  if (d_right == 357 && d_front == 357 && d_left == 357) find_object();
  object_location();
  PID();
  End:
  delay(int(dt*1000));
}
