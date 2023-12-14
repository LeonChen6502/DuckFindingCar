#include <SoftwareSerial.h>

//right
#define trigPin_1 11;
#define echoPin_1 12;
//left
#define trigPin_2 11;
#define echoPin_2 12;
//low
#define trigPin_3 11;
#define echoPin_3 12;


#define counterPin 2;

int L_RPWM = 5;  // Digital/PWM pin 5 to the RPWM on the BTS7960
int L_LPWM = 6;  // Digital/PWM pin 6 to the LPWM on the BTS7960

int R_RPWM = 10;
int R_LPWM = 11;

//RPM
unsigned int count=0;
unsigned long rpm_time;
unsigned int rpm;      
unsigned int grid_num = 20;

//main time
unsigned long start_time;
unsigned long duration_time;

//steps time
unsigned long steps_start_time;
unsigned long steps_dt;

//object detection data
double x = 0;
double y = 0;

//real distance
double z;
double l;

double p = 0;
double q = 0;
double h = 0;
double m = 0;
double f = 0;

//polar coordinate
double d;
double angle;

//car values
double dt;
double a;
double v;
int direc;

double v_L;
double v_R;
double car_l;

//path calculation
double k;
double theta;
double alpha;
double R_avg;
double d_c;
double d_new;
double angle_new;

SoftwareSerial mySerial(2, 3); // RX, TX pins for SoftwareSerial

void PID(){
    
}

void counter() {
   count++;
}
void RPM(){
    if (millis() - rpm_time >= 1000){   /* 每秒更新 */
 
     // 計算 rpm 時，停止計時
     detachInterrupt(0);

     // 偵測的格數count * (60 * 1000 / 一圈網格數20）/ 時間差) 
     rpm = (60 * 1000 / grid_num )/ (millis() - time2)* count;
     rpm_time = millis();
     count = 0;
 
     // 輸出至Console
     Serial.print("RPM = ");
     Serial.println(rpm,DEC);
     //Restart the interrupt processing
     attachInterrupt(0, counter, FALLING);
  }
}

void xy_to_zl(){
    y -= 48;
    y *= 1200/96;
    z = m - h*y/(y*p+q);
    x-=48;
    x *= 1200/96;
    l = abs(x*z/f);
    direc = x/abs(x);
    //z_range = z*10;     //not sure
    double z_max;
    double z_min;

    double z_read1 = read_distance(trigPin_1,echoPin_1);   //mid
    double z_read2 = read_distance(trigPin_2,echoPin_2);   //left
    double z_read3 = read_distance(trigPin_3,echoPin_3);   //right
    
    z_read2 = (z_read2 != -1) ? sqrt(z_read2*z_read2 - x*x) : -1;
    z_read3 = (z_read2 != -1) ? sqrt(z_read2*z_read3 - x*x) : -1;

    if(z_read1 <= z_max and z_read1 >= z_min and z_read1 != -1){
        z = z_read1;
    }
    else if(z_read2 <= z_max and z_read2 >= z_min and z_read2 != -1){
        z = z_read2;
    }
    else if(z_read3 <= z_max and z_read3 >= z_min and z_read3 != -1){
        z = z_read3;
    }
}

void zl_to_palor(){
    angle = atan(l/z);
    d = sqrt(l*l + z*z);
}


double read_distance(int t_pin,int e_pin){
    digitalWrite(t_pin, HIGH);
    delay(1);
    digitalWrite(t_pin, LOW);
    int Duration = pulseln(e_pin, HIGH);
    double cm = Duration*0.034/2;
    int cm_int = Duration*0.034/2;
    if(cm_int == 74.0){
        cm = -1;
    }
    return cm;
}

void drive_car(){
    v_L= a*car_l/v/2*direc;
    v_R= a*car_l/v/2*direc*(-1);
    
    //PID();

    v_L = v + v_L;
    v_R = v + v_R;
    
    steps_start_time = millis();
}

void stop_car(){
    analogWrite(L_RPWM, 0); 
    analogWrite(R_LPWM, 0);
}


void esp32_update(){
    delay(100);
    mySerial.write(100);
    while(!mySerial.available()){}
    int x_r = int(mySerial.read());
    int y_r = int(mySerial.read());
    if(x_r > 100){
        x = double(x_r);
        y = double(y_r);
    }else{
        x = double(y_r);
        y = double(x_r);
    }
    xy_to_zl();
    zl_to_palor();
}

void path_calculate(){
    theta = 2*M_PI*a/v*dt;                                  //compute the next posion of the car after dt
    alpha = angle - theta/2;
    R_avg = v*v/a;
    d_c = 2*R_avg*sin(theta/2);
    d_new = sqrt(d*d+d_c*d_c-2*d*d_c*cos(alpha));
    angle_new = asin(d/d_new*sin(alpha))-(theta/2);
    d = d_new;
    angle = angle_new;
}


void setup() {
    Serial.begin(9600); // Initialize the hardware serial port for debugging
    mySerial.begin(115200); // Initialize the software serial port

    pinMode(L_LPWM, OUTPUT);
    pinMode(L_RPWM, OUTPUT);
    pinMode(R_LPWM, OUTPUT);
    pinMode(R_RPWM, OUTPUT);
    stop_car();

    pinMode(trigPin_1, OUTPUT);
    pinMode(echoPin_1, INPUT);
    digitalWrite(trigPin_1, LOW);

    pinMode(trigPin_2, OUTPUT);
    pinMode(echoPin_2, INPUT);
    digitalWrite(trigPin_2, LOW);

    pinMode(trigPin_3, OUTPUT);
    pinMode(echoPin_3, INPUT);
    digitalWrite(trigPin_3, LOW);

    pinMode(counterPin, INPUT);
    attachInterrupt(0, counter, FALLING);
    count = 0;
    rpm = 0;
    rpm_time = 0;

    steps_dt = int(dt*1000)
}

void loop() {
    stop_car();
    esp32_update();
    bool first_time = 1;
    while (millis() - start_time < duration_time*1000){                                 //drive for time t and then stop to read the new value
        if((millis - steps_start_time >= steps_dt) or first_time){          //wait dt so that the car can drive for given time dt
            a = pow(angle,3)/ sqrt(d) * k * v;                      //compute the a for given status
            drive_car(); 
            path_calculate();                                              
            first_time = 0;
        }
    }
    
  
}