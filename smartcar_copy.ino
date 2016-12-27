//Robotic Tracking and Folowing Project 
//     version#1
//      change:                                                                   modify time: 
//     This is good for distance sensoring and motor steering control             6/1/2016
//     this is the code for only distance, motor control and diffirential speed   8/21/2016
//     version #2
//     This is test for web control
//     check_motor() is added into code,which is only the testing for motor contol  

//     version #3                                                                 9/21/2016
//     sys_count has been modified for car and filter.
//     camera data is simulated by analog input A0  
//     servo_tuning is add for control the servo  
//     I2C library need to be add in arduino for (servo driver)
// ------> http://www.adafruit.com/products/815
#define SERVOMIN0  110 // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX0  440// this is the 'maximum' pulse length count (out of 4096)


#define USB_USBCON
#include <Servo.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include "Adafruit_PWMServoDriver.h"
#include <stdint.h>

uint32_t a; 
//for ros system
#include <ros.h>
#include <std_msgs/Int8.h>
#include <std_msgs/UInt16.h>
int flag = 0;
int x_value = 90;
int y_value = 90;
int depth = 0;
int start = 90;
int last=90;
int dis=0;
int target=0;
Servo myservo;  // create servo object to control a servo
int pulselength0 = SERVOMIN0;
std_msgs::Int8 aws_msg;
ros::NodeHandle  nh;
ros::Publisher aws("lock_r", &aws_msg);

void messageServo(const std_msgs::UInt16& servo_data){
  target = servo_data.data;
}

void messageServoDis(const std_msgs::UInt16& servo_data){
  dis = servo_data.data;
}

ros::Subscriber<std_msgs::UInt16> sub2("servo_r", messageServo);
ros::Subscriber<std_msgs::UInt16> sub3("servo_dis", messageServoDis);



         
// data recieve from python transimit
char buffer[4];
int value=0;
int count=1;

//define camera feedback 
int camera_data=90; // camera data to control servo and 
                    // vary from 0--180 degree 

//define for distance sensor
//#define trigPin 13    //pin for distance sensor
//#define echoPin 12    //pin for distance sensor
#define trigPin 4    //pin for distance sensor
#define echoPin 2    //pin for distance sensor
int Actual_Dis=0;//define the inital distance sensor reading
int Target_Dis=40;//set the designed distance to be 70 cm.
int dis_noise=3; //distance noise range. 


//define for filter
int numReading = 2;  //define the filter size for distance sensor
int readings[2];     //create the array using filter size for distance sensor
int count_sys_filter = 0;        // the index of the current reading

////define for servo

//int oldpos=0;  //define the old position for serve motor
//int pos = 0;    // variable to store the servo position

// i2c defined servo control
// called this way, it uses the default address 0x40
// you can also call it with a different address you want
//Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x41);

#define SERVOMIN1  130 // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX1  455// this is the 'maximum' pulse length count (out of 4096)

#define SERVOMIN2  120 // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX2  430// this is the 'maximum' pulse length count (out of 4096)

//define the motor control
//Note: Set Sabertooth dip switches on the board for simplified serial and 9600 Baudrate. 
#define SABER_TX_PIN  13 //Digital pin 13 is serial transmit pin to sabertooth
#define SABER_RX_PIN  12 //Not used but still initialised, Digital pin 12 is serial receive from Sabertooth
#define SABER_BAUDRATE  9600 //set baudrate to match sabertooth dip settings

//simplifierd serial limits for each motor
#define SABER_MOTOR1_FULL_FORWARD 1
#define SABER_MOTOR1_FULL_REVERSE 127
#define SABER_MOTOR2_FULL_FORWARD 128
#define SABER_MOTOR2_FULL_REVERSE 255
// our servo # counter
//uint8_t servonum = 0; 
int pulselength1 = SERVOMIN1; 
int pulselength2 = SERVOMIN2; 

int degrees0 = 90 ;
int degrees1 = 90 ; 
int degrees2 = 90 ; 

#define SABER_ALL_STOP  0 //motor command to send when issuing full stop command
SoftwareSerial SaberSerial = SoftwareSerial (SABER_RX_PIN, SABER_TX_PIN );                              
void initSabertooth (void)  { //initialize software to communicate with sabertooth 
  pinMode ( SABER_TX_PIN, OUTPUT );
  SaberSerial.begin( SABER_BAUDRATE );
  SaberSerial.write((byte) 0);   //kill motors when first switched on
}

signed char Motor1percent;
signed char Motor2percent;
float Motor_speed = 0;
int   Motor_steer_L = 0;
int   Motor_steer_R = 0;
int count_sys_car = 0;        // the index of the current reading
int Motor1arr[2];
int Motor2arr[2];
int maxMotor_speedControl= 40;   //define the max motorpercentage 12v*percentage, 
                                 //max motor voltage is 6 v, Therefore,this value should below 60
                                 //sign deosen't mater here   
int minMotor_speedControl= 10;   //define the min motorpercentage 
                                 //min motor voltage is 1 v, Therefore,this value should above 10
                                 //sign deosen't mater here      
void setup() {
  myservo.attach(12);  // attaches the servo on pin 9 to the servo object
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);  
  
  nh.initNode();
  nh.subscribe(sub2);
  nh.subscribe(sub3);
  nh.advertise(aws);
  //pulselength0 = map(90, 0, 180, SERVOMIN0, SERVOMAX0);
  //pwm.setPWM(0, 0, pulselength0);
  initSabertooth(); //initialze saber motor controller
  SaberSerial.write((byte) 0);   //kill motors n first switched on
  
  Serial.begin(9600);



// set pwm for servos

  delay(10);
  
}

void loop() {
  nh.spinOnce();
  
  aws_msg.data = 1;
  aws.publish(&aws_msg);
  
  if (dis<400){
    set_motor(0,0);
  }else{
    if (target>183){
      set_motor(1, 0);
    }else if (target<180 && target>3){
      set_motor(-1, 0);
    }else set_motor(0, dis);
  }
  Serial.print(dis);
  aws_msg.data = 0;
  aws.publish(&aws_msg);
      
}





int read_distance(){
  long duration, distance;
  digitalWrite(trigPin, LOW);  // Added this line
  delayMicroseconds(2); // Added this line
  digitalWrite(trigPin, HIGH);

  delayMicroseconds(10); // Added this line
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  distance = (duration/2) / 29.1;
  if (distance >= 180|| distance <= 0){
    return -1;
  }
  else {
    return distance;
  }
  
}

int filted_Distance(){
  int dis=read_distance();
  while(dis==-1){
    dis=read_distance();
  }
  readings[count_sys_filter%numReading]=dis;
  int total=0;
  for (int i=0;i<numReading;i++){
     total =total+readings[(count_sys_filter-(i+1))%numReading];
  }
  int average= (float (total)/numReading);
  count_sys_filter++;  
  return average ;   

}


void set_motor(int flag, int dis)   {
  ////////////////////////////////////////////////////////////////////////////////
  unsigned char cSpeedVal_Motor1 = 0;
  unsigned char cSpeedVal_Motor2 = 0;
  
  if (flag==-1){
     Motor1percent = (signed char) -30;
     Motor2percent = (signed char) 30;
  }else if (flag==1){
     Motor1percent = (signed char) 30;
     Motor2percent = (signed char) -30;
  }else{
     if (dis<400){
       Motor1percent = (signed char) 0;
       Motor2percent = (signed char) 0;
     }else{
       Motor1percent = (signed char) -50;
       Motor2percent = (signed char) -50;
     
     }
  }

  if (Motor1percent > 100) Motor1percent = 100;
  if (Motor1percent < -100) Motor1percent = -100;
  if (Motor2percent > 100) Motor2percent = 100;
  if (Motor2percent < -100) Motor2percent = -100;
 
 
  cSpeedVal_Motor1 = map (
			  Motor1percent,
			  -100,
			  100,
			  SABER_MOTOR1_FULL_REVERSE,
			  SABER_MOTOR1_FULL_FORWARD);
                         
  cSpeedVal_Motor2 = map (
			  Motor2percent,
			  -100,
			  100,
			  SABER_MOTOR2_FULL_REVERSE,
			  SABER_MOTOR2_FULL_FORWARD);
                         
  SaberSerial.write ((byte) cSpeedVal_Motor1);
  SaberSerial.write ((byte) cSpeedVal_Motor2);
}

    
