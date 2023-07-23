#include <ros.h>
#include <std_msgs/Int16.h>
ros::NodeHandle nh;


void leftside_forward(int spin_speed){
  digitalWrite(2, HIGH); //pin 2 is direction on motor controller 1
  analogWrite(3, spin_speed); //pin 3 is the enable pin on the motor controller
}

void leftside_back(int spin_speed){
  digitalWrite(2, LOW); //pin 2 is direction on motor controller 1
  analogWrite(3, spin_speed); //pin 3 is the enable pin on the motor controller
}

void leftside_stop(int spin_speed){
  digitalWrite(2, LOW); //pin 2 is direction on motor controller 1
  analogWrite(3, spin_speed); //pin 3 is the enable pin on the motor controller
}

void rightside_forward(int spin_speed){
  digitalWrite(4, LOW); //pin 2 is direction on motor controller 1
  analogWrite(5, spin_speed); //pin 3 is the enable pin on the motor controller
}

void rightside_back(int spin_speed){
  digitalWrite(4, HIGH); //pin 2 is direction on motor controller 1
  analogWrite(5, spin_speed); //pin 3 is the enable pin on the motor controller
}

void rightside_stop(int spin_speed){
  digitalWrite(4, LOW); //pin 2 is direction on motor controller 1
  analogWrite(5, spin_speed); //pin 3 is the enable pin on the motor controller
}

void forward(){
  rightside_forward(125); //old value was 125
  leftside_forward(125);
}

void back(){
  rightside_back(125);
  leftside_back(125);
}

void spinleft(){
  rightside_forward(200);
  leftside_back(200);
  //delay(10);
}

void spinright(){
  rightside_back(200);
  leftside_forward(200);
  //delay(10);
}

void stopspin(){
  leftside_stop(0);
  rightside_stop(0);
}

void motorCb(const std_msgs::Int16& cmd_msg){
  digitalWrite(LED_BUILTIN, HIGH);  // Turn on the built-in LED when a message is received
  delay(500);  // Wait for half a second
  digitalWrite(LED_BUILTIN, LOW);  // Turn off the LED
//testing switch
  switch(cmd_msg.data){
    case 1:
    spinright();
    break;
     
    case 2:
    spinleft();
    break;
    
    case 3:
    forward();
    break;
  
    case 4:
    back();
    break;
   
    case 5:
    stopspin();
    break;   
  
}
}

ros::Subscriber<std_msgs::Int16> sub("/motor_cmd", motorCb);

void setup() {
  // put your setup code here, to run once:
  // Set motor control pins as output
  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);

  digitalWrite(2, LOW);
  digitalWrite(3, LOW);
  digitalWrite(4, LOW);
  digitalWrite(5, LOW);

  Serial.begin(57600);
  nh.initNode();
  nh.subscribe(sub);
  
}

void loop() {
 nh.spinOnce();
 delay(1);
 }
