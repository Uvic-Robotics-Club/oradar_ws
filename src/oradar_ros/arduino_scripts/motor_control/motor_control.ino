#include <ros.h>
#include <std_msgs/Int16.h>

// Define the pin your motor is connected to


ros::NodeHandle nh;

void motorCb(const std_msgs::Int16& cmd_msg){
 // digitalWrite(LED_BUILTIN, HIGH);  // Turn on the built-in LED when a message is received
//  delay(500);  // Wait for half a second
 // digitalWrite(LED_BUILTIN, LOW);  // Turn off the LED
  if(cmd_msg.data == 1){
  //    Serial.println("Received message on /motor_cmd"); 
      digitalWrite(6,HIGH);
      digitalWrite(5,LOW);
      digitalWrite(4,HIGH);
  } else {
    // Turn motor off
      digitalWrite(6,LOW);
      digitalWrite(5,LOW);
      digitalWrite(4,HIGH);
  }
}

ros::Subscriber<std_msgs::Int16> sub("/motor_cmd", motorCb);

void setup(){
  pinMode(LED_BUILTIN, OUTPUT); //intitalize built in LED
  pinMode(6,OUTPUT); //enA
  pinMode(5,OUTPUT); //INA
  pinMode(4,OUTPUT); // INB
  Serial.begin(57600);
  nh.initNode();
  nh.subscribe(sub);
}

void loop(){
  nh.spinOnce();
  delay(1);
}
