# oradar_ws
Autonomous_rover 2.0
## Initial Runt Rover Arduino Tests July 11th
![Runt Rover Motor Configuration](/home/jetson/Downloads/20230711_174701.jpg)

The test arduino code for the runt rover will move each motor seperately to verify that the wiring is correct.
```arduino


void setup() {
  // put your setup code here, to run once:
  // Set motor control pins as output
  pinMode(1, OUTPUT);
  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(7, OUTPUT);
  pinMode(8, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(11, OUTPUT);
  pinMode(12, OUTPUT);

  digitalWrite(1, LOW);
  digitalWrite(2, LOW);
  digitalWrite(3, LOW);
  digitalWrite(4, LOW);
  digitalWrite(5, LOW);
  digitalWrite(6, LOW);
  digitalWrite(7, LOW);
  digitalWrite(8, LOW);
  digitalWrite(9, LOW);
  digitalWrite(10, LOW);
  digitalWrite(11, LOW);
  digitalWrite(12, LOW);

  
}


void loop() {
  // put your main code here, to run repeatedly:
//  INB controls the direction, INA controls the speed
//motor 1 control
  digitalWrite(2, LOW); //stop motor 1
  digitalWrite(3, LOW);
  delay(1000);            //spin motor 1 forward
  digitalWrite(2, HIGH); //pin 2 is direction on motor controller 1
  digitalWrite(3, HIGH); //pin 3 is the enable pin on the motor controller
  delay(1000);
  digitalWrite(2, LOW); //stop motor 1
  digitalWrite(3, LOW);
  delay(1000);
  digitalWrite(2, LOW); //spin motor 1 back
  digitalWrite(3, HIGH);
  delay(1000);
  digitalWrite(2, LOW); //stop motor 1
  digitalWrite(3, LOW);
  delay(1000);
//motor 2 control
  digitalWrite(4, LOW); //stop motor 2
  digitalWrite(5, LOW);
  delay(1000);            //spin motor 2 forward
  digitalWrite(4, HIGH); //pin 4 is direction on motor controller 1
  digitalWrite(5, HIGH); //pin 5 is the enable pin on the motor controller
  delay(1000);
  digitalWrite(4, LOW); //stop motor 2
  digitalWrite(5, LOW);
  delay(1000);
  digitalWrite(4, LOW); //spin motor 2 back
  digitalWrite(5, HIGH);
  delay(1000);
  digitalWrite(4, LOW); //stop motor 2
  digitalWrite(5, LOW);
  delay(1000);
//motor 3 control
  digitalWrite(6, LOW); //stop motor 3
  digitalWrite(7, LOW);
  delay(1000);            //spin motor 3 forward
  digitalWrite(6, HIGH); //pin 6 is direction on motor controller 1
  digitalWrite(7, HIGH); //pin 7 is the enable pin on the motor controller
  delay(1000);
  digitalWrite(6, LOW); //stop motor 3
  digitalWrite(7, LOW);
  delay(1000);
  digitalWrite(6, LOW); //spin motor 3 back
  digitalWrite(7, HIGH);
  delay(1000);
  digitalWrite(6, LOW); //stop motor 3
  digitalWrite(7, LOW);
  delay(1000);
//motor 4 control
  digitalWrite(8, LOW); //stop motor 4
  digitalWrite(9, LOW);
  delay(1000);            //spin motor 4 forward
  digitalWrite(8, HIGH); //pin 8 is direction on motor controller 1
  digitalWrite(9, HIGH); //pin 9 is the enable pin on the motor controller
  delay(1000);
  digitalWrite(8, LOW); //stop motor 4
  digitalWrite(9, LOW);
  delay(1000);
  digitalWrite(8, LOW); //spin motor 4 back
  digitalWrite(9, HIGH);
  delay(1000);
  digitalWrite(8, LOW); //stop motor 4
  digitalWrite(9, LOW);
  delay(1000);
//motor 5 control
  digitalWrite(10, LOW); //stop motor 5
  digitalWrite(11, LOW);
  delay(1000);            //spin motor 5 forward
  digitalWrite(10, HIGH); //pin 10 is direction on motor controller 1
  digitalWrite(11, HIGH); //pin 11 is the enable pin on the motor controller
  delay(1000);
  digitalWrite(10, LOW); //stop motor 5
  digitalWrite(11, LOW);
  delay(1000);
  digitalWrite(10, LOW); //spin motor 5 back
  digitalWrite(11, HIGH);
  delay(1000);
  digitalWrite(10, LOW); //stop motor 5
  digitalWrite(11, LOW);
  delay(1000);
//motor 6 control
  digitalWrite(12, LOW); //stop motor 6
  digitalWrite(13, LOW);
  delay(1000);            //spin motor 6 forward
  digitalWrite(12, HIGH); //pin 12 is direction on motor controller 1
  digitalWrite(13, HIGH); //pin 13 is the enable pin on the motor controller
  delay(1000);
  digitalWrite(12, LOW); //stop motor 6
  digitalWrite(13, LOW);
  delay(1000);
  digitalWrite(12, LOW); //spin motor 6 back
  digitalWrite(13, HIGH);
  delay(1000);
  digitalWrite(12, LOW); //stop motor 6
  digitalWrite(13, LOW);
  delay(1000);
 }
```
