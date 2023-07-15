

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
  
}

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
  rightside_forward(125);
  leftside_forward(125);
}

void back(){
  rightside_back(125);
  leftside_back(125);
}

void spinleft(){
  rightside_back(200);
  leftside_forward(200);
}

void spinright(){
  rightside_forward(200);
  leftside_back(200);
}

void stopspin(){
  leftside_stop(0);
  rightside_stop(0);
}

void loop() {
  //rightside movement
  forward();
  delay(1000);
  stopspin();
  delay(1000);
  back();
  delay(1000);
  stopspin();
  delay(1000);
  spinleft();
  delay(1000);
  stopspin();
  delay(1000);
  spinright();
  delay(1000);
  stopspin();
 }
